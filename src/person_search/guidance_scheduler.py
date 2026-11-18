"""Bounded task scheduling, after entity association.

This module never associates people, commands motion, or reads ground truth.
Only material decision revisions enter request(); ordinary camera frames do not.
The caller owns a single instance for the task's global planning resource limit.
Audit formatting and I/O belong to the consumer, outside this lock.
"""
from collections import deque
from dataclasses import dataclass
from enum import IntFlag
import math
from threading import RLock


class Reason(IntFlag):
    ELIGIBLE = 1
    REACQUIRE = 2
    VISIBILITY = 4
    CONTRADICTION = 8
    MAP = 16
    BUDGET = 32
    ALIAS = 64


@dataclass(frozen=True)
class Request:
    entity: str
    generation: int
    first_requested_s: float
    reasons: Reason
    eligible_generation: int | None = None


@dataclass(frozen=True)
class Work:
    token: int
    request: Request
    started_s: float


@dataclass
class _Entity:
    generation: int = 0
    confirmed: bool = False


class GuidanceScheduler:
    def __init__(self, *, max_entities=64, max_pending=64, max_inflight=1,
                 audit_capacity=256):
        limits=(max_entities,max_pending,max_inflight)
        if any(isinstance(n,bool) or not isinstance(n,int) or n<1 for n in limits):
            raise ValueError('positive integer scheduler limits required')
        if isinstance(audit_capacity,bool) or not isinstance(audit_capacity,int):
            raise ValueError('integer audit capacity required')
        self.max_entities=max_entities
        self.max_pending=max_pending
        self.max_inflight=max_inflight
        self._parent={}
        self._entities={}
        self._revisions={}
        self._pending={}
        self._active={}
        self._serial=0
        self._token=0
        self._audit=deque(maxlen=audit_capacity)
        self.audit_dropped=0
        self._clock=0.
        self._lock=RLock()

    def _time(self,now):
        if not math.isfinite(now) or now<self._clock:
            raise ValueError('scheduler clock must be finite, nonnegative and monotonic')
        self._clock=now

    def _log(self,event,entity,now):
        if len(self._audit)==self._audit.maxlen:
            self.audit_dropped+=1
        self._audit.append((event,entity,now))

    def _root(self,entity):
        while self._parent[entity]!=entity:
            entity=self._parent[entity]
        return entity

    def _register(self,entity):
        if not isinstance(entity,str) or not entity:
            raise ValueError('nonempty canonical entity ID required')
        if entity not in self._parent:
            if len(self._parent)>=self.max_entities:
                return False
            self._parent[entity]=entity
            self._entities[entity]=_Entity()
        return True

    def _advance(self,root):
        self._serial+=1
        self._entities[root].generation=self._serial
        return self._serial

    def request(self,entity,revision,reasons,now):
        """Return accepted/coalesced/duplicate/full/confirmed; no solver is run.

        revision is monotonic per input entity, not a frame counter. Eligibility
        is latched in the pending request; a later visibility event cannot erase it.
        """
        if isinstance(revision,bool) or not isinstance(revision,int) or revision<1:
            raise ValueError('positive integer decision revision required')
        reasons=Reason(reasons)
        if not reasons or int(reasons)&~127:
            raise ValueError('unsupported decision reason')
        with self._lock:
            self._time(now)
            if not self._register(entity):
                self._log('entity_capacity',entity,now)
                return 'full'
            root=self._root(entity)
            if self._entities[root].confirmed:
                self._log('confirmed',root,now)
                return 'confirmed'
            if revision<=self._revisions.get(entity,0):
                self._log('duplicate',root,now)
                return 'duplicate'
            old=self._pending.get(root)
            # Even a change rejected by queue capacity invalidates a running result.
            generation=self._advance(root)
            if old is None and len(self._pending)>=self.max_pending:
                # Leave the revision available to retry after the queue drains.
                self._log('queue_capacity',root,now)
                return 'full'
            self._revisions[entity]=revision
            eligible=(old.eligible_generation if old else None)
            if eligible is None and reasons&Reason.ELIGIBLE:
                eligible=generation
            self._pending[root]=Request(root,generation,
                old.first_requested_s if old else now,
                (old.reasons if old else Reason(0))|reasons,eligible)
            result='coalesced' if old else 'accepted'
            self._log(result,root,now)
            return result

    def start(self,now):
        with self._lock:
            self._time(now)
            if not self._pending or len(self._active)>=self.max_inflight:
                return None
            active_roots={self._root(w.request.entity) for w in self._active.values()}
            for root,request in self._pending.items():
                if root in active_roots:
                    continue
                del self._pending[root]
                self._token+=1
                work=Work(self._token,request,now)
                self._active[work.token]=work
                self._log('started',root,now)
                return work
            return None

    def finish(self,work,now,*,map_valid=True,ledger_valid=True):
        """Release one resource; True only means the task snapshot is current.

        This is NOT trajectory authorization. The caller must still validate
        current map/ledger and all existing safety and handoff gates.
        """
        with self._lock:
            self._time(now)
            if self._active.get(work.token)!=work:
                self._log('unknown_completion',work.request.entity,now)
                return False
            del self._active[work.token]
            root=self._root(work.request.entity)
            # Confirmation and aliasing advance the root generation under this lock.
            current=(self._entities[root].generation==work.request.generation and
                     map_valid and ledger_valid)
            self._log('current_completion' if current else 'stale_completion',root,now)
            return bool(current)

    def confirm(self,entity,now):
        """Called only by the common confirmation/identity authority."""
        with self._lock:
            self._time(now)
            if not self._register(entity):
                self._log('entity_capacity',entity,now)
                return False
            root=self._root(entity)
            self._entities[root].confirmed=True
            self._advance(root)
            self._pending.pop(root,None)
            self._log('confirmed',root,now)
            return True

    def alias(self,source,target,now):
        """Merge known IDs from the identity authority, never geometric guesses.

        Running work is not force-cancelled. Its resource remains occupied until
        completion; an alias invalidates its snapshot. If two existing jobs are
        merged, no new job for that entity starts until both finish.
        """
        with self._lock:
            self._time(now)
            if source not in self._parent or target not in self._parent:
                raise KeyError('alias requires registered entities')
            a,b=self._root(source),self._root(target)
            if a==b:
                return
            old_a=self._pending.pop(a,None)
            old_b=self._pending.pop(b,None)
            self._parent[a]=b
            self._entities[b].confirmed|=self._entities.pop(a).confirmed
            generation=self._advance(b)
            requests=[]
            if not self._entities[b].confirmed:
                requests=[r for r in (old_a,old_b) if r]
                requests.extend(w.request for w in self._active.values()
                                if self._root(w.request.entity)==b)
            if requests:
                reasons=Reason.ALIAS
                for r in requests:
                    reasons|=r.reasons
                if len(self._pending)<self.max_pending:
                    eligible=min((r.eligible_generation for r in requests
                                  if r.eligible_generation is not None),default=None)
                    self._pending[b]=Request(b,generation,
                        min(r.first_requested_s for r in requests),reasons,
                        eligible)
                else:
                    self._log('alias_queue_capacity',b,now)
                # Restore age order after merging rather than jumping the queue.
                self._pending=dict(sorted(self._pending.items(),
                    key=lambda item:(item[1].first_requested_s,item[0])))
            self._log('aliased',b,now)

    def snapshot(self):
        with self._lock:
            return dict(pending=tuple(self._pending.values()),
                        active=tuple(self._active.values()),
                        registered_ids=len(self._parent),
                        entities=len(self._entities),
                        audit=tuple(self._audit),audit_dropped=self.audit_dropped)
