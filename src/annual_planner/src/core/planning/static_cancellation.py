"""Versioned cancellation ledger, with no motion or safety-authority decisions.

An executing task can retire only after its caller supplies the existing native
stopped-reference certificate. Capacity exhaustion fails closed: tombstones are
never evicted while late DDS commands from this session may still arrive.
"""
from dataclasses import asdict, dataclass
import sys


@dataclass(frozen=True)
class StaticTask:
    session: str
    task: int
    proposal: int
    map_revision: int

    @classmethod
    def parse(cls, packet):
        if not isinstance(packet, dict) or set(packet) != {
                'session', 'task', 'proposal', 'map_revision'}:
            raise ValueError('Invalid static task identity')
        if (not isinstance(packet['session'], str) or
                not 1 <= len(packet['session']) <= 128):
            raise ValueError('Invalid cancellation session')
        for name in ('task', 'proposal', 'map_revision'):
            value = packet[name]
            if type(value) is not int or not 0 <= value < 2**53:
                raise ValueError('Invalid exact task integer')
        if not packet['task'] or not packet['map_revision']:
            raise ValueError('Task and map revision must be positive')
        return cls(**packet)


def _valid_certificate(proof, *required):
    if (not isinstance(proof, dict) or proof.get('observation_complete') is not False
            or any(proof.get(flag) is not True for flag in required)):
        return False
    checked_at = proof.get('checked_at')
    return type(checked_at) in (int, float) and 0 <= checked_at <= sys.float_info.max


def _valid_successor(successor, token):
    return isinstance(successor, str) and bool(successor) and successor != token


class StaticCancellation:
    TERMINAL = frozenset({'revoked_before_start', 'retired_at_safe_rest', 'superseded_by_c2'})

    def __init__(self, session, capacity=None):
        StaticTask.parse(dict(session=session, task=1, proposal=0, map_revision=1))
        if capacity is not None and (type(capacity) is not int or capacity < 1):
            raise ValueError('Positive cancellation capacity required')
        self.session = session
        self.capacity = capacity
        self.records = {}
        self.proposals = {}

    def key(self, packet):
        key = StaticTask.parse(packet)
        if key.session != self.session:
            raise ValueError('Cancellation session mismatch')
        return key

    def _record(self, packet):
        key = self.key(packet)
        old = self.proposals.get(key.proposal)
        if old is not None and old != key:
            raise ValueError('Conflicting task or map version for proposal')
        if key not in self.records:
            if self.capacity is not None and len(self.records) >= self.capacity:
                raise ValueError('Cancellation ledger full; new admission blocked')
            self.records[key] = dict(phase='prepared', token=None, revision=0, proof=None)
            self.proposals[key.proposal] = key
        return key, self.records[key]

    def _admit(self, packet, token):
        if not isinstance(token, str) or not token:
            raise ValueError('Executor token required')
        _, state = self._record(packet)
        if state['revision'] or state['phase'] != 'prepared':
            return None
        if state['token'] not in (None, token):
            raise ValueError('Executor token changed for static task')
        state['token'] = token
        return state

    def admit(self, packet, token):
        return self._admit(packet, token) is not None

    def reject_prepared(self, packet, token):
        """Prevent a rejected candidate from reappearing in a delayed command."""
        state = self.records.get(self.key(packet))
        if state and state['phase'] == 'prepared' and state['token'] == token:
            state['phase'] = 'revoked_before_start'

    def activated(self, packet, token):
        state = self._admit(packet, token)
        if state is None:
            raise ValueError('Cancelled or repeated static task cannot activate')
        state['phase'] = 'executing'

    def request(self, packet):
        if (not isinstance(packet, dict) or set(packet) != {'version', 'task', 'revision'}
                or type(packet['version']) is not int or packet['version'] != 1
                or type(packet['revision']) is not int or packet['revision'] != 1):
            raise ValueError('Cancellation v1 has exactly one irreversible revision')
        key, state = self._record(packet['task'])
        if not state['revision']:
            state['revision'] = 1
            if state['phase'] not in self.TERMINAL:
                state['phase'] = ('waiting_safe_rest' if state['phase'] == 'executing'
                                  else 'revoked_before_start')
        return self.ack(key)

    def supersede(self, packet, token, successor, proof):
        key = self.key(packet); state = self.records.get(key)
        if (not state or state['token'] != token or
                state['phase'] not in ('executing', 'waiting_safe_rest')):
            raise ValueError('No matching active task for C2 replacement')
        if not (_valid_successor(successor, token)
                and _valid_certificate(proof, 'authorized_transition_passed')):
            raise ValueError('Actual C2 transition certificate required')
        state.update(phase='superseded_by_c2', proof=dict(proof, successor_token=successor))
        return self.ack(key)

    def _waiting(self, key, token):
        state = self.records.get(key)
        return state if state and state['token'] == token and state['phase'] == 'waiting_safe_rest' else None

    def waiting(self, packet, token):
        return packet is not None and self._waiting(self.key(packet), token) is not None

    def retire(self, packet, token, proof):
        key = self.key(packet)
        state = self._waiting(key, token)
        if state is None:
            raise ValueError('No matching executing cancellation')
        if not _valid_certificate(proof, 'reference_stopped', 'original_safety_checks_passed'):
            raise ValueError('Existing safety certificate required')
        state.update(phase='retired_at_safe_rest', proof=dict(proof))
        return self.ack(key)

    def ack(self, key):
        state = self.records[key]
        return dict(version=1, task=asdict(key), revision=state['revision'],
                    state=state['phase'], terminal=state['phase'] in self.TERMINAL,
                    executor_token=state['token'], proof=state['proof'])


class PlannerCancellation:
    """Holds the planner slot until an exact terminal acknowledgement arrives."""
    def __init__(self, task):
        self.task = StaticTask.parse(task)
        self.cancelled = False
        self.resolved = False
        self.executor_token = None

    def request(self):
        self.cancelled = True
        return dict(version=1, task=asdict(self.task), revision=1)

    def acknowledge(self, ack):
        if not self.cancelled or not isinstance(ack, dict):
            return False
        try:
            key = StaticTask.parse(ack.get('task'))
        except ValueError:
            return False
        if (key != self.task or type(ack.get('version')) is not int or ack['version'] != 1
                or type(ack.get('revision')) is not int or ack['revision'] != 1):
            return False
        token = ack.get('executor_token')
        if self.executor_token is not None and token != self.executor_token:
            return False
        if token is not None:
            if not isinstance(token, str) or not token:
                return False
            self.executor_token = token
        if ack.get('terminal') is not True or ack.get('state') not in StaticCancellation.TERMINAL:
            return False
        proof = ack.get('proof')
        if ack['state'] == 'retired_at_safe_rest' and not (
                token and _valid_certificate(proof, 'reference_stopped', 'original_safety_checks_passed')):
            return False
        if ack['state'] == 'superseded_by_c2' and not (
                token and _valid_certificate(proof, 'authorized_transition_passed')
                and _valid_successor(proof.get('successor_token'), token)):
            return False
        self.resolved = True
        return True

    def may_accept_result(self, task):
        return not self.cancelled and StaticTask.parse(task) == self.task
