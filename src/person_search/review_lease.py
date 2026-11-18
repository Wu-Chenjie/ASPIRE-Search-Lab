"""Single-authority task ownership and exact terminal receipts.

Transport adapter must use the authority clock. Expiry requires cancellation;
it never authorizes a second executor before the first has stopped. This is not
distributed consensus and does not initiate motion or acquire identities.
"""
from dataclasses import dataclass,replace
import math

@dataclass(frozen=True)
class ReviewLease:
    component_key:str
    owner:str
    evidence_version:int
    token:int
    expires_at:float
    hard_deadline:float
    revoke_requested:bool=False
    terminal:bool=False
    issued_at:float | None=None

class ReviewLeaseBook:
    def __init__(self,clock_domain,max_active=32,max_duration_s=30.):
        if not clock_domain or max_active<1 or not math.isfinite(max_duration_s) or max_duration_s<=0:raise ValueError('Invalid lease configuration')
        self.clock_domain=clock_domain;self.max_active=max_active;self.max_duration_s=max_duration_s
        self.leases={};self.next_token=1;self.versions={}

    def grant(self,component_key,owner,evidence_version,now,ttl_s,clock_domain):
        if clock_domain!=self.clock_domain or not component_key or not owner or type(evidence_version) is not int or evidence_version<1:raise ValueError('Lease provenance required')
        if not math.isfinite(now) or now<0 or not 0<ttl_s<=self.max_duration_s:raise ValueError('Invalid lease time')
        if evidence_version<=self.versions.get(component_key,0):return None
        active=0
        for lease in self.leases.values():
            if lease.terminal:continue
            active+=1
            if lease.component_key==component_key or active>=self.max_active:return None
        lease=ReviewLease(component_key,owner,evidence_version,self.next_token,
                          now+ttl_s,now+self.max_duration_s,issued_at=now)
        self.next_token+=1;self.leases[lease.token]=lease;self.versions[component_key]=evidence_version
        return lease

    def valid(self,token,owner,now):
        lease=self.leases.get(token)
        if lease is None or lease.owner!=owner or lease.terminal or lease.revoke_requested:
            return False
        issued_at=lease.issued_at
        if issued_at is None:
            # Earlier records kept only the deadline. Retain their original lower bound.
            issued_at=lease.hard_deadline-self.max_duration_s
        return bool(math.isfinite(now) and issued_at<=now<lease.expires_at)

    def renew(self,token,owner,now,ttl_s):
        if not self.valid(token,owner,now) or not math.isfinite(ttl_s) or ttl_s<=0:return False
        lease=self.leases[token]
        self.leases[token]=replace(lease,expires_at=min(lease.hard_deadline,now+ttl_s));return True

    def revoke(self,token):
        lease=self.leases.get(token)
        if lease is None or lease.terminal:return False
        self.leases[token]=replace(lease,revoke_requested=True);return True

    def expire(self,now):
        if not math.isfinite(now):raise ValueError('Invalid authority time')
        for token,lease in self.leases.items():
            if not lease.terminal and not lease.revoke_requested and now>=lease.expires_at:
                self.leases[token]=replace(lease,revoke_requested=True)

    def acknowledge(self,token,owner,*,stopped):
        lease=self.leases.get(token)
        if lease is None or owner!=lease.owner or stopped is not True:return False
        if not lease.terminal:
            self.leases[token]=replace(lease,terminal=True)
        return True

    def reconcile(self,old_keys,new_key):
        """Registry-driven merge invalidates every outstanding old ownership."""
        old_keys=set(old_keys)
        if not new_key:raise ValueError('Canonical component key required')
        version=self.versions.get(new_key,0)
        self.versions[new_key]=max(version,max((self.versions.get(k,0) for k in old_keys),default=version))
        for token,lease in self.leases.items():
            if lease.component_key in old_keys:
                self.leases[token]=replace(lease,component_key=new_key,revoke_requested=not lease.terminal)
