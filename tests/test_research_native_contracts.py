"""Native alignment, bounded component review and ownership semantics."""
from math import nextafter

from test_native_static_events import compile_run
from person_search.review_lease import ReviewLease, ReviewLeaseBook

def test_peer_alignment_clock_epoch_uncertainty_and_origin_replay(tmp_path):
    compile_run(tmp_path,r'''#include "target_registry.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetRegistry r;TargetObservation local;
local.key={"uav0","s",1};local.sequence=1;local.first=local.capture=1;local.frame_version="local-v1";
local.has_position=true;local.localization="reliable";local.position={4,0,1};local.visual="visually_confirmed";
assert(r.ingest(local,1).accepted);
auto peer=local;peer.key={"uav1","remote",1};peer.revision=1;peer.first=peer.capture=10;
peer.frame_version="remote-v1";peer.clock_domain="remote-clock";peer.position={3,0,1};peer.shared_aligned=true;
peer.observation_id="uav1/remote/image10/0";
assert(!r.ingest(peer,1,true).accepted); // a boolean is not an alignment estimate
TargetAlignment a;a.robot="uav1";a.id="estimated-v1";a.source_version="remote-v1";a.target_version="local-v1";
a.source_clock="remote-clock";a.clock_offset_s=-9;a.clock_error_s=.001;a.translation={1,0,0};a.position_error_m=.02;a.valid_until=100;
r.peer_alignments[a.robot]=a;assert(r.ingest(peer,1,true).accepted);r.associate({peer.key},1);
auto malformed=peer;malformed.radius=INFINITY;assert(!r.ingest(malformed,1,true).accepted);
assert(r.roots().size()==1);auto&record=r.records.at(peer.key);
assert(record.latest.position[0]==4&&record.latest.capture==1&&std::abs(record.latest.radius-.17)<1e-12);
assert(!r.ingest(peer,1,true).accepted);
auto forwarded=peer;forwarded.key.counter=2;assert(!r.ingest(forwarded,1,true).accepted);
auto wrong=peer;wrong.revision=2;wrong.clock_domain="wrong";assert(!r.ingest(wrong,1,true).accepted);
r.retire(101);assert(!record.has_anchor&&!record.latest.has_position);assert(r.roots().size()==2);
assert(record.confirmed); // coordinate expiry never negates semantic evidence
}
''')

def test_component_review_budget_survives_merge_split_and_reserves_route(tmp_path):
    compile_run(tmp_path,r'''#include "static_review_budget.hpp"
#include <cassert>
using namespace racer_task;
int main(){StaticReviewBudget b;b.component_limit=2;b.cooldown_s=1.;TargetKey a{"uav0","s",1},c{"uav0","s",2};
using P=StaticReviewBudget::Purpose;
assert(!b.eligible({a},a,"v1",1,1,2,P::ImprovePosition,true).empty());
assert(b.eligible({a},a,"v1",1,1,2,P::ConfirmPresence,false).empty());b.reserve({a},a,"v1",1);
assert(!b.eligible({a},a,"v1",2,2,2,P::ConfirmPresence,false).empty());
assert(b.route(a,"v1",10,2));assert(b.route(a,"v1",10,2));
b.reconcile({a,c});assert(!b.eligible({c},c,"v1",1.5,1.5,2,P::ConfirmPresence,false).empty());
assert(b.eligible({c},c,"v1",2,2,2,P::ConfirmPresence,false).empty());b.reserve({c},c,"v1",2);
assert(!b.route(c,"v1",25,1));assert(!b.route(c,"v1",5,5));assert(b.route(c,"v1",5,2));
b.reconcile({a,c});assert(!b.eligible({c},c,"v2",4,4,2,P::ConfirmPresence,false).empty());
assert(!b.eligible({a},a,"v2",100,1,2,P::DisambiguateIdentity,true,true).empty());
}
''')

def test_sparse_anchor_candidates_keep_old_confirmations_without_scanning_all(tmp_path):
    compile_run(tmp_path,r'''#include "target_registry.hpp"
#include <cassert>
using namespace racer_task;
int main(){TargetRegistry r;TargetObservation o;o.key={"uav0","s",1};o.sequence=1;o.frame_version="m";
o.has_position=true;o.localization="reliable";o.visual="visually_confirmed";
for(int i=1;i<=200;++i){o.key.counter=i;o.first=o.capture=i*3.;o.position={i*3.,0,1};assert(r.ingest(o,o.capture).accepted);}
o.key.counter=201;o.first=o.capture=1000;o.position={3.1,0,1};r.ingest(o,1000);r.associate({o.key},1000);
assert(r.component(o.key).size()==2);assert(r.stats.scans<5);assert(r.records.size()==201);
}
''')

def test_lease_expiry_needs_exact_stop_receipt_and_merge_revokes_owners():
    book=ReviewLeaseBook('authority');a=book.grant('component1','uav0',1,10,2,'authority')
    assert a and book.valid(a.token,'uav0',11)
    assert book.grant('component1','uav1',2,11,2,'authority') is None
    book.expire(12);assert not book.valid(a.token,'uav0',12)
    assert book.grant('component1','uav1',2,13,2,'authority') is None
    assert not book.acknowledge(a.token,'uav1',stopped=True)
    assert not book.acknowledge(a.token,'uav0',stopped=False)
    assert book.acknowledge(a.token,'uav0',stopped=True)
    b=book.grant('component1','uav1',2,14,2,'authority');assert b
    book.reconcile(['component1'],'component0')
    assert not book.valid(b.token,'uav1',14)
    assert book.grant('component0','uav0',3,14,2,'authority') is None


def test_fractional_grant_is_valid_at_issue_and_renewal_respects_hard_deadline():
    book = ReviewLeaseBook('s', max_active=1)
    lease = book.grant('e', 'o', 1, .01, .5, 's')
    earlier = nextafter(.01, 0.)
    assert not book.valid(lease.token, 'o', earlier)
    assert not book.renew(lease.token, 'o', earlier, 30.)
    assert book.valid(lease.token, 'o', .01)
    assert not book.valid(lease.token, 'o', lease.expires_at)
    assert book.renew(lease.token, 'o', .01, 60.)
    assert book.leases[lease.token].expires_at == lease.hard_deadline
    assert not book.renew(lease.token, 'o', lease.hard_deadline, 1.)
    book.expire(lease.hard_deadline)
    assert book.leases[lease.token].revoke_requested
    assert book.grant('other', 'o', 1, lease.hard_deadline, .5, 's') is None
    assert book.acknowledge(lease.token, 'o', stopped=True)
    assert book.acknowledge(lease.token, 'o', stopped=True)
    assert book.grant('other', 'o', 1, lease.hard_deadline, .5, 's')


def test_book_accepts_lease_records_without_issue_time():
    book = ReviewLeaseBook('s', max_active=1)
    book.leases[1] = ReviewLease('finished', 'o', 1, 1, 10., 39., terminal=True)
    book.leases[2] = ReviewLease('e', 'o', 1, 2, 11., 40.)
    book.next_token = 3
    book.versions.update(finished=1, e=1)

    assert not book.valid(2, 'o', 9.)
    assert book.valid(2, 'o', 10.)
    assert book.grant('other', 'o', 1, 10., 1., 's') is None
    assert book.acknowledge(2, 'o', stopped=True)
    assert book.grant('other', 'o', 1, 10., 1., 's').token == 3
