"""No new permissions, exact affinity installation and isolated critical child."""
from pathlib import Path
import os,sys,json,shutil,subprocess
from types import SimpleNamespace as NS
import pytest
from tools.flight.cpu_profile import CPUProfile

@pytest.mark.parametrize('role',['executor','agent','mapper','evaluation'])
def test_other_owned_clients_cannot_use_reserved_cpu(monkeypatch,role):
    monkeypatch.setattr(shutil,'which',lambda _: '/usr/bin/taskset')
    p=CPUProfile('render3-perception2-px4reserved',allowed=range(2,8))
    assert p.command(role,['work'])==['/usr/bin/taskset','-c','2,3,4,6,7','work']
    assert p.target('px4')==[5] and p.target('gazebo')==[2,3,4] and p.target('person_search')==[6,7]
    assert all(set(p.target(r))<=set(p.allowed) for r in [role,'px4','gazebo','person_search'])

def test_parent_changes_only_its_existing_mask(monkeypatch):
    current=[set(range(6))];calls=[]
    monkeypatch.setattr(CPUProfile,'parent_tasks',lambda _:[0])
    monkeypatch.setattr(os,'sched_getaffinity',lambda _:set(current[0]),raising=False)
    def setmask(pid,cpus):assert pid==0;calls.append(set(cpus));current[0]=set(cpus)
    monkeypatch.setattr(os,'sched_setaffinity',setmask,raising=False)
    p=CPUProfile('render3-perception2-px4reserved',allowed=range(6));p.install_parent()
    assert calls and all(a=={0,1,2,4,5} for a in calls) and p.report()['verified_parent']==[0,1,2,4,5]
    assert p.report()['parent_allowed']==list(range(6))
    assert p.report()['no_cpu_permission_expansion']

def test_changed_parent_mask_fails_closed(monkeypatch):
    monkeypatch.setattr(os,'sched_getaffinity',lambda _:{0,1,2,4,5},raising=False)
    p=CPUProfile('render3-perception2-px4reserved',allowed=range(6))
    with pytest.raises(RuntimeError,match='changed before'):p.install_parent()

def test_missing_taskset_rejected_and_wrong_child_not_accepted(monkeypatch):
    p=CPUProfile('render3-perception2-px4reserved',allowed=range(6))
    with pytest.raises(ValueError,match='6 existing'):
        CPUProfile('render3-perception2-px4reserved',allowed=range(5))
    monkeypatch.setattr(shutil,'which',lambda _:None)
    with pytest.raises(ValueError,match='taskset'):p.command('px4',['work'])
    monkeypatch.setattr(os,'sched_getaffinity',lambda _:{0,1,2,3,4,5},raising=False)
    with pytest.raises(RuntimeError,match='not installed'):p.verify('px4',NS(pid=10,poll=lambda:0))

def test_real_children_and_all_created_threads_inherit_masks():
    if not hasattr(os,'sched_getaffinity') or not shutil.which('taskset'):pytest.skip('Linux affinity required')
    original=set(os.sched_getaffinity(0));original_masks={tid:set(os.sched_getaffinity(tid)) for tid in CPUProfile.parent_tasks()}
    if len(original)<6:pytest.skip('Six existing CPUs required')
    p=CPUProfile('render3-perception2-px4reserved')
    worker="import os,json,threading;out=[];a=threading.Thread(target=lambda:out.append(sorted(os.sched_getaffinity(0))));a.start();a.join();print(json.dumps([sorted(os.sched_getaffinity(0)),*out]),flush=True);input()"
    try:
        p.install_parent();assert p.roles['px4'][0] not in os.sched_getaffinity(0)
        for role in ['px4','mapper','person_search']:
            proc=subprocess.Popen(p.command(role,[sys.executable,'-c',worker]),stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            try:
                p.verify(role,proc);assert json.loads(proc.stdout.readline())==[p.target(role)]*2
            finally:proc.communicate('\n',timeout=5)
    finally:
        for tid,mask in original_masks.items():
            try:os.sched_setaffinity(tid,mask)
            except ProcessLookupError:pass
    assert set(os.sched_getaffinity(0))==original

def test_real_preexisting_native_thread_cannot_keep_reserved_core():
    import threading
    if not hasattr(os,'sched_getaffinity'):pytest.skip('Linux affinity required')
    original_masks={tid:set(os.sched_getaffinity(tid)) for tid in CPUProfile.parent_tasks()}
    if len(os.sched_getaffinity(0))<6:pytest.skip('Six existing CPUs required')
    ready=threading.Event();done=threading.Event();observed={}
    def worker():
        observed['tid']=threading.get_native_id();ready.set();done.wait();observed['after']=sorted(os.sched_getaffinity(0))
    thread=threading.Thread(target=worker);thread.start();ready.wait()
    try:
        p=CPUProfile('render3-perception2-px4reserved');p.install_parent()
        assert p.report()['parent_thread_masks'][str(observed['tid'])]==p.other_cpus
        done.set();thread.join(timeout=2);assert not thread.is_alive()
        assert observed['after']==p.other_cpus
    finally:
        done.set();thread.join(timeout=2)
        for tid,mask in original_masks.items():
            try:os.sched_setaffinity(tid,mask)
            except ProcessLookupError:pass
