"""Common SITL resource isolation; no VM/cgroup/network permissions changed."""
import os,shutil,time
class CPUProfile:
    def __init__(self,name='inherit',allowed=None):
        self.name=name;self.verified={}
        if name=='inherit':self.allowed=[];self.roles={};return
        if name not in ('render3-perception2','render3-perception2-px4reserved'):raise ValueError('Unknown CPU profile')
        if allowed is None:
            if not hasattr(os,'sched_getaffinity'):raise ValueError('CPU affinity unavailable')
            allowed=os.sched_getaffinity(0)
        self.allowed=sorted(set(allowed))
        reserve_px4=name=='render3-perception2-px4reserved'
        minimum_cpus=6 if reserve_px4 else 5
        if len(self.allowed)<minimum_cpus:raise ValueError(f'{minimum_cpus} existing allowed CPUs required')
        if any(not isinstance(c,int) or c<0 for c in self.allowed):raise ValueError('Invalid CPU set')
        self.roles={'gazebo':self.allowed[:3],'person_search':self.allowed[-2:]}
        self.other_cpus=None;self.parent_installed=False
        if reserve_px4:
            self.roles['px4']=[self.allowed[3]]
            self.other_cpus=[c for c in self.allowed if c!=self.allowed[3]]
    @staticmethod
    def parent_tasks():
        return [int(tid) for tid in os.listdir('/proc/self/task')]
    def parent_masks(self):
        masks={}
        for tid in self.parent_tasks():
            try:masks[str(tid)]=sorted(os.sched_getaffinity(tid))
            except ProcessLookupError:pass
        return masks
    def install_parent(self):
        if self.name!='render3-perception2-px4reserved':return
        if set(os.sched_getaffinity(0))!=set(self.allowed):raise RuntimeError('Parent affinity changed before isolation')
        # Linux affinity is per thread. Pin the calling thread before enumerating
        # existing native pools, then close any thread-creation race by verifying
        # the whole owned task set. No foreign process or scheduling policy change.
        os.sched_setaffinity(0,self.other_cpus);deadline=time.monotonic()+.1
        while True:
            for tid in self.parent_tasks():
                try:os.sched_setaffinity(tid,self.other_cpus)
                except ProcessLookupError:pass
            masks=self.parent_masks()
            if masks and all(set(c)==set(self.other_cpus) for c in masks.values()):break
            if time.monotonic()>=deadline:raise RuntimeError('Parent thread isolation not installed')
        self.parent_installed=True
    def target(self,name):
        return self.roles.get(name,self.other_cpus if self.name=='render3-perception2-px4reserved' else None)
    def command(self,name,args):
        cpus=self.target(name)
        if cpus is None:return list(args)
        binary=shutil.which('taskset')
        if binary is None:raise ValueError('taskset unavailable')
        return [binary,'-c',','.join(map(str,cpus)),*args]
    def best_effort_command(self,name,args):
        cpus=self.target(name)
        if cpus is None:return list(args)
        # Always exec the real workload, including if affinity cannot be set.
        # The shell becomes the workload PID that the launcher owns and checks.
        return ['/bin/sh','-c','taskset -pc "$1" "$$" >/dev/null; shift; exec "$@"',
                'cpu-affinity',','.join(map(str,cpus)),*args]
    def verify(self,name,process):
        cpus=self.target(name)
        if cpus is None:return
        wanted=set(cpus);deadline=time.monotonic()+.1
        while True:
            actual=os.sched_getaffinity(process.pid)
            if actual==wanted:self.verified[name]=sorted(actual);return
            if process.poll() is not None or time.monotonic()>=deadline:
                raise RuntimeError('Child CPU affinity not installed: '+name)
            time.sleep(.002)
    def report(self):
        extra={}
        if self.name=='render3-perception2-px4reserved':extra=dict(parent_thread_masks=self.parent_masks() if self.parent_installed else {},other_role_cpus=self.other_cpus,parent_installed=self.parent_installed,verified_parent=sorted(os.sched_getaffinity(0)) if self.parent_installed else None,scope='Owned process affinity only; no host/cgroup/VM isolation guarantee')
        return dict(name=self.name,parent_allowed=self.allowed,assigned=self.roles,
            verified=dict(self.verified),other_roles_inherit_parent=True,
            no_cpu_permission_expansion=True,**extra)
