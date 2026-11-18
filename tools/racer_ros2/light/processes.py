"""Terminate only descendants of this run, including ROS1 detached children."""
import os
from pathlib import Path
import signal
import time

PROC = Path('/proc')


def _live_stat(directory):
    try:
        value=(directory/'stat').read_text();fields=value[value.rfind(')')+2:].split()
        if fields[0]!='Z':
            return int(fields[1]),fields[19]
    except (OSError,IndexError,ValueError):
        pass


def same_process(pid,birth):
    entry=_live_stat(PROC/str(pid))
    return entry is not None and entry[1]==birth


def snapshot():
    table={}
    for p in PROC.iterdir():
        if not p.name.isdigit():continue
        entry=_live_stat(p)
        if entry is not None:table[int(p.name)]=entry
    return table


def terminate(children,timeout=8.,defer_pids=(),before_signals=None):
    before={c.pid:c.poll() for c in children}
    table=snapshot();owned={pid:table[pid][1] for pid,status in before.items() if status is None and pid in table}
    def extend(table):
        while True:
            new={pid:start for pid,(parent,start) in table.items() if parent in owned and table.get(parent,(None,None))[1]==owned[parent] and pid not in owned}
            if not new:break
            owned.update(new)
    extend(table)
    # Display servers must outlive their graphical clients during shutdown.
    # Ownership and PID birth-time checks apply to both phases; no external PID
    # or nonzero return code is hidden by the ordering change.
    deferred=set(defer_pids)&set(owned)
    def update_deferred(table):
        while True:
            new={pid for pid,(parent,start) in table.items() if pid in owned and parent in deferred and pid not in deferred}
            if not new:break
            deferred.update(new)
    update_deferred(table)
    killed=[]
    def phase(is_deferred,budget):
        table=snapshot();extend(table);update_deferred(table)
        targets={pid:start for pid,start in owned.items() if (pid in deferred)==is_deferred}
        for pid,start in targets.items():
            if same_process(pid,start):
                try:os.kill(pid,signal.SIGINT)
                except ProcessLookupError:pass
        deadline=time.monotonic()+budget
        while time.monotonic()<deadline:
            table=snapshot();extend(table);update_deferred(table)
            targets={pid:start for pid,start in owned.items() if (pid in deferred)==is_deferred}
            if not any(table.get(pid,(None,None))[1]==start for pid,start in targets.items()):break
            time.sleep(.05)
        for pid,start in targets.items():
            if same_process(pid,start):
                try:os.kill(pid,signal.SIGKILL);killed.append(pid)
                except ProcessLookupError:pass
    non_display_deadline=time.monotonic()+timeout
    if before_signals is not None:before_signals()
    phase(False,max(0.,non_display_deadline-time.monotonic()))
    phase(True,min(timeout,2.))
    for child in children:child.wait(timeout=5)
    deadline=time.monotonic()+2
    remaining=[pid for pid,start in owned.items() if same_process(pid,start)]
    while remaining and time.monotonic()<deadline:
        time.sleep(.05)
        remaining=[pid for pid,start in owned.items() if same_process(pid,start)]
    return dict(owned_processes=len(owned),deferred_pids=sorted(deferred),forced_kill_pids=killed,remaining_pids=remaining,
                direct_returncodes=[dict(pid=c.pid,returncode=c.returncode,was_running_before_cleanup=before[c.pid] is None) for c in children])
