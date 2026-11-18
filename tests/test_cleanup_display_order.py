"""Owned display/client lifecycle regression, no simulator launch."""
import os,sys,time,subprocess
from pathlib import Path
from types import SimpleNamespace
import pytest
from tools.racer_ros2.light.processes import terminate


def test_owned_proc_birth_is_rechecked_before_signals(tmp_path, monkeypatch):
    from tools.racer_ros2.light import processes

    def stat(pid, parent, birth, state='S'):
        directory = tmp_path/str(pid)
        directory.mkdir(exist_ok=True)
        fields = [state, str(parent)] + ['0']*17 + [str(birth)]
        (directory/'stat').write_text(f'{pid} (has ) in comm) ' + ' '.join(fields))

    stat(100, 1, 1000)
    stat(101, 100, 1001)
    stat(102, 100, 1002)
    stat(103, 1, 9000)  # Reuses the PID of an already reaped child.
    stat(104, 100, 1004, state='Z')
    monkeypatch.setattr(processes, 'PROC', tmp_path)
    assert processes.snapshot() == {100: (1, '1000'), 101: (100, '1001'),
                                    102: (100, '1002'), 103: (1, '9000')}
    assert not processes.same_process(104, '1004')
    now = [0.]
    monkeypatch.setattr(processes.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(processes.time, 'sleep', lambda delta: now.__setitem__(0, now[0]+delta))
    signals = []

    def kill(pid, sig):
        signals.append((pid, sig))
        if pid == 100:
            stat(100, 1, 1000, state='Z')
        if pid == 101:
            stat(101, 1, 9901)
        if pid == 102 and sig == processes.signal.SIGKILL:
            (tmp_path/'102/stat').unlink()

    monkeypatch.setattr(processes.os, 'kill', kill)
    children = [SimpleNamespace(pid=pid, returncode=code, poll=lambda code=code: code,
                                wait=lambda **_: 0)
                for pid, code in ((100, None), (103, 7))]
    report = terminate(children, timeout=.05)
    assert set(signals) == {(100, processes.signal.SIGINT), (101, processes.signal.SIGINT),
                            (102, processes.signal.SIGINT), (102, processes.signal.SIGKILL)}
    assert report['forced_kill_pids'] == [102] and report['remaining_pids'] == []

@pytest.mark.skipif(not Path('/proc').exists(),reason='Linux owned-PID lifecycle')
def test_display_survives_client_sigint_and_abnormal_code_stays_visible(tmp_path):
    ready=tmp_path/'ready'
    display=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    script="import os,signal,sys,time;from pathlib import Path;pid=int(sys.argv[1]);signal.signal(signal.SIGINT,lambda *a:sys.exit(0 if Path('/proc/'+str(pid)).exists() else 9));Path(sys.argv[2]).write_text('ready');time.sleep(30)"
    client=subprocess.Popen([sys.executable,'-c',script,str(display.pid),str(ready)])
    bad=subprocess.Popen([sys.executable,'-c','import sys;sys.exit(7)']);bad.wait(timeout=3)
    until=time.monotonic()+3
    while not ready.exists() and time.monotonic()<until:time.sleep(.01)
    assert ready.exists()
    result=terminate([display,client,bad],timeout=2,defer_pids=[display.pid])
    assert client.returncode==0 and bad.returncode==7
    assert result['remaining_pids']==[] and result['forced_kill_pids']==[]
    assert display.pid in result['deferred_pids']
