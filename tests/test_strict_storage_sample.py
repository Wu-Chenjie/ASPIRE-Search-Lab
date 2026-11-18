import os
from contextlib import contextmanager
import pytest
from tools.flight import strict_storage_sample as m


def test_real_nested_tree(tmp_path):
    (tmp_path/'nested').mkdir();(tmp_path/'a').write_bytes(b'ab');(tmp_path/'nested'/'b').write_bytes(b'123')
    assert m.tree_bytes(tmp_path)=={'bytes':5+tmp_path.stat().st_size+(tmp_path/'nested').stat().st_size,'attempts':1}


@pytest.mark.parametrize('name,exception,recover,ok',[
    ('status.tmp',FileNotFoundError,True,True),
    ('status.tmp',FileNotFoundError,False,False),
    ('data.jsonl',FileNotFoundError,True,False),
    ('status.tmp',PermissionError,True,False),
    ('status.tmp',OSError,True,False),
])
def test_read_failures_are_not_zero_capacity(monkeypatch,tmp_path,name,exception,recover,ok):
    (tmp_path/'data').write_bytes(b'1234');real=os.scandir;calls=[]
    class Entry:
        path=str(tmp_path/name)
        def stat(self,**kw):raise exception('injected filesystem failure')
    Entry.name=name
    @contextmanager
    def scan(path):
        calls.append(path)
        if len(calls)==1 or not recover:yield iter([Entry()])
        else:
            with real(path) as entries:yield entries
    monkeypatch.setattr(m.os,'scandir',scan)
    if ok:assert m.tree_bytes(tmp_path)=={'bytes':4+tmp_path.stat().st_size,'attempts':2}
    else:
        with pytest.raises(m.StorageSampleError):m.tree_bytes(tmp_path)
    assert len(calls)==(2 if ok else 3 if exception is FileNotFoundError and name.endswith('.tmp') else 1)


def test_missing_root_is_error(tmp_path):
    with pytest.raises(m.StorageSampleError):m.tree_bytes(tmp_path/'gone')
