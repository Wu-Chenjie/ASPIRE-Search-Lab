import os,struct,subprocess,shutil,json
from pathlib import Path
from test_native_static_events import compile_run
from test_async_identity import CPP

def read(path):
    data=path.read_bytes();h=struct.unpack_from('<16Q',data);assert h[0]==0x4153504952454331 and h[1]==1
    return h,[struct.unpack_from('<10Q',data,128+i*80) for i in range(h[3])]

def test_trace_bound_overflow_and_exact_fields(tmp_path):
    p=tmp_path/'journal.bin'
    src=r'''#include "static_causal_trace.hpp"
int main(){racer_task::StaticCausalTrace disabled(nullptr);disabled.record(1,1.25);
 racer_task::StaticCausalTrace enabled("PATH");for(unsigned i=0;i<8200;++i)enabled.record(2,10.5,7,8,i,10.25,1,2);}
'''.replace('PATH',str(p))
    compile_run(tmp_path,src);h,rows=read(p)
    assert len(p.read_bytes())==128+8192*80 and h[3:5]==(8192,8)
    assert rows[0][3:]==(2,7,8,0,10250000000,1,2)
    assert rows[-1][6]==8191 and rows[0][1]==10500000000
    assert all(a[2]<=b[2] for a,b in zip(rows,rows[1:]))

def test_enabled_journal_preserves_all_adversarial_scheduler_results(tmp_path,monkeypatch):
    path=tmp_path/'scheduler.bin';monkeypatch.setenv('ASPIRE_NATIVE_CAUSAL_JOURNAL',str(path))
    compile_run(tmp_path,CPP);h,rows=read(path)
    assert h[4]==0
    assert {10,11,12}<={r[3] for r in rows}
    assert any(r[3]==12 and r[9]==3 for r in rows)  # deadline release
    assert any(r[3]==12 and r[9]==1 for r in rows)  # confirmed merge
