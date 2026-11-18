from pathlib import Path
import shutil,subprocess
import pytest

def test_conditional_count_bounds_never_force_alias_or_use_bearing_proximity(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:pytest.skip('C++17 compiler unavailable')
    root=Path(__file__).resolve().parents[1];exe=tmp_path/'count-probe'
    subprocess.run([compiler,'-O2','-std=c++17','-I',str(root/'tools/flight'),str(root/'tests/identity_count_probe.cpp'),'-o',str(exe)],check=True,capture_output=True,text=True)
    out=subprocess.run([str(exe)],capture_output=True,text=True);assert out.returncode==0,(out.stdout,out.stderr)
    assert 'PASS_CONDITIONAL_IDENTITY_BOUNDS' in out.stdout
