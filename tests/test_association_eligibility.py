"""Native association regressions; no truth enters TargetRegistry."""
from pathlib import Path
import shutil
import subprocess
import pytest

def test_selectable_edges_competition_and_identity_boundaries(tmp_path):
    compiler=shutil.which('c++')
    if compiler is None:
        pytest.skip('C++17 compiler unavailable')
    root=Path(__file__).resolve().parents[1]
    binary=tmp_path/'eligible-edges'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(root/'tools/flight'),
                    str(root/'tests/association_eligibility_probe.cpp'),'-o',str(binary)],
                   check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
    assert 'PASS_ELIGIBLE_EDGES' in result.stdout
