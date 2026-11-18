"""Compile and exercise the exact mission policy included by native RACER."""
from pathlib import Path
import shutil
import subprocess
import pytest


def test_return_mission_physical_completion_and_retry_backoff(tmp_path):
    compiler=shutil.which('c++')
    if compiler is None:pytest.skip('C++ compiler unavailable')
    root=Path(__file__).resolve().parents[1]
    shutil.copyfile(root/'tools/flight/recovery_policy.hpp',tmp_path/'task_recovery_policy.hpp')
    binary=tmp_path/'return-policy'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'tools/flight'),'-I',str(tmp_path),str(root/'tests/return_policy_test.cpp'),'-o',str(binary)],capture_output=True,text=True,check=True)
    result=subprocess.run([str(binary)],capture_output=True,text=True,check=True)
    assert 'PASS:' in result.stdout
