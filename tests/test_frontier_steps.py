"""The same geometric selector built into RACER must respect the measured side."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import pytest


def test_native_frontier_steps_follow_mirrored_measured_space(tmp_path):
    compiler = shutil.which('c++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    pkg_config = shutil.which('pkg-config')
    probe = subprocess.run([pkg_config, '--cflags', 'eigen3'], capture_output=True, text=True) if pkg_config else None
    if probe is not None and probe.returncode == 0:
        flags = shlex.split(probe.stdout)
    else:
        incl = Path(os.environ.get('EIGEN3_INCLUDE_DIR', '/usr/include/eigen3'))
        if not (incl / 'Eigen/Core').is_file():
            pytest.skip('Eigen development headers unavailable')
        flags = ['-I', str(incl)]
    root = Path(__file__).resolve().parents[1]
    exe = tmp_path / 'frontier-steps-test'
    subprocess.run([compiler, '-std=c++17', *flags, '-I', str(root / 'tools/flight'),
                    str(root / 'tests/frontier_steps_test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
