import hashlib
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_build_accepts_empty_directory_with_one_copy_owner(tmp_path):
    output = tmp_path/'workspace'
    output.mkdir()
    command = [sys.executable, '-B', str(ROOT/'tools/search/build_native.py'), str(output)]
    with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE) as first, \
         subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE) as second:
        first.communicate(timeout=30)
        second.communicate(timeout=30)
    assert sum(process.returncode == 0 for process in (first, second)) == 1
    record = json.loads((output/'task-overlay.json').read_text())
    assert record['base_migration_sha256'] == hashlib.sha256(
        (ROOT/'external/racer_ros2_ws/migration.json').read_bytes()).hexdigest()
    assert record['consistent_inflation'] is False
    assert (output/'src/racer_core/native_driver.cpp').is_file()


def test_guarded_build_uses_the_consistent_inflation_source(tmp_path):
    generated = []
    for flag in ('--consistent-inflation', '--execution-safety'):
        output = tmp_path/flag[2:]
        subprocess.run([sys.executable, '-B', str(ROOT/'tools/search/build_native.py'),
                        str(output), flag], check=True, capture_output=True)
        record = json.loads((output/'task-overlay.json').read_text())
        assert record['consistent_inflation'] is True
        generated.append((output/'src/racer_core/src/plan_env/sdf_map.cpp').read_bytes())
    original = ROOT/'external/racer_ros2_ws/src/racer_core/src/plan_env/sdf_map.cpp'
    assert generated[0] == generated[1] != original.read_bytes()


def test_build_refuses_existing_workspace_contents(tmp_path):
    output = tmp_path/'workspace'
    output.mkdir()
    marker = output/'evidence'
    marker.write_text('existing data')
    command = [sys.executable, '-B', str(ROOT/'tools/search/build_native.py'), str(output)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode != 0 and 'Build directory is not empty' in result.stderr
    assert list(output.iterdir()) == [marker] and marker.read_text() == 'existing data'
