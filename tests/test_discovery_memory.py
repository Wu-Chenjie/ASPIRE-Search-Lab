from pathlib import Path
import shutil,subprocess,pytest

def test_cpp_discovery_epoch_and_failure_memory(tmp_path):
    compiler=shutil.which('c++')
    if not compiler:pytest.skip('C++17 compiler unavailable')
    root=Path(__file__).resolve().parents[1];header=root/'tools/flight';binary=tmp_path/'discovery-probe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(header),str(header/'discovery_probe.cpp'),'-o',str(binary)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
    assert 'PASS_DISCOVERY' in result.stdout
