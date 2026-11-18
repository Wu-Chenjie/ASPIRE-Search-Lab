import hashlib
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Barrier

from tools.flight import build_px4


def test_isolated_px4_build_claims_precreated_output_once(tmp_path, monkeypatch):
    source = tmp_path/'original'
    ekf = source/'src/modules/ekf2/EKF2.cpp'
    ekf.parent.mkdir(parents=True)
    original = b'bool update = (timestamp >= _last_status_flags_publish + 1_s);\n'
    ekf.write_bytes(original)
    monkeypatch.setattr(build_px4, 'EKF_SHA', hashlib.sha256(original).hexdigest())
    monkeypatch.setattr(build_px4, 'local_microcdr', lambda *args: {})
    out = tmp_path/'precreated'
    out.mkdir()
    monkeypatch.setattr(sys, 'argv', ['build_px4', '--source', str(source), '--output', str(out)])
    monkeypatch.setattr(build_px4.subprocess, 'check_output',
                        lambda *args, **kw: 'sha\n' if kw.get('text') else b'PATH=/usr/bin\x00')

    def fake_cmake(cmd, **kwargs):
        if cmd[1] == '--build':
            binaries = Path(cmd[2])/'bin'
            binaries.mkdir(parents=True)
            (binaries/'px4').write_bytes(b'fixture')
            bridge = binaries/'px4-gz_bridge'
            bridge.touch()
            bridge.chmod(0o755)

    monkeypatch.setattr(build_px4.subprocess, 'run', fake_cmake)
    barrier = Barrier(2)
    copytree = build_px4.shutil.copytree

    def simultaneous_copy(src, dst, *args, **kwargs):
        if Path(src) == source:
            barrier.wait(timeout=10)
        return copytree(src, dst, *args, **kwargs)

    monkeypatch.setattr(build_px4.shutil, 'copytree', simultaneous_copy)

    def try_build():
        try:
            build_px4.main()
        except FileExistsError:
            return False
        return True

    with ThreadPoolExecutor(max_workers=2) as pool:
        assert sorted(pool.map(lambda _: try_build(), range(2))) == [False, True]
    patched = (out/'source'/ekf.relative_to(source)).read_bytes()
    assert patched == original.replace(b'1_s', b'100_ms')
    assert ekf.read_bytes() == original
    report = json.loads((out/'build-manifest.json').read_text())
    assert report['patched_ekf_sha256'] == hashlib.sha256(patched).hexdigest()
    assert not try_build()
    assert (out/'source'/ekf.relative_to(source)).read_bytes() == patched
