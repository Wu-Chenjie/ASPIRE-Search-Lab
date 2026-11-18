import io
import tarfile
import pytest
from tools.flight.recording import restore


def test_saved_replay_restores_only_requested_regular_inputs(tmp_path):
    archive = tmp_path / 'runs.tar.gz'
    with tarfile.open(archive, 'w:gz') as out:
        for name in ['case/telemetry.jsonl', 'case/rgbd-001.npz', 'other/telemetry.jsonl',
                     'case/log.txt', 'case/../escape', '/case/rgbd-002.npz']:
            info = tarfile.TarInfo(name);info.size = 4
            out.addfile(info, io.BytesIO(b'data'))
        info = tarfile.TarInfo('case/rgbd-link.npz');info.type = tarfile.SYMTYPE
        info.linkname = '/etc/passwd';out.addfile(info)
    metadata = restore(archive, 'case', tmp_path / 'restored')
    assert metadata['run'] == 'case'
    assert sorted(p.name for p in (tmp_path / 'restored').iterdir()) == ['rgbd-001.npz', 'telemetry.jsonl']
    assert not (tmp_path / 'escape').exists()
    with pytest.raises(ValueError):
        restore(archive, '../case', tmp_path / 'bad')
    with pytest.raises(ValueError):
        restore(archive, 'missing', tmp_path / 'missing')
