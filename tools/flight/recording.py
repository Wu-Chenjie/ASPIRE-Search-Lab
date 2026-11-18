"""Restore only the minimal, regular replay inputs from a saved run archive."""
import hashlib
import tarfile
from pathlib import Path


def restore(archive_path, run_name, destination):
    if not run_name or Path(run_name).name != run_name or run_name in ('.', '..'):
        raise ValueError('Expected a recorded run name, not a path')
    destination.mkdir(parents=True, exist_ok=False)
    with tarfile.open(archive_path, 'r:gz') as archive:
        for member in archive.getmembers():
            path = Path(member.name)
            if not member.isfile() or path.parent.as_posix() != run_name:
                continue
            if path.name == 'telemetry.jsonl' or (path.name.startswith('rgbd-') and path.suffix == '.npz'):
                with archive.extractfile(member) as source:
                    (destination / path.name).write_bytes(source.read())
    if not (destination / 'telemetry.jsonl').is_file() or not list(destination.glob('rgbd-*.npz')):
        raise ValueError('Recorded run lacks telemetry or RGB-D frames')
    return dict(path=str(archive_path), sha256=hashlib.sha256(archive_path.read_bytes()).hexdigest(), run=run_name)
