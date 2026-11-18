"""Verify a completed official HM3D TAR and extract only self-contained GLBs.

No authentication or network access. Partial browser downloads are rejected.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import struct
import tarfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = 'https://api.matterport.com/resources/habitat/hm3d-minival-glb-v0.2.tar'
LICENSE = 'https://matterport.com/legal/matterport-end-user-license-agreement-academic-use-model-data'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def glb_info(data):
    if len(data) < 20 or struct.unpack('<4sII', data[:12]) != (b'glTF', 2, len(data)):
        raise ValueError('Invalid or truncated GLB header')
    offset, chunks = 12, []
    while offset < len(data):
        if offset + 8 > len(data): raise ValueError('Truncated GLB chunk header')
        size, kind = struct.unpack('<I4s', data[offset:offset+8])
        offset += 8
        if size % 4 or offset + size > len(data): raise ValueError('Truncated GLB chunk')
        chunks.append((kind, data[offset:offset+size]))
        offset += size
    if not chunks or chunks[0][0] != b'JSON': raise ValueError('Missing GLB JSON')
    definition = json.loads(chunks[0][1])
    if any('uri' in item for key in ('buffers', 'images') for item in definition.get(key, [])):
        raise ValueError('External GLB resources are not supported by this extractor')
    return dict(meshes=len(definition.get('meshes', [])), images=len(definition.get('images', [])),
                materials=len(definition.get('materials', [])))


def extract(archive, output):
    archive, output = Path(archive).resolve(), Path(output).resolve()
    if any(part.endswith(('.download', '.partial', '.part')) for part in archive.parts):
        raise ValueError('Browser download is incomplete')
    if archive.suffix != '.tar': raise ValueError('Expected completed uncompressed TAR')
    if output == ROOT or ROOT in output.parents: raise ValueError('Data must remain outside code tree')
    if output.exists(): raise FileExistsError(output)
    before = archive.stat()
    archive_hash = digest(archive)
    with tarfile.open(archive, 'r:') as tf:
        members = tf.getmembers()
        names = set()
        for member in members:
            path = PurePosixPath(member.name)
            if path.is_absolute() or '..' in path.parts or '\\' in member.name:
                raise ValueError('Unsafe TAR path')
            if not (member.isfile() or member.isdir()) or member.issparse():
                raise ValueError('Links, special and sparse files are not allowed')
            key = str(path)
            if key in names: raise ValueError('Duplicate TAR path')
            names.add(key)
        logical_end = max((m.offset_data + ((m.size+511)//512)*512 for m in members), default=0)
        with archive.open('rb') as stream:
            stream.seek(logical_end)
            tail = stream.read()
        if len(tail) < 1024 or any(tail): raise ValueError('Missing complete TAR end markers')
        glbs = [m for m in members if m.isfile() and m.name.endswith('.glb')]
        if not glbs: raise ValueError('No GLB scenes in archive')
        # Validate every GLB before writing any asset. Do not treat file names as commands.
        for member in glbs: glb_info(tf.extractfile(member).read())
        output.mkdir(parents=True)
        records = []
        for member in members:
            if not member.isfile(): continue
            data = tf.extractfile(member).read()
            if len(data) != member.size: raise ValueError('Truncated TAR payload')
            record = dict(path=member.name, bytes=len(data), sha256=hashlib.sha256(data).hexdigest(), extracted=member in glbs)
            if record['extracted']:
                dest = output.joinpath(*PurePosixPath(member.name).parts)
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
                record['glb'] = glb_info(data)
            records.append(record)
    after = archive.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('Archive changed during validation')
    report = dict(complete=True, archive=str(archive), archive_bytes=before.st_size,
        archive_sha256=archive_hash, official_source=SOURCE, license=LICENSE,
        scope='Local integrity hashes, not a comparison with a publisher-signed checksum. Official browser provenance is recorded separately.',
        glb_count=len(glbs), members=records)
    (output/'archive-manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('archive', type=Path); p.add_argument('output', type=Path)
    a = p.parse_args(); r = extract(a.archive, a.output)
    print(json.dumps({k:v for k,v in r.items() if k != 'members'}, indent=2))
