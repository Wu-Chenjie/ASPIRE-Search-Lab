"""Exact native sequential fusion diagnostic of saved measured clouds; no flight.

Uses original protocol parameters and actual point order, preserving the native
transport's float32 coordinates and float64 camera origin. This deliberately
does not assert original DDS callback delivery or scheduling equivalence.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--until', type=float, default=float('inf'))
    parser.add_argument('--root', nargs=3, type=float, action='append', required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    recorded = args.capture / 'native-racer/measured-input.jsonl.gz'
    protocol = json.loads((args.capture / 'native-racer/protocol.json').read_text())
    configs = [c for c in protocol['nodes'] if c['executable'] == 'exploration_node']
    if len(configs) != 1:
        raise ValueError('Diagnostic currently requires a single recorded native map')
    binary_input = args.output / 'frames.bin'
    count = 0
    with gzip.open(recorded, 'rt') as source, binary_input.open('wb') as target:
        target.write(b'RCPROV01' + struct.pack('<Q', 0))
        for line in source:
            row = json.loads(line)
            if row['type'] != 'frame' or row['time'] > args.until:
                continue
            if row['drone'] != 0:
                raise ValueError('Unexpected additional drone')
            points = np.asarray(row['points'], dtype='<f4').reshape(-1, 3)
            origin = np.asarray(row['origin'], dtype=np.float64)
            if not np.isfinite(points).all() or not np.isfinite(origin).all():
                raise ValueError('Nonfinite recorded input')
            target.write(struct.pack('<d3dI', row['time'], *origin, len(points)))
            target.write(points.tobytes())
            count += 1
        target.seek(8)
        target.write(struct.pack('<Q', count))
    if not count:
        raise ValueError('No recorded frame selected')
    params = dict(configs[0]['parameters'])
    params.update({'diagnostic.input': str(binary_input),
                   'diagnostic.output': str(args.output / 'provenance.jsonl')})
    params['diagnostic.roots'] = ';'.join(','.join(map(str, root)) for root in args.root)
    param_path = args.output / 'params.yaml'
    param_path.write_text(yaml.safe_dump({'hm3d_provenance_probe': {'ros__parameters': params}}))
    from ament_index_python.packages import get_package_prefix
    binary = Path(get_package_prefix('racer_core')) / 'lib/racer_core/hm3d_provenance_probe'
    command = [str(binary), '--ros-args', '--params-file', str(param_path)]
    # No executor spins in this diagnostic. Keep discovery isolated nevertheless.
    with (args.output / 'native.log').open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
    manifest = dict(scope='Sequential native pointcloud fusion only; no autonomous flight or original callback-delivery equivalence',
                    input=str(recorded), input_sha256=hashlib.sha256(recorded.read_bytes()).hexdigest(),
                    float32_stream_sha256=hashlib.sha256(binary_input.read_bytes()).hexdigest(),
                    params_sha256=hashlib.sha256(param_path.read_bytes()).hexdigest(),
                    binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                    frames=count, until=args.until if np.isfinite(args.until) else None,
                    root=args.root, ros_domain_id=os.environ.get('ROS_DOMAIN_ID'), returncode=result.returncode)
    (args.output / 'protocol.json').write_text(json.dumps(manifest, indent=2) + '\n')
    if result.returncode:
        raise RuntimeError(f'Native provenance diagnostic failed: {result.returncode}')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
