"""Verify immutable RACER snapshot and retained assets; report task modifications."""
import hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from tools.racer_ros2.verify import verify
verify(ROOT/'external/racer_ros2_ws')
m=json.loads((ROOT/'docs/source-manifest.json').read_text());changed=[]
for name,digest in m['files'].items():
 p=ROOT/name
 if not p.is_file():raise ValueError('Missing retained source: '+name)
 if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
  if name.startswith('external/'):raise ValueError('Changed locked dependency: '+name)
  changed.append(name)
print(json.dumps({'immutable_dependencies':'PASS','task_modified_copied_files':changed},indent=2))
