from pathlib import Path,PurePosixPath
import hashlib,json,shutil,sys,tarfile

def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
 return h.hexdigest()

def create(archive,inventory,parent,run):
 parent=Path(parent)
 if not parent.is_absolute() or not str(parent).startswith('/Volumes/ColimaRuntime/runtime/research-'):
  raise ValueError('Analysis parent must be explicit external shadow APFS research path')
 if Path(run).name!=run or run.startswith('.') or '+' in run:raise ValueError('Invalid run identifier')
 parent.mkdir(parents=True,exist_ok=True)
 target=parent/run
 if target.exists():raise FileExistsError(target)
 # Validate ALL member names and expected regular contents before writing files.
 expected={str(PurePosixPath(x['relative']).relative_to(run)):x['sha256'] for x in inventory if x['type']=='file'}
 if not expected:raise ValueError('No expected regular originals')
 with tarfile.open(archive) as tf:
  members=tf.getmembers()
  for m in members:
   p=PurePosixPath(m.name)
   if p.is_absolute() or '..' in p.parts or not p.parts or p.parts[0]!=run:raise ValueError('Invalid archive root/member')
  target.mkdir(exist_ok=False)
  for m in members:
   relative=PurePosixPath(m.name).relative_to(run)
   if str(relative)=='.':continue
   path=target/Path(str(relative))
   if m.isdir():path.mkdir(parents=True,exist_ok=True)
   elif m.isfile():
    path.parent.mkdir(parents=True,exist_ok=True)
    with tf.extractfile(m) as source,path.open('xb') as output:shutil.copyfileobj(source,output,1048576)
 actual={str(p.relative_to(target)):digest(p) for p in target.rglob('*') if p.is_file() and not p.is_symlink()}
 if actual!=expected:raise RuntimeError('Complete regular original content mismatch')
 return dict(analysis_view=str(target),regular_files=len(actual),regular_bytes=sum((target/n).stat().st_size for n in actual),all_file_SHA_equal=True,symlink_and_other_metadata_preserved_in_original_PAX=True,view_contains_regular_content_only=True,source_PAX_preserved=True)

if __name__=='__main__':
 a,i,p,run=sys.argv[1:];print(json.dumps(create(Path(a),json.loads(Path(i).read_text()),p,run),indent=2))
