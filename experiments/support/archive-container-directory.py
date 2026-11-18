import sys,json,subprocess,hashlib,tarfile,time
from pathlib import Path
root=Path('/Volumes/WuChenjie/Mac-runtime/runtime-stage52-20261018T1315Z');guest=sys.argv[1];alias=sys.argv[2];assert guest.startswith('/tmp/') and '/' not in alias and alias
D=['docker','--context','colima-racer-gvp','exec','-e','PYTHONDONTWRITEBYTECODE=1','annual-px4-research'];parent=str(Path(guest).parent);name=Path(guest).name
code="""from pathlib import Path
import os,stat,json,hashlib,sys
root=Path(sys.argv[1]);rows=[]
for p in [root,*sorted(root.rglob('*'))]:
 s=p.lstat();t='file' if stat.S_ISREG(s.st_mode) else 'dir' if stat.S_ISDIR(s.st_mode) else 'symlink' if stat.S_ISLNK(s.st_mode) else 'other';x={'relative':str(p.relative_to(root.parent)),'mode':stat.S_IMODE(s.st_mode),'uid':s.st_uid,'gid':s.st_gid,'mtime_ns':s.st_mtime_ns,'bytes':s.st_size,'type':t}
 if t=='file':x['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
 if t=='symlink':x['link']=os.readlink(p)
 rows.append(x)
print(json.dumps(rows))
"""
def inventory():
 p=subprocess.run(D+['python3','-c',code,guest],capture_output=True,text=True);assert p.returncode==0,p.stderr;return json.loads(p.stdout)
rows=inventory();assert not any(x['type']=='other' for x in rows)
manifest=root/'audit'/(alias+'-source-manifest.json');archive=root/'evidence'/(alias+'.pax.tar');proof=root/'audit'/(alias+'-ARCHIVE-VERIFIED.json');assert not manifest.exists() and not archive.exists() and not proof.exists();manifest.write_text(json.dumps(rows,indent=2))
with archive.open('xb') as out,(root/'logs'/(alias+'-tar.stderr.txt')).open('wb') as err:p=subprocess.run(D+['tar','--format=pax','--xattrs','--acls','--numeric-owner','-C',parent,'-cf','-',name],stdout=out,stderr=err)
assert p.returncode==0
found={}
with tarfile.open(archive,'r:') as tf:
 for m in tf:
  t='file' if m.isfile() else 'dir' if m.isdir() else 'symlink' if m.issym() else 'other';x={'type':t,'mode':m.mode,'uid':m.uid,'gid':m.gid}
  if t=='file':x.update(bytes=m.size,sha256=hashlib.sha256(tf.extractfile(m).read()).hexdigest())
  if t=='symlink':x['link']=m.linkname
  found[str(Path(m.name))]=x
assert len(found)==len(rows)
for x in rows:
 y=found[x['relative']];keys=['type','mode','uid','gid']+(['bytes','sha256'] if x['type']=='file' else ['link'] if x['type']=='symlink' else []);assert all(x[k]==y[k] for k in keys),(x,y)
assert rows==inventory(),'source changed during archive'
summary={'guest_root':guest,'archive':str(archive),'entries':len(rows),'regular_files':sum(x['type']=='file' for x in rows),'content_bytes':sum(x['bytes'] for x in rows if x['type']=='file'),'archive_bytes':archive.stat().st_size,'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'source_stable':True,'all_content_types_modes_uid_gid_symlink_targets_equal':True,'source_preserved':True,'same_physical_disk':True,'independent_media_backup':False,'utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())};proof.write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
