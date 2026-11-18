from pathlib import Path
import json,pytest
from stage68_publication import publish_ready
from stage68_monitor_inventory import measure,read_closed_status

def test_ready_visible_only_after_complete_write(tmp_path,monkeypatch):
 final=tmp_path/'initial-bootstrap-ready.json';temporary=final.with_suffix('.tmp');payload=json.dumps({'actual_saved_ready':True})+'\n';original=Path.open;seen=[]
 class Paused:
  def __enter__(self):self.stream=original(temporary,'x');return self
  def write(self,text):
   self.stream.write(text[:5]);self.stream.flush();seen.append((final.exists(),temporary.read_text()));self.stream.write(text[5:]);return len(text)
  def __exit__(self,*args):self.stream.close()
 def open_(self,*args,**kw):return Paused() if self==temporary and args==('x',) else original(self,*args,**kw)
 monkeypatch.setattr(Path,'open',open_);publish_ready(final,payload)
 assert seen==[(False,payload[:5])] and final.read_text()==payload and not temporary.exists()

def test_verbatim_saved_ready_payload(tmp_path):
 text=' {"physical_sim_us":4000,"actual_raw_topics":["sensor_accel","sensor_gyro"]}\n';p=tmp_path/'initial-bootstrap-ready.json';publish_ready(p,text);assert p.read_bytes()==text.encode()

def test_existing_partial_temporary_does_not_overwrite(tmp_path):
 p=tmp_path/'initial-bootstrap-ready.json';p.with_suffix('.tmp').write_text('old')
 with pytest.raises(FileExistsError):publish_ready(p,'new')
 assert not p.exists() and p.with_suffix('.tmp').read_text()=='old'

def test_write_failure_propagates_no_ready(tmp_path,monkeypatch):
 p=tmp_path/'initial-bootstrap-ready.json'
 def fail(*args,**kw):raise PermissionError('explicit-fixture')
 monkeypatch.setattr(Path,'open',fail)
 with pytest.raises(PermissionError):publish_ready(p,'text')
 assert not p.exists()

def test_replace_failure_preserves_temp_and_fails(tmp_path,monkeypatch):
 p=tmp_path/'initial-bootstrap-ready.json'
 def fail(*args,**kw):raise OSError('explicit-fixture')
 monkeypatch.setattr(Path,'replace',fail)
 with pytest.raises(OSError):publish_ready(p,'complete')
 assert not p.exists() and p.with_suffix('.tmp').read_text()=='complete'

def test_published_ready_temporary_rename_inventory_is_named(tmp_path,monkeypatch):
 p=tmp_path/'initial-bootstrap-ready.tmp';p.write_text('actual-fixture');old=Path.lstat
 def race(self,*a,**kw):
  if self==p:p.rename(tmp_path/'initial-bootstrap-ready.json')
  return old(self,*a,**kw)
 monkeypatch.setattr(Path,'lstat',race)
 assert measure(tmp_path)['volatile_temporaries_renamed']==['initial-bootstrap-ready.tmp']


def test_partial_status_pending_is_bounded_and_corruption_still_fails(tmp_path):
 p=tmp_path/'status.json';p.write_text('{');stamp=p.stat().st_mtime
 assert read_closed_status(p,stamp+.1) is None
 with pytest.raises(json.JSONDecodeError):read_closed_status(p,stamp+1.01)

def test_completed_status_returns_exact_original_values(tmp_path):
 p=tmp_path/'status.json';row=dict(closed=True,written=3,write_errors=1);p.write_text(json.dumps(row));assert read_closed_status(p)==row
