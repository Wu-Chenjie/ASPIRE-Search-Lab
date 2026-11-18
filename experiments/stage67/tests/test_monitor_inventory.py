from pathlib import Path
import pytest
from stage67_monitor_inventory import measure

def test_stable_files_and_original_health_exclusion(tmp_path):
 (tmp_path/'a').write_bytes(b'123');d=tmp_path/'map-evidence'/'drone-0-mapper';d.mkdir(parents=True);(d/'record.npz').write_bytes(b'56789');(d/'health.tmp').write_bytes(b'excluded');(tmp_path/'link').symlink_to(tmp_path/'a')
 assert measure(tmp_path)==dict(capture_bytes=8197,other_bytes=3,volatile_temporaries_renamed=[])

def test_actual_atomic_rename_between_discovery_and_stat_is_recorded(tmp_path,monkeypatch):
 p=tmp_path/'observer-status.tmp';p.write_bytes(b'123');old=Path.lstat
 def race(self,*a,**kw):
  if self==p:p.rename(tmp_path/'observer-status.json')
  return old(self,*a,**kw)
 monkeypatch.setattr(Path,'lstat',race)
 assert measure(tmp_path)['volatile_temporaries_renamed']==['observer-status.tmp']

def test_persistent_missing_input_still_fails(tmp_path,monkeypatch):
 p=tmp_path/'raw.jsonl';p.write_bytes(b'123');old=Path.lstat
 def race(self,*a,**kw):
  if self==p:p.unlink()
  return old(self,*a,**kw)
 monkeypatch.setattr(Path,'lstat',race)
 with pytest.raises(FileNotFoundError):measure(tmp_path)

def test_unknown_temporary_still_fails(tmp_path,monkeypatch):
 p=tmp_path/'unknown.tmp';p.write_bytes(b'123');old=Path.lstat
 def race(self,*a,**kw):
  if self==p:p.unlink()
  return old(self,*a,**kw)
 monkeypatch.setattr(Path,'lstat',race)
 with pytest.raises(FileNotFoundError):measure(tmp_path)

def test_permission_error_not_suppressed(tmp_path,monkeypatch):
 p=tmp_path/'observer-status.tmp';p.write_bytes(b'123');old=Path.lstat
 def fail(self,*a,**kw):
  if self==p:raise PermissionError('explicit-test')
  return old(self,*a,**kw)
 monkeypatch.setattr(Path,'lstat',fail)
 with pytest.raises(PermissionError):measure(tmp_path)

def test_nested_same_name_not_an_exemption(tmp_path,monkeypatch):
 p=tmp_path/'unexpected'/'observer-status.tmp';p.parent.mkdir();p.write_bytes(b'123');old=Path.lstat
 def race(self,*a,**kw):
  if self==p:p.unlink()
  return old(self,*a,**kw)
 monkeypatch.setattr(Path,'lstat',race)
 with pytest.raises(FileNotFoundError):measure(tmp_path)
