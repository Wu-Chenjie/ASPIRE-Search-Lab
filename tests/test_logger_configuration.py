from pathlib import Path
import pytest
from tools.flight.logger_configuration import install_logger_topics

def stock(tmp_path):
 b=tmp_path/'build';p=b/'etc/init.d-posix/airframes';p.mkdir(parents=True);(p/'4002_gz_x500_depth').write_text('original airframe\n');(b/'etc/init.d').mkdir();(b/'etc/init.d/rc.logging').write_text('original logging startup\n');(b/'etc/extras').mkdir();(b/'etc/extras/metadata').write_bytes(b'fixed');return b

def test_logger_install_preserves_px4_autostart_glob_and_stock_bytes(tmp_path):
 b=stock(tmp_path);before={str(p.relative_to(b)):p.read_bytes() for p in b.rglob('*') if p.is_file()};w=tmp_path/'work';w.mkdir();topics=tmp_path/'topics.txt';topics.write_text('trajectory_setpoint 0 0\n')
 v=install_logger_topics(w,b,topics);files=list((w/'etc/init.d-posix/airframes').glob('4002_*'));assert len(files)==1 and files[0].read_text()=='original airframe\n';assert (w/'etc/init.d/rc.logging').read_text()=='original logging startup\n';assert (w/'etc/logging/logger_topics.txt').read_bytes()==topics.read_bytes();assert not (b/'etc/logging').exists();assert before=={str(p.relative_to(b)):p.read_bytes() for p in b.rglob('*') if p.is_file()};assert (w/'etc/init.d-posix').is_symlink() and len(v['preserved_resource_links'])==3

def test_original_naive_mkdir_reproduces_missing_autostart_without_running_px4(tmp_path):
 b=stock(tmp_path);w=tmp_path/'work';(w/'etc/logging').mkdir(parents=True);assert list((w/'etc/init.d-posix/airframes').glob('4002_*'))==[];topics=tmp_path/'topics';topics.write_text('trajectory_setpoint 0 0\n')
 with pytest.raises(ValueError,match='existing'):install_logger_topics(w,b,topics)

def test_missing_original_resources_rejected_before_shadowing(tmp_path):
 b=tmp_path/'empty';b.mkdir();w=tmp_path/'work';w.mkdir();topics=tmp_path/'topics';topics.write_text('trajectory_setpoint 0 0\n')
 with pytest.raises(ValueError,match='airframe'):install_logger_topics(w,b,topics)
 assert not (w/'etc').exists()
