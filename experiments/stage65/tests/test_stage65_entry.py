import importlib.util,json
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('private_flight_entry','/tmp/stage65-measurement/stage65-flight-entry.py')
entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
class Process:
 def __init__(self,value):self.returncode=value
 def poll(self):return self.returncode

def test_pending_last_actual_feedback_does_not_finish(tmp_path):
 assert entry.paced_completion(Process(None),tmp_path) is False

def test_real_completion_requires_result_and_native_success(tmp_path):
 p=tmp_path/'pacer-result.json'
 p.write_text(json.dumps(dict(completed=True,errors=[],counts=dict(queue_drop=0))))
 assert entry.paced_completion(Process(0),tmp_path)
 with pytest.raises(RuntimeError):entry.paced_completion(Process(2),tmp_path)

@pytest.mark.parametrize('row',[dict(completed=False,errors=[],counts={}),dict(completed=True,errors=['lost'],counts={}),dict(completed=True,errors=[],counts=dict(queue_drop=1))])
def test_capture_failures_are_not_success(tmp_path,row):
 (tmp_path/'pacer-result.json').write_text(json.dumps(row))
 with pytest.raises(RuntimeError):entry.paced_completion(Process(0),tmp_path)

def test_no_missing_result_fallback(tmp_path):
 with pytest.raises(FileNotFoundError):entry.paced_completion(Process(0),tmp_path)
