import sys,os
from pathlib import Path
S=Path(os.environ.get('STAGE65_BASE','/Users/wuchenjie/Documents/Codex/2026-10-13/task-3-stage45'))
sys.path[:0]=[str(Path(__file__).parent),str(Path(os.environ.get('STAGE65_MEASUREMENT','/tmp/stage65-measurement'))),str(S),str(S/'src'),str(S/'src/annual_planner/src')]
import stage65_capture
import annual_runtime.map_evidence as wiring
wiring.bind_capture=stage65_capture.bind_capture
import pytest
raise SystemExit(pytest.main(sys.argv[1:]))
