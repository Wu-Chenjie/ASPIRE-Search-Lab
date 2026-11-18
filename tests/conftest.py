import sys
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# Research proof helpers live under tools/, not in the installed ROS package.
# Native CI deliberately runs from /tmp; expose only the repository root so
# core/annual_runtime still resolve from the installation in installed mode.
sys.path.append(str(ROOT))
PACKAGE = ROOT / 'src' / 'annual_planner'
if os.environ.get('ANNUAL_TEST_INSTALLED') != '1':
    sys.path[:0] = [str(ROOT / 'src'), str(PACKAGE / 'src')]
