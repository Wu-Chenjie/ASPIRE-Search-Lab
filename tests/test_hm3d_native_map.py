"""Native invariant regression on identical saved sensor inputs, no oracle map."""
import json,os,subprocess,sys
from pathlib import Path
import pytest

pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix


def test_native_inflation_matches_single_locked_kernel(tmp_path):
    capture=os.environ.get('SEARCH_HM3D_CAPTURE')
    if not capture:pytest.skip('Supply saved static HM3D capture for native diagnostic regression')
    binary=Path(get_package_prefix('racer_core'))/'lib/racer_core/hm3d_map_probe'
    if not binary.is_file():pytest.skip('Build diagnostic map probe first')
    from tools.hm3d.snapshot_audit import audit
    out=tmp_path/'native-map'
    subprocess.run([sys.executable,'-m','tools.hm3d.replay_map_probe',capture,str(out),'--incremental'],check=True,timeout=90)
    result=audit(out)
    assert result['unknown']>0, 'Unobserved space must remain unknown'
    assert result['occupied']>0
    assert result['missing_inflation']==0
    assert result['inflated_without_current_obstacle']==0
    config=json.loads((out/'protocol.json').read_text())['hello']
    assert config['body_clearance']==.6 and not config['vertical_samples']
