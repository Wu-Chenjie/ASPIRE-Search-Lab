"""The same geometric selector built into RACER must respect the measured side."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import pytest


def test_native_frontier_steps_follow_mirrored_measured_space(tmp_path):
    compiler = shutil.which('c++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    pkg_config = shutil.which('pkg-config')
    probe = subprocess.run([pkg_config, '--cflags', 'eigen3'], capture_output=True, text=True) if pkg_config else None
    if probe is not None and probe.returncode == 0:
        flags = shlex.split(probe.stdout)
    else:
        incl = Path(os.environ.get('EIGEN3_INCLUDE_DIR', '/usr/include/eigen3'))
        if not (incl / 'Eigen/Core').is_file():
            pytest.skip('Eigen development headers unavailable')
        flags = ['-I', str(incl)]
    root = Path(__file__).resolve().parents[1]
    exe = tmp_path / 'frontier-steps-test'
    subprocess.run([compiler, '-std=c++17', *flags, '-I', str(root / 'tools/flight'),
                    str(root / 'tests/frontier_steps_test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


def test_frontier_steps_run_after_bearing_on_settled_exploration_only(tmp_path):
    root = Path(__file__).resolve().parents[1]
    out = tmp_path / 'native'
    subprocess.run([sys.executable, str(root / 'tools/search/build_native.py'), str(out),
                    '--execution-safety', '--consistent-inflation'], check=True)
    src = out / 'src/racer_core/src/exploration_manager'
    manager = (src / 'fast_exploration_manager.cpp').read_text()
    assert 'nh.param("sdf_map/obstacles_inflation", frontier_step_radius_, -1.0);' in manager
    assert 'sdf_map_->mp_' not in manager
    plan = manager[manager.index('int FastExplorationManager::planExploreMotion('):
                   manager.index('int FastExplorationManager::planTrajToView(')]
    assert (plan.index('const bool bearing_selected = taskBearingRerank(')
            < plan.index('ep_->drone_num_ == 1 && !bearing_selected &&\n      !task_static_evaluation_')
            < plan.index('aspire_steps::refine(')
            < plan.index('planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw)'))
    assert 'if (!prefer_observation) return FAIL;' in plan
    assert 'planTrajToView(pos, vel, acc, yaw, tour_point, tour_yaw)' in plan
    assert 'ed_->frontiers_[next_frontier]' in plan
    fsm = (src / 'fast_exploration_fsm.cpp').read_text()
    assert ('res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,\n'
            '        fd_->start_yaw_, live_execution_ && task_stop_and_plan_ && fd_->static_state_);') in fsm
    planner = fsm[fsm.index('int FastExplorationFSM::callExplorationPlanner('):
                  fsm.index('void FastExplorationFSM::visualize(')]
    assert planner.index('settled_reference_id_ == executed_id_') < planner.index(
        'fd_->start_pt_ = settled_reference_position_;') < planner.index('planExploreMotion(')
    assert 'fd_->start_vel_.setZero();\n    fd_->start_acc_.setZero();' in planner
    assert '!task_review_enabled_ && !task_static_replan_' in planner
    assert 'ref[0] == executed_id_ && (executed_id_ < 0 || executed_done_)' in fsm
