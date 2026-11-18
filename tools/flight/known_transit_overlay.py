"""Optional measured-space transit for native point-to-view proposals."""
from pathlib import Path
from .optimizer_overlay import replace_once


def apply(core):
    header = core / 'include/plan_manage/planner_manager.h'
    anchor = '  void initPlanModules('
    header.write_text(replace_once(header.read_text(), anchor, '''  bool planKnownTransit(const Eigen::Vector3d& start, const Eigen::Vector3d& vel,
      const Eigen::Vector3d& acc, const Eigen::Vector3d& yaw, const Eigen::Vector3d& goal,
      double view_yaw);
''' + anchor))
    manager = core / 'src/plan_manage/planner_manager.cpp'
    method = Path(__file__).with_name('known_transit.cpp').read_text()
    anchor = 'void FastPlannerManager::planExploreTraj('
    manager.write_text('#include <cstdlib>\n' + replace_once(manager.read_text(), anchor, method + '\n' + anchor))
    exploration = core / 'src/exploration_manager/fast_exploration_manager.cpp'
    anchor = '  if (len < radius_close || optimistic) {'
    text = replace_once(exploration.read_text(), anchor, '''  const bool transit = planner_manager_->planKnownTransit(pos, vel, acc, yaw, next_pos, next_yaw);
  if (transit) {
    ed_->next_goal_ = next_pos;
    if (optimistic) ++ed_->plan_num_;
  } else ''' + anchor.strip())
    call = '  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);'
    exploration.write_text(replace_once(text, call, '  if (!transit) ' + call.strip()))
