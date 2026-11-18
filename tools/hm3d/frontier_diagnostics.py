"""Instrument native RACER sampling rejection causes on a generated build only."""
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    p=core/'src/active_perception/frontier_finder.cpp';s=p.read_text()
    begin=s.index('void FrontierFinder::sampleViewpoints(Frontier& frontier) {')
    end=s.index('\nbool FrontierFinder::isFrontierCovered()',begin)
    part=s[begin:end]
    part=part.replace('// Evaluate sample viewpoints on circles, find ones that cover most cells',
        'int bounds_rejected=0,inflated_rejected=0,unknown_rejected=0,visibility_rejected=0;\n  // Evaluate sample viewpoints on circles, find ones that cover most cells')
    old='''      if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
        continue;'''
    part=replace_once(part,old,'''      if(!edt_env_->sdf_map_->isInBox(sample_pos)){++bounds_rejected;continue;}
      if(edt_env_->sdf_map_->getInflateOccupancy(sample_pos)==1){++inflated_rejected;continue;}
      if(isNearUnknown(sample_pos)){++unknown_rejected;continue;}''')
    part=part.replace('      if (visib_num > min_visib_num_) {','      if(visib_num<=min_visib_num_)++visibility_rejected;\n      if (visib_num > min_visib_num_) {')
    at=part.rfind('\n}')
    part=part[:at]+'''\n  ROS_INFO("HM3D_FRONTIER cells=%zu center=%.3f,%.3f,%.3f bounds=%d inflated=%d unknown=%d invisible=%d accepted=%zu",frontier.cells_.size(),frontier.average_.x(),frontier.average_.y(),frontier.average_.z(),bounds_rejected,inflated_rejected,unknown_rejected,visibility_rejected,frontier.viewpoints_.size());'''+part[at:]
    p.write_text(s[:begin]+part+s[end:])
