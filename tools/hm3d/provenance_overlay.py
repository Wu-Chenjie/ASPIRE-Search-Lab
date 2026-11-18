"""Opt-in diagnostic observers around unchanged native occupancy fusion.

Only generated diagnostic builds receive these hooks. Empty observers do not
change map state, thresholds, ray traversal, update bounds or inflation.
"""
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    header = core / 'include/plan_env/sdf_map.h'
    text = header.read_text()
    text = text.replace('#include <queue>', '#include <queue>\n#include <functional>')
    anchor = '  friend MapROS;'
    text = replace_once(text, anchor, anchor + '''
  friend class NativeMapProvenanceProbe;
  std::function<void(int,int,int,double,double)> diagnostic_fusion_observer_;
  std::function<void(int,int,int,const Eigen::Vector3d&)> diagnostic_endpoint_observer_;
''')
    header.write_text(text)
    source = core / 'src/plan_env/sdf_map.cpp'
    text = source.read_text()
    anchor = '    setCacheOccupancy(vox_adr, tmp_flag);'
    text = replace_once(text, anchor, anchor + '''
    if (diagnostic_endpoint_observer_) diagnostic_endpoint_observer_(i, vox_adr, tmp_flag, pt_w);
''')
    anchor = '    double log_odds_update ='
    text = replace_once(text, anchor, '''    const int diagnostic_hits = md_->count_hit_[adr];
    const int diagnostic_miss = md_->count_miss_[adr];
    const double diagnostic_before = md_->occupancy_buffer_[adr];
''' + anchor)
    anchor = '            mp_->clamp_max_log_);'
    text = replace_once(text, anchor, anchor + '''
    if (diagnostic_fusion_observer_) diagnostic_fusion_observer_(adr, diagnostic_hits,
        diagnostic_miss, diagnostic_before, md_->occupancy_buffer_[adr]);
''')
    source.write_text(text)
