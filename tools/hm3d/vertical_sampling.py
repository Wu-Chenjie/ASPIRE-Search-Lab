"""Opt-in native RACER viewpoint altitude sampling, keeping all original guards."""
from tools.flight.optimizer_overlay import replace_once


def apply(core):
    header=core/'include/active_perception/frontier_finder.h';s=header.read_text()
    anchor='  double candidate_rmax_'
    s=replace_once(s,anchor,'  bool task_vertical_samples_ = false;\n'+anchor);header.write_text(s)
    p=core/'src/active_perception/frontier_finder.cpp';s=p.read_text()
    anchor='  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);'
    s=replace_once(s,anchor,anchor+'\n  nh.param("frontier/task_vertical_samples", task_vertical_samples_, false);')
    anchor='  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;'
    s=replace_once(s,anchor,'''  const std::vector<double> offsets = task_vertical_samples_ ?
      std::vector<double>{0., -.3, .3, -.6, .6, -.9, .9} : std::vector<double>{0.};
  for (const double dz : offsets)
'''+anchor)
    anchor='const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);'
    s=replace_once(s,anchor,'const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0) + Vector3d(0,0,dz);')
    p.write_text(s)
