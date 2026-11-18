"""Recompute local native inflation with an occupied-source halo.

The old routine erased contributions from occupied cells outside the cleared
region and did not cover the full vertical influence of changed occupancy.
Keep the exact original cubic kernel, radius, resolution and unknown policy.
"""
def apply(core):
    p=core/'src/plan_env/sdf_map.cpp';s=p.read_text()
    start=s.index('  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);')
    end=s.index('\n}\n\ndouble SDFMap::getResolution()',start)
    s=s[:start]+'''  const int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
  // Changed occupied cells can affect a full kernel beyond the raw update box.
  Eigen::Vector3i target_min = md_->local_bound_min_ - Eigen::Vector3i::Constant(inf_step);
  Eigen::Vector3i target_max = md_->local_bound_max_ + Eigen::Vector3i::Constant(inf_step);
  boundIndex(target_min); boundIndex(target_max);
  // Occupied cells outside that target can still contribute into it.
  Eigen::Vector3i source_min = target_min - Eigen::Vector3i::Constant(inf_step);
  Eigen::Vector3i source_max = target_max + Eigen::Vector3i::Constant(inf_step);
  boundIndex(source_min); boundIndex(source_max);
  for(int x=target_min.x();x<=target_max.x();++x)
    for(int y=target_min.y();y<=target_max.y();++y)
      for(int z=target_min.z();z<=target_max.z();++z)
        md_->occupancy_buffer_inflate_[toAddress(x,y,z)]=0;
  for(int x=source_min.x();x<=source_max.x();++x)
    for(int y=source_min.y();y<=source_max.y();++y)
      for(int z=source_min.z();z<=source_max.z();++z) {
        if(md_->occupancy_buffer_[toAddress(x,y,z)]<=mp_->min_occupancy_log_)continue;
        for(int ix=std::max(target_min.x(),x-inf_step);ix<=std::min(target_max.x(),x+inf_step);++ix)
          for(int iy=std::max(target_min.y(),y-inf_step);iy<=std::min(target_max.y(),y+inf_step);++iy)
            for(int iz=std::max(target_min.z(),z-inf_step);iz<=std::min(target_max.z(),z+inf_step);++iz)
              md_->occupancy_buffer_inflate_[toAddress(ix,iy,iz)]=1;
      }
  // ESDF must include the same changed inflation target.
  md_->local_bound_min_=target_min;md_->local_bound_max_=target_max;
''' +s[end:];p.write_text(s)
