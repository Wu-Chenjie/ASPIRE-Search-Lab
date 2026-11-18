// Diagnostic only: recorded float32 clouds -> exact native fusion/inflation.
// No executor, planner, DDS spin, synthetic sensing, or truth input.
#include <plan_env/sdf_map.h>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <cstdint>
#include <stdexcept>
#include <sstream>

namespace fast_planner {
class NativeMapProvenanceProbe {
  SDFMap map_;
  std::ofstream out_;
  std::vector<Eigen::Vector3d> roots_;
  std::vector<Eigen::Vector3i> root_indices_;
  std::unordered_map<int,Eigen::Vector3i> watched_;
  struct Last { double stamp=-1; int frame=-1,hits=0,miss=0; double before=0,after=0; };
  std::unordered_map<int,Last> last_;
  double stamp_=0;
  int frame_=0;
  const pcl::PointCloud<pcl::PointXYZ>* cloud_=nullptr;
  Eigen::Vector3d origin_;
  template<class T> static void read(std::istream& in,T& value) {
    if(!in.read(reinterpret_cast<char*>(&value),sizeof(T))) throw std::runtime_error("Truncated frame stream");
  }
  template<class V> void vec(const V& v) { out_<<'['<<v[0]<<','<<v[1]<<','<<v[2]<<']'; }
  void common(const char* type) { out_<<"{\"type\":\""<<type<<"\",\"frame\":"<<frame_<<",\"time\":"<<stamp_; }
public:
  int run(racer_ros::NodeHandle& nh) {
    std::string input,output;
    nh.param("diagnostic/input",input,std::string(""));
    nh.param("diagnostic/output",output,std::string(""));
    std::string roots;nh.param("diagnostic/roots",roots,std::string(""));
    std::istringstream root_stream(roots);std::string part;
    while(std::getline(root_stream,part,';')){
      std::replace(part.begin(),part.end(),',',' ');std::istringstream values(part);Eigen::Vector3d root;
      if(!(values>>root[0]>>root[1]>>root[2])||!root.allFinite())throw std::runtime_error("Invalid roots");
      roots_.push_back(root);
    }
    if(roots_.empty())throw std::runtime_error("No roots");
    if(input.empty()||output.empty()) throw std::runtime_error("Missing diagnostic input/output");
    map_.initMap(nh);

    const int radius=std::ceil(map_.mp_->obstacles_inflation_/map_.mp_->resolution_);
    for(const auto& root:roots_){
    if(!map_.isInMap(root))throw std::runtime_error("Watch root outside map");
    Eigen::Vector3i root_idx;map_.posToIndex(root,root_idx);root_indices_.push_back(root_idx);
    for(int x=-radius;x<=radius;++x)for(int y=-radius;y<=radius;++y)for(int z=-radius;z<=radius;++z){
      Eigen::Vector3i idx=root_idx+Eigen::Vector3i(x,y,z);
      if(map_.isInMap(idx))watched_.emplace(map_.toAddress(idx),idx);
    }
    }
    out_.open(output);if(!out_)throw std::runtime_error("Cannot open output");out_<<std::setprecision(17);
    out_<<"{\"type\":\"metadata\",\"roots\":[";
    for(size_t i=0;i<roots_.size();++i){if(i)out_<<',';out_<<"{\"position\":";vec(roots_[i]);out_<<",\"index\":";vec(root_indices_[i]);out_<<'}';}out_<<']';
    out_<<",\"map_origin\":";vec(map_.mp_->map_origin_);out_<<",\"map_shape\":";vec(map_.mp_->map_voxel_num_);
    out_<<",\"resolution\":"<<map_.mp_->resolution_<<",\"inflation_radius_cells\":"<<radius
        <<",\"occupancy_log_threshold\":"<<map_.mp_->min_occupancy_log_<<",\"hit_log\":"<<map_.mp_->prob_hit_log_
        <<",\"miss_log\":"<<map_.mp_->prob_miss_log_<<",\"clamp_min_log\":"<<map_.mp_->clamp_min_log_
        <<",\"clamp_max_log\":"<<map_.mp_->clamp_max_log_<<",\"max_ray_length\":"<<map_.mp_->max_ray_length_
        <<",\"scope\":\"Exact native sequential fusion, float32 point inputs; no online DDS callback receipt history\"}\n";
    map_.diagnostic_endpoint_observer_=[&](int point,int adr,int hit,const Eigen::Vector3d& effective){
      if(!watched_.count(adr))return;
      common("endpoint");out_<<",\"address\":"<<adr<<",\"index\":";vec(watched_.at(adr));
      out_<<",\"point_index\":"<<point<<",\"hit\":"<<hit<<",\"raw_float32\":";
      const auto& p=cloud_->points[point];vec(Eigen::Vector3d(p.x,p.y,p.z));
      out_<<",\"effective\":";vec(effective);out_<<",\"origin\":";vec(origin_);out_<<"}\n";
    };
    map_.diagnostic_fusion_observer_=[&](int adr,int hits,int miss,double before,double after){
      if(!watched_.count(adr))return;
      last_[adr]=Last{stamp_,frame_,hits,miss,before,after};
      common("fusion");out_<<",\"address\":"<<adr<<",\"index\":";vec(watched_.at(adr));
      out_<<",\"hit_count\":"<<hits<<",\"miss_flag\":"<<miss<<",\"before\":"<<before<<",\"after\":"<<after<<"}\n";
    };
    std::ifstream in(input,std::ios::binary);char magic[8];in.read(magic,8);
    if(std::string(magic,8)!="RCPROV01")throw std::runtime_error("Bad stream magic");
    std::uint64_t count;read(in,count);
    for(frame_=0;frame_<static_cast<int>(count);++frame_){
      read(in,stamp_);for(int k=0;k<3;++k)read(in,origin_[k]);
      std::uint32_t n;read(in,n);if(n>10000000)throw std::runtime_error("Invalid point count");
      pcl::PointCloud<pcl::PointXYZ> cloud;cloud.resize(n);
      for(auto& p:cloud){read(in,p.x);read(in,p.y);read(in,p.z);}
      cloud_=&cloud;
      map_.inputPointCloud(cloud,n,origin_);
      // The production cloud callback invokes the same native routine immediately.
      if(n)map_.clearAndInflateLocalMap();
      common("snapshot");out_<<",\"origin\":";vec(origin_);out_<<",\"point_count\":"<<n<<",\"queries\":[";
      for(size_t query=0;query<roots_.size();++query){
        if(query)out_<<',';const auto& idx=root_indices_[query];
        out_<<"{\"query\":"<<query<<",\"root_occupancy\":"<<map_.getOccupancy(idx)<<",\"root_inflated\":"<<map_.getInflateOccupancy(idx)
            <<",\"root_logodds\":"<<map_.md_->occupancy_buffer_[map_.toAddress(idx)]<<",\"occupied_support\":[";
        bool first=true;
        for(const auto& w:watched_){
          if((w.second-idx).cwiseAbs().maxCoeff()>radius||map_.getOccupancy(w.second)!=SDFMap::OCCUPIED)continue;
          if(!first)out_<<',';first=false;Eigen::Vector3d center;map_.indexToPos(w.second,center);
          out_<<"{\"address\":"<<w.first<<",\"index\":";vec(w.second);out_<<",\"center\":";vec(center);
          out_<<",\"logodds\":"<<map_.md_->occupancy_buffer_[w.first];
          if(last_.count(w.first)){const auto& l=last_.at(w.first);out_<<",\"last_update_time\":"<<l.stamp<<",\"last_update_frame\":"<<l.frame<<",\"last_hit_count\":"<<l.hits<<",\"last_miss_flag\":"<<l.miss;}
          out_<<'}';
        }
        out_<<"],\"kernel_matches\":"<<((map_.getInflateOccupancy(idx)==int(!first))?"true":"false")<<'}';
      }
      out_<<"]}\n";
    }
    if(in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Trailing input bytes");
    out_.flush();return 0;
  }
};
}

int main(int argc,char**argv){
  racer_ros::init(argc,argv,"hm3d_provenance_probe");racer_ros::NodeHandle nh("~");
  try { fast_planner::NativeMapProvenanceProbe probe;return probe.run(nh); }
  catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}
}
