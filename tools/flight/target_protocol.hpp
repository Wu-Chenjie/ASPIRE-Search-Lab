#pragma once
#include "target_registry.hpp"
#include "review_protocol.hpp"
#pragma push_macro("inf")
#undef inf
#include <boost/property_tree/json_parser.hpp>
#pragma pop_macro("inf")
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace racer_task {
using TargetTree=boost::property_tree::ptree;
inline std::uint64_t targetUnsigned(const TargetTree&t,const std::string&key){
 auto s=t.get<std::string>(key);if(s.empty()||s.size()>20||!std::all_of(s.begin(),s.end(),[](unsigned char c){return c>='0'&&c<='9';}))
  throw std::runtime_error("invalid_integer_"+key);
 return std::stoull(s);
}
inline TargetTree parseTargetTree(const std::string&text){
 if(text.size()>131072)throw std::runtime_error("target_message_too_large");
 std::istringstream in(text);TargetTree t;boost::property_tree::read_json(in,t);
 std::set<std::string> names;for(const auto&kv:t)if(!names.insert(kv.first).second)throw std::runtime_error("duplicate_target_key");
 return t;
}
inline TargetVec targetReadVec(const TargetTree&t,const std::string&prefix){
 return {t.get<double>(prefix+"x"),t.get<double>(prefix+"y"),t.get<double>(prefix+"z")};
}
inline TargetObservation readTargetObservation(const TargetTree&t){
 static const std::set<std::string> fields{
  "schema","kind","now_s","coobserved_counters","robot","session","counter","sequence","revision","first_s","capture_s","frame",
  "frame_version","map_epoch","clock_domain","observation_id","shared_aligned","visual_state","localization_state","has_position","has_bearing",
  "identity_input_state","identity_input_deadline_s","identity_surface_policy","ix","iy","iz","identity_radius_m",
  "surface_quality_policy","support_pixels","valid_depth_fraction","component_fraction","depth_span_m","rgb_depth_skew_s","calibration_id","covariance_calibrated",
  "px","py","pz","radius_m","ox","oy","oz","dx","dy","dz","half_angle_rad","confidence",
  "support_captures","support_first_s","support_policy","camera_half_fov_rad","sync_for","sync_request","sync_more","sync_after"
 };
 std::set<std::string> names;
 for(const auto&kv:t){
  if(!names.insert(kv.first).second||!fields.count(kv.first)||!kv.second.empty())
   throw std::runtime_error("unsupported_target_field");
 }
 // Optional for frozen legacy recordings; reject unknown navigation contracts.
 auto support_policy=t.get<std::string>("support_policy", "legacy_rgb");
 if(support_policy!="legacy_rgb"&&support_policy!="navigation_rgb_v1")
  throw std::runtime_error("unsupported_navigation_support_policy");
 TargetObservation o;o.support_policy=support_policy;o.key={t.get<std::string>("robot"),t.get<std::string>("session"),targetUnsigned(t,"counter")};
 std::istringstream co(t.get<std::string>("coobserved_counters",""));std::string token;
 while(std::getline(co,token,',')){TargetTree n;n.put("id",token);o.exclusions.push_back(targetUnsigned(n,"id"));}
 o.sequence=targetUnsigned(t,"sequence");o.revision=targetUnsigned(t,"revision");
 o.first=t.get<double>("first_s");o.capture=t.get<double>("capture_s");
 o.frame=t.get<std::string>("frame");o.frame_version=t.get<std::string>("frame_version");
 o.map_epoch=t.get<std::string>("map_epoch",o.frame_version);o.clock_domain=t.get<std::string>("clock_domain","mission");
 o.observation_id=t.get<std::string>("observation_id","");
 if(t.count("surface_quality_policy")){
  if(t.get<std::string>("surface_quality_policy")!="visible_support_v1"||t.get<bool>("covariance_calibrated"))throw std::runtime_error("unsupported_surface_quality_contract");
  o.has_surface_quality=true;o.support_pixels=t.get<int>("support_pixels");o.valid_depth_fraction=t.get<double>("valid_depth_fraction");
  o.component_fraction=t.get<double>("component_fraction");o.depth_span_m=t.get<double>("depth_span_m");
  o.rgb_depth_skew_s=t.get<double>("rgb_depth_skew_s");o.calibration_id=t.get<std::string>("calibration_id");
 }else if(t.count("support_pixels")||t.count("valid_depth_fraction")||t.count("component_fraction")||t.count("depth_span_m")||t.count("rgb_depth_skew_s")||t.count("calibration_id")||t.count("covariance_calibrated"))throw std::runtime_error("surface_quality_policy_required");
 o.shared_aligned=t.get<bool>("shared_aligned");o.visual=t.get<std::string>("visual_state");
 o.localization=t.get<std::string>("localization_state");o.has_position=t.get<bool>("has_position");
 o.has_bearing=t.get<bool>("has_bearing");o.confidence=t.get<double>("confidence");
 o.identity_input=t.get<std::string>("identity_input_state","unknown");
 if(o.identity_input!="unknown")o.identity_deadline=t.get<double>("identity_input_deadline_s");
 else if(t.count("identity_input_state")||t.count("identity_input_deadline_s"))throw std::runtime_error("explicit_identity_state_required");
 auto identity_policy=t.get_optional<std::string>("identity_surface_policy");
 bool identity_fields=t.count("ix")||t.count("iy")||t.count("iz")||t.count("identity_radius_m");
 if(identity_policy){
  if(*identity_policy!="registered_surface_anchor_v1")throw std::runtime_error("unsupported_identity_surface_policy");
  o.has_identity_surface=true;o.identity_surface=targetReadVec(t,"i");o.identity_radius=t.get<double>("identity_radius_m");
 }else if(identity_fields)throw std::runtime_error("identity_surface_policy_required");
 if(o.has_position){o.position=targetReadVec(t,"p");o.radius=t.get<double>("radius_m");}
 if(o.has_bearing){
  o.origin=targetReadVec(t,"o");o.direction=targetReadVec(t,"d");
  o.half_angle=t.get<double>("half_angle_rad");o.hfov=t.get<double>("camera_half_fov_rad");
  o.support=t.get<int>("support_captures");o.support_first=t.get<double>("support_first_s");
  if(!std::isfinite(o.hfov)||o.hfov<=0||o.hfov>=1.57||o.support<0||
    !std::isfinite(o.support_first)||o.support_first>o.capture)throw std::runtime_error("invalid_bearing_support");
 }
 return o;
}
inline std::string targetNumber(double v){
 if(!std::isfinite(v))return "null";std::ostringstream out;out<<std::setprecision(17)<<v;return out.str();
}
inline std::string targetVector(const TargetVec&v){
 return "["+targetNumber(v[0])+","+targetNumber(v[1])+","+targetNumber(v[2])+"]";
}
inline std::string targetKeyString(const TargetKey&k){
 return k.robot+"/"+k.session+"/"+std::to_string(k.counter);
}
inline std::string targetObservationJSON(const TargetObservation&o,std::uint64_t revision,
                                       const std::string&kind="target_summary"){
 std::ostringstream s;s<<"{\"schema\":1,\"kind\":"<<reviewJsonString(kind)
  <<",\"robot\":"<<reviewJsonString(o.key.robot)<<",\"session\":"<<reviewJsonString(o.key.session)
  <<",\"counter\":"<<reviewJsonString(std::to_string(o.key.counter))
  <<",\"sequence\":"<<reviewJsonString(std::to_string(o.sequence))
  <<",\"revision\":"<<reviewJsonString(std::to_string(revision))
  <<",\"first_s\":"<<targetNumber(o.first)<<",\"capture_s\":"<<targetNumber(o.capture)
  <<",\"frame\":"<<reviewJsonString(o.frame)<<",\"frame_version\":"<<reviewJsonString(o.frame_version)
  <<",\"map_epoch\":"<<reviewJsonString(o.map_epoch.empty()?o.frame_version:o.map_epoch)<<",\"clock_domain\":"<<reviewJsonString(o.clock_domain)
  <<",\"observation_id\":"<<reviewJsonString(o.observation_id)
  <<",\"shared_aligned\":"<<(o.shared_aligned?"true":"false")
  <<",\"visual_state\":"<<reviewJsonString(o.visual)
  <<",\"localization_state\":"<<reviewJsonString(o.localization)
  <<",\"support_policy\":"<<reviewJsonString(o.support_policy)
  <<",\"has_position\":"<<(o.has_position?"true":"false")
  <<",\"has_bearing\":"<<(o.has_bearing?"true":"false")
  <<",\"confidence\":"<<targetNumber(o.confidence);
 if(o.has_surface_quality)s<<",\"surface_quality_policy\":\"visible_support_v1\",\"support_pixels\":"<<o.support_pixels
  <<",\"valid_depth_fraction\":"<<targetNumber(o.valid_depth_fraction)<<",\"component_fraction\":"<<targetNumber(o.component_fraction)
  <<",\"depth_span_m\":"<<targetNumber(o.depth_span_m)<<",\"rgb_depth_skew_s\":"<<targetNumber(o.rgb_depth_skew_s)
  <<",\"calibration_id\":"<<reviewJsonString(o.calibration_id)<<",\"covariance_calibrated\":false";
 if(kind=="source_record"&&o.identity_input!="unknown")s<<",\"identity_input_state\":"<<reviewJsonString(o.identity_input)
  <<",\"identity_input_deadline_s\":"<<targetNumber(o.identity_deadline);
 if(kind=="source_record"&&o.has_identity_surface)s<<",\"identity_surface_policy\":\"registered_surface_anchor_v1\",\"ix\":"<<targetNumber(o.identity_surface[0])
  <<",\"iy\":"<<targetNumber(o.identity_surface[1])<<",\"iz\":"<<targetNumber(o.identity_surface[2])<<",\"identity_radius_m\":"<<targetNumber(o.identity_radius);
 if(o.has_position)s<<",\"px\":"<<targetNumber(o.position[0])<<",\"py\":"<<targetNumber(o.position[1])
  <<",\"pz\":"<<targetNumber(o.position[2])<<",\"radius_m\":"<<targetNumber(o.radius);
 if(o.has_bearing)s<<",\"ox\":"<<targetNumber(o.origin[0])<<",\"oy\":"<<targetNumber(o.origin[1])
  <<",\"oz\":"<<targetNumber(o.origin[2])<<",\"dx\":"<<targetNumber(o.direction[0])
  <<",\"dy\":"<<targetNumber(o.direction[1])<<",\"dz\":"<<targetNumber(o.direction[2])
  <<",\"half_angle_rad\":"<<targetNumber(o.half_angle)<<",\"support_captures\":"<<o.support
  <<",\"support_first_s\":"<<targetNumber(o.support_first)<<",\"camera_half_fov_rad\":"<<targetNumber(o.hfov);
 std::string co;for(auto counter:o.exclusions){if(!co.empty())co+=",";co+=std::to_string(counter);}
 s<<",\"coobserved_counters\":"<<reviewJsonString(co);
 s<<"}";return s.str();
}
inline std::string targetRecordSummaryJSON(const TargetRecord&r){auto o=r.material_reference;o.exclusions={r.exclusions.begin(),r.exclusions.end()};if(r.confirmed)o.visual="visually_confirmed";return targetObservationJSON(o,r.revision);}
inline TargetAlignment readTargetAlignment(const TargetTree&t){
 TargetAlignment a;a.id=t.get<std::string>("evidence_id");a.robot=t.get<std::string>("robot");
 a.source_version=t.get<std::string>("source_version");a.target_version=t.get<std::string>("target_version");
 a.source_clock=t.get<std::string>("source_clock");a.target_clock=t.get<std::string>("target_clock");
 a.clock_offset_s=t.get<double>("clock_offset_s");a.clock_error_s=t.get<double>("clock_error_s");
 a.position_error_m=t.get<double>("position_error_m");a.rotation_error_rad=t.get<double>("rotation_error_rad");
 a.valid_from=t.get<double>("valid_from_s");a.valid_until=t.get<double>("valid_until_s");a.translation=targetReadVec(t,"t");
 for(int i=0;i<3;++i)for(int j=0;j<3;++j)a.rotation[i*3+j]=t.get<double>("r"+std::to_string(i)+std::to_string(j));
 if(!a.valid(a.valid_from))throw std::runtime_error("invalid_alignment_contract");return a;
}
inline TargetVisibility readTargetVisibility(const TargetTree&t){
 const std::set<std::string> fields{"robot","session","counter","capture_s","anchor_capture_s","frame_version",
  "cx","cy","cz","ex","ey","ez","state","reason","scope","model_id","detected_source","checked_pixels"};
 std::set<std::string> seen;for(const auto&kv:t)if(!fields.count(kv.first)||!seen.insert(kv.first).second||!kv.second.empty())
  throw std::runtime_error("invalid_visibility_field");
 TargetVisibility v;v.key={t.get<std::string>("robot"),t.get<std::string>("session"),targetUnsigned(t,"counter")};
 v.capture=t.get<double>("capture_s");v.anchor_capture=t.get<double>("anchor_capture_s");
 v.center=targetReadVec(t,"c");v.extent=targetReadVec(t,"e");
 v.frame_version=t.get<std::string>("frame_version");v.state=t.get<std::string>("state");
 v.reason=t.get<std::string>("reason");v.scope=t.get<std::string>("scope");v.model_id=t.get<std::string>("model_id");return v;
}
inline TargetBatchResult processTargetTree(TargetRegistry&r,const TargetTree&tree,double now){
 TargetBatchResult result;
 if(tree.get<int>("schema")!=1)throw std::runtime_error("unsupported_target_schema");
 auto kind=tree.get<std::string>("kind");
 if(kind=="visibility"){
  auto items=tree.get_child_optional("regions");if(!items||items->size()>64)throw std::runtime_error("invalid_visibility_batch");
  std::vector<TargetVisibility> regions;for(const auto&item:*items){
   if(!item.first.empty())throw std::runtime_error("visibility_must_be_array");regions.push_back(readTargetVisibility(item.second));
  }
  for(const auto&v:regions)r.observeVisibility(v,now);
  result.reason="surface_region_visibility_only";
 }else if(kind=="observations"){
  auto items=tree.get_child_optional("observations");if(!items)throw std::runtime_error("missing_observations");
  const auto raw_tracks=tree.get<std::size_t>("raw_tracks_total",0);
  if(items->size()>128)throw std::runtime_error("too_many_frame_observations");
  std::vector<TargetObservation> obs;
  obs.reserve(items->size());
  for(const auto&item:*items){
   if(!item.first.empty())throw std::runtime_error("observations_must_be_array");
   obs.push_back(readTargetObservation(item.second));
  }
  result=r.ingestLocalFrame(std::move(obs),now);
  result.raw_tracks=raw_tracks;
 }else if(kind=="target_summary"){
  auto o=readTargetObservation(tree);auto u=r.ingest(o,now,true);result.peer=true;result.reason=u.reason;
  if(u.accepted){result.accepted.push_back(o.key);r.associate(result.accepted,now);}
  else r.events.push_back({"summary_ignored",u.reason,o.key,{},now});
 }else throw std::runtime_error("unsupported_target_message_kind");
 return result;
}
inline TargetBatchResult processTargetJSON(TargetRegistry&r,const std::string&text,double now){
 return processTargetTree(r,parseTargetTree(text),now);
}
inline std::string targetEventsJSON(const std::vector<TargetEvent>&events){
 std::ostringstream s;s<<"[";bool first=true;
 for(const auto&e:events){if(!first)s<<",";first=false;
  s<<"{\"kind\":"<<reviewJsonString(e.kind)<<",\"reason\":"<<reviewJsonString(e.reason)
   <<",\"source\":"<<reviewJsonString(targetKeyString(e.key))
   <<",\"other\":"<<reviewJsonString(targetKeyString(e.other))<<",\"time_s\":"<<targetNumber(e.time)<<"}";
 }s<<"]";return s.str();
}
inline std::string targetCountBoundsJSON(const TargetRegistry&r){
 auto c=targetCountBounds(r);std::ostringstream s;
 s<<"{\"scope\":\"observed_confirmed_hypotheses_conditional_on_visual_alias_and_coexistence_validity\","
  <<"\"ground_truth_verified\":false,\"unseen_people_bounded\":false,\"independent_true_people\":null,"
  <<"\"visual_confirmed_hypotheses\":"<<c.upper<<",\"distinct_lower_bound\":";
 if(c.inconsistent)s<<"null";else s<<c.lower;
 s<<",\"distinct_upper_bound\":";if(c.inconsistent)s<<"null";else s<<c.upper;
 s<<",\"conditional_distinct_count\":";if(!c.inconsistent&&c.lower==c.upper)s<<c.lower;else s<<"null";
 s<<",\"unresolved_additional_hypotheses\":";if(c.inconsistent)s<<"null";else s<<c.upper-c.lower;
 s<<",\"status\":"<<reviewJsonString(c.inconsistent?"inconsistent_constraints":c.lower==c.upper?"exact_under_evidence_assumptions":"identity_interval")
  <<",\"lower_bound_method\":\"bounded_greedy_cannot_link_clique_witness\",\"pair_checks\":"<<c.pair_checks
  <<",\"pair_budget_exhausted\":"<<(c.pair_budget_exhausted?"true":"false")<<",\"witness\":[";
 bool first=true;for(const auto&k:c.witness){if(!first)s<<",";first=false;s<<reviewJsonString(targetKeyString(k));}
 s<<"]}";return s.str();
}
inline std::string targetRegistryJSON(const TargetRegistry&r,double now,std::size_t raw_tracks=0){
 auto roots=r.roots();std::size_t confirmed=0,reliable=0,unresolved=0,source_confirmed=0;
 for(const auto&kv:r.records)source_confirmed+=kv.second.confirmed;
 for(const auto&k:roots){confirmed+=r.groupConfirmed(k);reliable+=r.currentPosition(k,now)!=nullptr;
  unresolved+=r.component(k).size()==1;}
 std::ostringstream s;s<<"{\"schema_version\":3,\"mode\":\"cpp_target_registry\",\"robot\":"
  <<reviewJsonString(r.self)<<",\"processing_time_s\":"<<targetNumber(now)
  <<",\"counts\":{\"raw_local_visual_tracks_total\":"<<raw_tracks
  <<",\"active_cache_sources\":"<<r.active_sources.size()<<",\"source_records\":"<<r.records.size()<<",\"persistent_hypotheses\":"<<roots.size()
  <<",\"visually_confirmed_source_records\":"<<source_confirmed
  <<",\"confirmed_presence_hypotheses\":"<<confirmed
  <<",\"reliable_localizations_current\":"<<reliable
  <<",\"identity_unresolved_hypotheses\":"<<unresolved<<",\"independent_true_people\":null}"
  <<",\"count_scope\":\"Local source/alias hypotheses, not verified unique true people\""
  <<",\"identity_count\":"<<targetCountBoundsJSON(r)
  <<",\"association\":{\"detections\":"<<r.stats.detections<<",\"records\":"<<r.stats.records
  <<",\"linear_scans\":"<<r.stats.scans<<",\"candidate_edges\":"<<r.stats.edges
  <<",\"ambiguous\":"<<r.stats.ambiguous<<",\"linked\":"<<r.stats.linked
  <<",\"unmatched\":"<<r.stats.unmatched<<",\"elapsed_us\":"<<targetNumber(r.stats.elapsed_us)<<"}"
  <<",\"hypotheses\":[";
 bool first=true;
 for(const auto&k:roots){if(!first)s<<",";first=false;auto g=r.component(k);const auto*p=r.currentPosition(k,now);
  const TargetRecord*recent=nullptr;for(const auto&i:g){const auto&v=r.records.at(i);if(!recent||v.last>recent->last)recent=&v;}
  std::string loc=p?"reliable":recent->latest.localization=="reliable"?"expired":recent->latest.localization;
  s<<"{\"id\":"<<reviewJsonString(targetKeyString(k))<<",\"first_discovery_s\":"<<targetNumber(r.firstDiscovery(k))
   <<",\"visual_state\":"<<reviewJsonString(r.groupConfirmed(k)?"visually_confirmed":"person_candidate")
   <<",\"semantic_state\":"<<reviewJsonString(r.groupConfirmed(k)?"confirmed":"candidate")
   <<",\"lifecycle_state\":"<<reviewJsonString(now-recent->last<=r.current_ttl?"active":"archived")
   <<",\"identity_state\":"<<reviewJsonString(g.size()>1?"associated_local_hypothesis":recent->identity=="ambiguous"?"ambiguous":"unresolved_single_source")
   <<",\"localization_state\":"<<reviewJsonString(loc)
   <<",\"position\":";
  if(p)s<<"{\"xyz\":"<<targetVector(p->latest.position)<<",\"capture_time_s\":"<<targetNumber(p->last)
   <<",\"frame\":"<<reviewJsonString(p->latest.frame)<<",\"frame_version\":"<<reviewJsonString(p->latest.frame_version)
   <<",\"source\":"<<reviewJsonString(targetKeyString(p->latest.key))
   <<",\"radius_m\":"<<targetNumber(p->latest.radius)
   <<",\"representative_point\":\"retained_visible_surface_not_body_centre\",\"fusion\":\"source_selected_not_averaged\"}";
  else s<<"null";
  s<<",\"source_ids\":[";bool f=true;for(const auto&i:g){if(!f)s<<",";f=false;s<<reviewJsonString(targetKeyString(i));}s<<"]}";
 }
 s<<"],\"source_records\":[";first=true;
 for(const auto&kv:r.records){if(!first)s<<",";first=false;
  auto current=kv.second.latest;
  if(now-current.capture>r.current_ttl){current.has_bearing=false;if(current.has_position){current.has_position=false;current.localization="expired";}}
  auto source=targetObservationJSON(current,kv.second.revision,"source_record");source.pop_back();
  s<<source<<",\"association_reason\":"<<reviewJsonString(kv.second.association_reason)
   <<",\"identity_state\":"<<reviewJsonString(kv.second.identity)
   <<",\"has_historical_anchor\":"<<(kv.second.has_anchor?"true":"false");
  if(kv.second.has_anchor)s<<",\"historical_anchor\":{\"xyz\":"<<targetVector(kv.second.anchor)
    <<",\"capture_time_s\":"<<targetNumber(kv.second.anchor_time)<<",\"radius_m\":"<<targetNumber(kv.second.anchor_radius)<<"}";
  s<<",\"region_visibility\":{\"state\":"<<reviewJsonString(kv.second.visibility)
   <<",\"capture_s\":"<<targetNumber(kv.second.visibility_capture)<<",\"visible_region_misses\":"<<kv.second.visible_region_misses
   <<",\"scope\":\"visible_surface_region_not_whole_person_absence\"}}";}
 s<<"],\"aliases\":[";first=true;
 for(const auto&e:r.aliases){if(!first)s<<",";first=false;
  s<<"{\"left\":"<<reviewJsonString(targetKeyString(e.a))<<",\"right\":"<<reviewJsonString(targetKeyString(e.b))
   <<",\"distance_m\":"<<targetNumber(e.distance)<<",\"evidence\":"<<reviewJsonString(e.surface_sequence?"registered_surface_sequence_to_anchor":"reliable_position_to_anchor")<<",\"reversible\":true}";}
 s<<"],\"events\":"<<targetEventsJSON(r.events)<<"}";return s.str();
}
}

