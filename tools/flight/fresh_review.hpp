#pragma once
#include "target_registry.hpp"
#if __has_include("task_bearing_bias.hpp")
#include "task_bearing_bias.hpp"
#else
#include "bearing_bias.hpp"
#endif

namespace racer_task {
inline constexpr double fresh_review_age_s=.75;

inline bool freshReviewCapture(double stamp,double now){
 return std::isfinite(now)&&std::isfinite(stamp)&&stamp>=0.&&now>=stamp&&now-stamp<=fresh_review_age_s;
}

inline const TargetRecord* reviewSource(const TargetRegistry& registry,const TargetObservation& original){
 const auto found=registry.records.find(original.key);
 if(found==registry.records.end()||found->second.confirmed||original.key.robot!=registry.self)return nullptr;
 if(!registry.compatibleFrames(found->second.latest,original))return nullptr;
 return &found->second;
}

// A completed paired capture can support a fresh bearing even if its surface
// is unsuitable for localization. Never change source key or coordinate epoch.
inline const TargetObservation* freshReviewFrame(const TargetRegistry& registry,
    const TargetObservation& original,double now,double minimum_capture=-1.){
 const auto* record=reviewSource(registry,original);
 if(!record)return nullptr;
 for(auto it=record->history.rbegin();it!=record->history.rend();++it){
  if(it->capture>now)continue;
  if(it->capture<original.capture||it->capture<minimum_capture||!freshReviewCapture(it->capture,now))break;
  if(registry.compatibleFrames(*it,original)&&it->identity_input=="completed"&&it->has_bearing)return &*it;
 }
 return nullptr;
}

// Admission qualifies a source for a bounded review. The newer paired frame
// attests to current geometry and remains an independent observation.
inline const TargetObservation* reviewNavigationFrame(const TargetRegistry& registry,
    const TargetObservation& admitted,const TargetObservation& paired,double now,
    double review_deadline,bool allow_weak){
 if(paired.key!=admitted.key||!registry.compatibleFrames(paired,admitted)||
    paired.identity_input!="completed"||!paired.has_bearing||paired.capture<admitted.capture||
    !freshReviewCapture(paired.capture,now))return nullptr;
 auto supported=[allow_weak](const TargetObservation& frame){
  return frame.has_bearing&&(bearingNavigationSupported(frame.confidence,frame.support,frame.support_first,frame.capture)||
     (allow_weak&&bearingWeakSupported(frame.confidence,frame.support,frame.support_first,frame.capture)));
 };
 if(supported(paired))return &paired;
 if(now<=review_deadline&&supported(admitted))return &admitted;
 return nullptr;
}
}
