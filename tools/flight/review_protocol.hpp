#pragma once
#include <cctype>
#include <cstdint>
#include <sstream>
#include <string>

namespace racer_task {
inline std::string reviewJsonString(const std::string& value) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:value) {
    if(c=='"' || c=='\\')out<<'\\'<<c;
    else if(c<32) {const char* hex="0123456789abcdef";out<<"\\u00"<<hex[c>>4]<<hex[c&15];}
    else out<<c;
  }
  out<<'"';return out.str();
}
// Deliberately small flat JSON wire contract: exactly two string members,
// in either order. No permissive substring extraction or floating point IDs.
inline bool parseReviewCancel(const std::string& text,std::int64_t& id,std::string& reason) {
  if(text.size()>1024)return false;
  size_t at=0;auto ws=[&](){while(at<text.size() && std::isspace(static_cast<unsigned char>(text[at])))++at;};
  auto take=[&](char c){ws();if(at==text.size() || text[at]!=c)return false;++at;return true;};
  auto string=[&](std::string& out) {
    if(!take('"'))return false;
    out.clear();
    while(at<text.size()) {
      unsigned char c=text[at++];if(c=='"')return true;if(c<32)return false;
      if(c=='\\') {if(at==text.size())return false;c=text[at++];
        if(c=='n')c='\n';else if(c=='r')c='\r';else if(c=='t')c='\t';
        else if(c!='"' && c!='\\' && c!='/')return false;
      }
      out.push_back(c);
    }
    return false;
  };
  if(!take('{'))return false;
  std::string id_text;bool have_id=false,have_reason=false;
  for(int i=0;i<2;++i) {
    std::string key,value;
    if(!string(key) || !take(':') || !string(value))return false;
    if(key=="request_id" && !have_id){id_text=value;have_id=true;}
    else if(key=="reason" && !have_reason){reason=value;have_reason=true;}
    else return false;
    if(i==0 && !take(','))return false;
  }
  if(!take('}'))return false;
  ws();
  if(at!=text.size() || !have_id || !have_reason || id_text.empty() || id_text.size()>19 || reason.empty() || reason.size()>160)return false;
  for(unsigned char c:id_text)if(!std::isdigit(c))return false;
  try {size_t used=0;id=std::stoll(id_text,&used);return used==id_text.size() && id>0;}catch(...){return false;}
}
} // namespace racer_task
