#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
int main(){uint32_t state=123;unsigned tested=0,bad=0,oldbad=0;char text[64];for(unsigned i=0;i<100020;++i){uint32_t bits; if(i<12){const uint32_t edge[]={0,0x80000000u,1,0x80000001u,0x007fffffu,0x00800000u,0x7f7fffffu,0xff7fffffu,0x3f800000u,0xbf800000u,0x3eaaaaabu,0x41200000u};bits=edge[i];}else{state=1664525u*state+1013904223u;bits=state;}if((bits&0x7f800000u)==0x7f800000u)continue;float value,back;uint32_t out;std::memcpy(&value,&bits,4);std::snprintf(text,sizeof text,"%.9g",(double)value);back=std::strtof(text,nullptr);std::memcpy(&out,&back,4);bad+=(bits!=out);std::snprintf(text,sizeof text,"%.5f",(double)value);back=std::strtof(text,nullptr);std::memcpy(&out,&back,4);oldbad+=(bits!=out);++tested;}std::printf("%u %u %u%c",tested,bad,oldbad,10);return bad?2:0;}
