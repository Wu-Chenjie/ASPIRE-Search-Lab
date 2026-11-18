#pragma once
// Optional fixed-size, pre-faulted journal. No JSON, allocation, disk syscall or
// flush on the callback path. Offline evidence only; never read by decisions.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif
namespace racer_task {
class StaticCausalTrace {
 std::uint64_t* data_=nullptr;std::mutex mutex_;
public:
 static constexpr std::size_t capacity=8192,header_words=16,row_words=10,bytes=(header_words+capacity*row_words)*8;
 explicit StaticCausalTrace(const char*path){
  if(!path||!*path)return;
#if defined(__unix__) || defined(__APPLE__)
  int fd=::open(path,O_RDWR|O_CREAT|O_EXCL,0600);if(fd<0)throw std::runtime_error("causal_journal_exclusive_open_failed");
  if(ftruncate(fd,bytes)){close(fd);throw std::runtime_error("causal_journal_size_failed");}
  void*m=mmap(nullptr,bytes,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);close(fd);
  if(m==MAP_FAILED)throw std::runtime_error("causal_journal_map_failed");
  data_=static_cast<std::uint64_t*>(m);std::memset(data_,0,bytes); // startup, before subscriptions
  data_[0]=0x4153504952454331ULL;data_[1]=1;data_[2]=capacity;data_[5]=row_words;data_[6]=header_words;
#else
  throw std::runtime_error("causal_journal_platform_unsupported");
#endif
 }
 ~StaticCausalTrace(){
#if defined(__unix__) || defined(__APPLE__)
  if(data_){msync(data_,bytes,MS_SYNC);munmap(data_,bytes);}
#endif
 }
 bool enabled()const{return data_!=nullptr;}
 static std::uint64_t stamp(double seconds){return std::isfinite(seconds)&&seconds>=0?std::llround(seconds*1e9):0;}
 void record(std::uint64_t kind,double sim,std::uint64_t callback=0,std::uint64_t source=0,std::uint64_t sequence=0,double capture=0,std::uint64_t a=0,std::uint64_t b=0){
  if(!data_)return;
  const auto mono=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  std::lock_guard<std::mutex> guard(mutex_);const auto n=data_[3];
  if(n>=capacity){++data_[4];return;}
  const std::array<std::uint64_t,row_words> row{n+1,stamp(sim),std::uint64_t(mono),kind,callback,source,sequence,stamp(capture),a,b};
  std::memcpy(data_+header_words+n*row_words,row.data(),sizeof(row));
  __atomic_store_n(data_+3,n+1,__ATOMIC_RELEASE);
 }
};
inline StaticCausalTrace&staticCausalTrace(){static StaticCausalTrace trace(std::getenv("ASPIRE_NATIVE_CAUSAL_JOURNAL"));return trace;}
}
