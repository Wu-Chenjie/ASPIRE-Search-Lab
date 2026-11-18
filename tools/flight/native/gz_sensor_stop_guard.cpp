#include <mutex>
#include <cstdlib>
#include <dlfcn.h>
namespace {
using Stop = void (*)(void *);
std::mutex stopMutex;
std::once_flag resolution;
Stop original=nullptr;
constexpr auto stopSymbol="_ZN2gz3sim2v87systems14SensorsPrivate4StopEv";
void resolveOriginal() {
  original=reinterpret_cast<Stop>(dlsym(RTLD_NEXT,stopSymbol));
  if(!original) {
    auto handle=dlopen("/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-sensors-system.so",RTLD_NOW|RTLD_NOLOAD);
    if(handle){original=reinterpret_cast<Stop>(dlsym(handle,stopSymbol));dlclose(handle);}
  }
  if(!original)std::abort();
}
}
extern "C" void serialStop(void *) asm("_ZN2gz3sim2v87systems14SensorsPrivate4StopEv");
extern "C" void serialStop(void *self) {
  std::call_once(resolution,resolveOriginal);
  if(reinterpret_cast<void *>(original)==reinterpret_cast<void *>(&serialStop))std::abort();
  std::lock_guard<std::mutex> lock(stopMutex);
  original(self);
}
