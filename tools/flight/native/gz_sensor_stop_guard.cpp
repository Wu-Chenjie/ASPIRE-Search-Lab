#include <cstdlib>
#include <dlfcn.h>
#include <mutex>

namespace {
using Stop = void (*)(void *);
constexpr auto sensorSymbol = "_ZN2gz3sim2v87systems14SensorsPrivate4StopEv";
constexpr auto runnerSymbol = "_ZN2gz3sim2v816SimulationRunner4StopEv";

Stop resolve(const char *symbol, const char *library, Stop wrapper) {
  auto original = reinterpret_cast<Stop>(dlsym(RTLD_NEXT, symbol));
  if (!original) {
    auto handle = dlopen(library, RTLD_NOW | RTLD_NOLOAD);
    if (handle) {
      original = reinterpret_cast<Stop>(dlsym(handle, symbol));
      dlclose(handle);
    }
  }
  if (!original || original == wrapper) std::abort();
  return original;
}
}

extern "C" void sensorStop(void *) asm("_ZN2gz3sim2v87systems14SensorsPrivate4StopEv");
extern "C" void sensorStop(void *self) {
  static const auto original = resolve(sensorSymbol,
      "/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-sensors-system.so", sensorStop);
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  original(self);
}

// The server-control worker and ServerPrivate destructor both emit the Stop
// event. Serialize the entire dispatch, including callbacks before Sensors.
extern "C" void runnerStop(void *) asm("_ZN2gz3sim2v816SimulationRunner4StopEv");
extern "C" void runnerStop(void *self) {
  static const auto original = resolve(runnerSymbol, "libgz-sim8.so.8", runnerStop);
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  original(self);
}
