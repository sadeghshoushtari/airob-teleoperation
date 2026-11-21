// Test stub of the VIRTUOSE API (not the Haption library) for MasterTeleImpedance/master_ti.cpp in simulation.
// The device pose is read every 20 ms from the file named by TI_STUB_POSE: "x y z qx qy qz qw" (m, unit
// quaternion); without the file the device rests at the origin. The dead-man switch reads released for the first
// second, then pressed (the slave needs to see a release first); emergency stop, power and alarm read healthy.
// The forces the master sends are appended at 100 Hz to the file named by TI_STUB_FORCE_LOG (if set):
// "t fx fy fz mx my mz".
#include <chrono>
#include <cstdlib>
#include <fstream>

#include "virtuoseAPI.h"

namespace {
using Clock = std::chrono::steady_clock;
Clock::time_point g_open_time;
Clock::time_point g_last_read;
Clock::time_point g_last_force_log;
float g_pose[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
int g_context = 1;

double since_open() { return std::chrono::duration<double>(Clock::now() - g_open_time).count(); }

void read_pose() {
  const char* path = std::getenv("TI_STUB_POSE");
  if (!path || Clock::now() - g_last_read < std::chrono::milliseconds(20)) return;
  g_last_read = Clock::now();
  std::ifstream file(path);
  float p[7];
  for (float& v : p)
    if (!(file >> v)) return;
  for (int i = 0; i < 7; ++i) g_pose[i] = p[i];
}
}  // namespace

extern "C" {
VirtContext virtOpen(const char*) {
  g_open_time = Clock::now();
  g_last_read = g_last_force_log = Clock::time_point{};
  return &g_context;
}
int virtClose(VirtContext) { return 0; }
int virtEnableForceFeedback(VirtContext, int) { return 0; }
int virtGetAlarm(VirtContext, unsigned int* alarm) { *alarm = 0; return 0; }
int virtGetAnalogicInputs(VirtContext, float* inputs) { for (int i = 0; i < 7; ++i) inputs[i] = 0.0f; return 0; }
int virtGetDeadMan(VirtContext, int* dead_man) { *dead_man = since_open() > 1.0 ? 1 : 0; return 0; }
int virtGetEmergencyStop(VirtContext, int* emergency_stop) { *emergency_stop = 1; return 0; }
int virtGetPosition(VirtContext, float* pos) {
  read_pose();
  for (int i = 0; i < 7; ++i) pos[i] = g_pose[i];
  return 0;
}
int virtGetPowerOn(VirtContext, int* power) { *power = 1; return 0; }
int virtGetSpeed(VirtContext, float* speed) { for (int i = 0; i < 6; ++i) speed[i] = 0.0f; return 0; }
int virtIsInShiftPosition(VirtContext, int* shift) { *shift = 0; return 0; }
int virtSaturateTorque(VirtContext, float, float) { return 0; }
int virtSetBaseFrame(VirtContext, float*) { return 0; }
int virtSetCommandType(VirtContext, VirtCommandType) { return 0; }
int virtSetForce(VirtContext, float* force) {
  const char* path = std::getenv("TI_STUB_FORCE_LOG");
  if (path && Clock::now() - g_last_force_log >= std::chrono::milliseconds(10)) {
    g_last_force_log = Clock::now();
    std::ofstream file(path, std::ios::app);
    file << since_open();
    for (int i = 0; i < 6; ++i) file << ' ' << force[i];
    file << '\n';
  }
  return 0;
}
int virtSetForceFactor(VirtContext, float) { return 0; }
int virtSetIndexingMode(VirtContext, VirtIndexingType) { return 0; }
int virtSetObservationFrame(VirtContext, float*) { return 0; }
int virtSetPowerOn(VirtContext, int) { return 0; }
int virtSetSpeedFactor(VirtContext, float) { return 0; }
int virtSetTimeStep(VirtContext, float) { return 0; }
int virtSetTimeoutValue(VirtContext, float) { return 0; }
}
