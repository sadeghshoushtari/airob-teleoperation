// Test stub of the VIRTUOSE API (not the Haption library) for MasterTeleImpedance/master_ti.cpp in simulation.
// The device pose is read every 20 ms from the file named by TI_STUB_POSE: "x y z qx qy qz qw" (m, unit
// quaternion); without the file the device rests at the origin. The dead-man switch reads released for the first
// second, then pressed (the slave needs to see a release first); emergency stop, power and alarm read healthy.
// The forces the master sends are appended at 100 Hz to the file named by TI_STUB_FORCE_LOG (if set):
// "t fx fy fz mx my mz".
// With TI_STUB_HAND = "m b k mr br kr" (kg, Ns/m, N/m, kg m^2, Nms/rad, Nm/rad) the device and the operator's hand
// are simulated instead of read: a body of mass m (rotational inertia mr) held by a hand spring k (kr) with damping
// b (br) around the pose in TI_STUB_POSE (now the hand's intended pose) and pushed by the forces the master sends,
// integrated at every virtGetPosition call (the master's 1 kHz loop); virtGetSpeed returns its velocity (world
// frame).
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "virtuoseAPI.h"

namespace {
using Clock = std::chrono::steady_clock;
Clock::time_point g_open_time;
Clock::time_point g_last_read;
Clock::time_point g_last_force_log;
float g_pose[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
int g_context = 1;

struct Hand {
  bool enabled = false;
  double m = 0, b = 0, k = 0, mr = 0, br = 0, kr = 0;
  bool started = false;
  Clock::time_point last;
  Eigen::Vector3d x = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero(), w = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  double force[6] = {0, 0, 0, 0, 0, 0};
} g_hand;

Eigen::Vector3d rotation_vector(const Eigen::Quaterniond& rotation) {
  Eigen::Quaterniond r = rotation.normalized();
  if (r.w() < 0.0) r.coeffs() = -r.coeffs();
  const Eigen::AngleAxisd aa(r);
  return aa.angle() * aa.axis();
}

// One integration step of the simulated device and hand towards the intended pose g_pose.
void step_hand() {
  const Clock::time_point now = Clock::now();
  const Eigen::Vector3d xi(g_pose[0], g_pose[1], g_pose[2]);
  const Eigen::Quaterniond qi(g_pose[6], g_pose[3], g_pose[4], g_pose[5]);
  if (!g_hand.started) {
    g_hand.started = true;
    g_hand.last = now;
    g_hand.x = xi;
    g_hand.q = qi.normalized();
    return;
  }
  const double dt = std::clamp(std::chrono::duration<double>(now - g_hand.last).count(), 0.0, 0.005);
  g_hand.last = now;
  const Eigen::Vector3d f(g_hand.force[0], g_hand.force[1], g_hand.force[2]);
  const Eigen::Vector3d tau(g_hand.force[3], g_hand.force[4], g_hand.force[5]);
  g_hand.v += dt * (f + g_hand.k * (xi - g_hand.x) - g_hand.b * g_hand.v) / g_hand.m;
  g_hand.x += dt * g_hand.v;
  g_hand.w += dt * (tau + g_hand.kr * rotation_vector(qi * g_hand.q.inverse()) - g_hand.br * g_hand.w) / g_hand.mr;
  const Eigen::Vector3d turn = dt * g_hand.w;
  if (turn.norm() > 1e-15) g_hand.q = Eigen::Quaterniond(Eigen::AngleAxisd(turn.norm(), turn.normalized())) * g_hand.q;
  g_hand.q.normalize();
}

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
  if (const char* hand = std::getenv("TI_STUB_HAND")) {
    std::istringstream in(hand);
    g_hand.enabled = static_cast<bool>(in >> g_hand.m >> g_hand.b >> g_hand.k >> g_hand.mr >> g_hand.br >> g_hand.kr);
  }
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
  if (g_hand.enabled) {
    step_hand();
    const float device[7] = {static_cast<float>(g_hand.x.x()), static_cast<float>(g_hand.x.y()),
                             static_cast<float>(g_hand.x.z()), static_cast<float>(g_hand.q.x()),
                             static_cast<float>(g_hand.q.y()), static_cast<float>(g_hand.q.z()),
                             static_cast<float>(g_hand.q.w())};
    for (int i = 0; i < 7; ++i) pos[i] = device[i];
    return 0;
  }
  for (int i = 0; i < 7; ++i) pos[i] = g_pose[i];
  return 0;
}
int virtGetPowerOn(VirtContext, int* power) { *power = 1; return 0; }
int virtGetSpeed(VirtContext, float* speed) {
  for (int i = 0; i < 3; ++i) {
    speed[i] = g_hand.enabled ? static_cast<float>(g_hand.v[i]) : 0.0f;
    speed[3 + i] = g_hand.enabled ? static_cast<float>(g_hand.w[i]) : 0.0f;
  }
  return 0;
}
int virtIsInShiftPosition(VirtContext, int* shift) { *shift = 0; return 0; }
int virtSaturateTorque(VirtContext, float, float) { return 0; }
int virtSetBaseFrame(VirtContext, float*) { return 0; }
int virtSetCommandType(VirtContext, VirtCommandType) { return 0; }
int virtSetForce(VirtContext, float* force) {
  for (int i = 0; i < 6; ++i) g_hand.force[i] = force[i];
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
