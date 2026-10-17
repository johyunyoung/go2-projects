#pragma once

#include <string>
#include <vector>

#include "go2_mpc/mpc_controller.hpp"
#include "go2_mpc/remote.hpp"

namespace go2 {

// Scripted operator input for repeatable tests without a remote (--script FILE).
//
//   - {t: 1.0, event: stand}     # stand | run | passive
//   - {t: 5.0, event: run}
//   - {t: 7.0, dz: -0.05}        # pose fields hold their value until changed
//   - {t: 9.0, pitch: 0.15, roll: 0.0}
//   - {t: 12.0, vx: 0.5, vy: 0.0, wz: 0.0}   # walking velocity (body frame)
//
// While a script is loaded the pose and velocity commands come from it, not from the
// sticks; remote buttons still work (e.g. 0 to stop).
class Script {
 public:
  static Script load(const std::string& path);

  // Applies every entry with t <= now that has not run yet.
  void apply(double now, RemoteEvents& events, PoseCommand& pose, VelocityCommand& vel);
  bool done() const { return next_ >= entries_.size(); }

 private:
  struct Entry {
    double t = 0.0;
    std::string event;
    bool has_dz = false, has_roll = false, has_pitch = false, has_yaw = false;
    double dz = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;
    bool has_vx = false, has_vy = false, has_wz = false;
    double vx = 0.0, vy = 0.0, wz = 0.0;
  };
  std::vector<Entry> entries_;
  size_t next_ = 0;
  PoseCommand pose_;
  VelocityCommand vel_;
};

}  // namespace go2
