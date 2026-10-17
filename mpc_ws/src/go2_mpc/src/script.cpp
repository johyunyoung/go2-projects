#include "go2_mpc/script.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace go2 {

Script Script::load(const std::string& path) {
  Script s;
  const YAML::Node root = YAML::LoadFile(path);
  if (!root.IsSequence()) throw std::runtime_error("script: expected a list of entries");
  for (const auto& n : root) {
    Entry e;
    e.t = n["t"].as<double>();
    if (n["event"]) {
      e.event = n["event"].as<std::string>();
      if (e.event != "stand" && e.event != "run" && e.event != "passive") {
        throw std::runtime_error("script: unknown event '" + e.event + "'");
      }
    }
    if (n["dz"]) { e.has_dz = true; e.dz = n["dz"].as<double>(); }
    if (n["roll"]) { e.has_roll = true; e.roll = n["roll"].as<double>(); }
    if (n["pitch"]) { e.has_pitch = true; e.pitch = n["pitch"].as<double>(); }
    if (n["yaw"]) { e.has_yaw = true; e.yaw = n["yaw"].as<double>(); }
    if (n["vx"]) { e.has_vx = true; e.vx = n["vx"].as<double>(); }
    if (n["vy"]) { e.has_vy = true; e.vy = n["vy"].as<double>(); }
    if (n["wz"]) { e.has_wz = true; e.wz = n["wz"].as<double>(); }
    s.entries_.push_back(e);
  }
  std::stable_sort(s.entries_.begin(), s.entries_.end(),
                   [](const Entry& a, const Entry& b) { return a.t < b.t; });
  return s;
}

void Script::apply(double now, RemoteEvents& events, PoseCommand& pose, VelocityCommand& vel) {
  while (next_ < entries_.size() && entries_[next_].t <= now) {
    const Entry& e = entries_[next_++];
    if (e.event == "stand") events.stand = true;
    if (e.event == "run") events.run = true;
    if (e.event == "passive") events.passive = true;
    if (e.has_dz) pose_.dz = e.dz;
    if (e.has_roll) pose_.roll = e.roll;
    if (e.has_pitch) pose_.pitch = e.pitch;
    if (e.has_yaw) pose_.yaw = e.yaw;
    if (e.has_vx) vel_.vx = e.vx;
    if (e.has_vy) vel_.vy = e.vy;
    if (e.has_wz) vel_.wz = e.wz;
    std::printf("[script] t=%.2f %s | pose dz %+.3f roll %+.3f pitch %+.3f yaw %+.3f | "
                "vel vx %+.2f vy %+.2f wz %+.2f\n",
                e.t, e.event.empty() ? "-" : e.event.c_str(), pose_.dz, pose_.roll, pose_.pitch,
                pose_.yaw, vel_.vx, vel_.vy, vel_.wz);
  }
  pose = pose_;
  vel = vel_;
}

}  // namespace go2
