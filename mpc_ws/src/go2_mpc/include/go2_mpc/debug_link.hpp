#pragma once

#include <string>

#include <netinet/in.h>

#include "go2_mpc/debug_packet.hpp"

namespace go2 {

// Fire-and-forget UDP sender for DebugPacket. Never blocks the control loop:
// the socket is non-blocking and a full send buffer simply drops the packet.
class DebugLink {
 public:
  DebugLink(const std::string& host, int port);
  ~DebugLink();
  DebugLink(const DebugLink&) = delete;
  DebugLink& operator=(const DebugLink&) = delete;

  void send(const DebugPacket& packet);

 private:
  int fd_ = -1;
  sockaddr_in addr_{};
};

}  // namespace go2
