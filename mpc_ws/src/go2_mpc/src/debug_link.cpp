#include "go2_mpc/debug_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

namespace go2 {

DebugLink::DebugLink(const std::string& host, int port) {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) throw std::runtime_error("DebugLink: socket() failed");
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);

  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1) {
    ::close(fd_);
    throw std::runtime_error("DebugLink: invalid IPv4 address '" + host + "'");
  }
}

DebugLink::~DebugLink() {
  if (fd_ >= 0) ::close(fd_);
}

void DebugLink::send(const DebugPacket& packet) {
  // Errors (no listener, full buffer) are ignored on purpose: debug output must
  // never disturb control.
  (void)::sendto(fd_, &packet, sizeof(packet), MSG_DONTWAIT,
                 reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_));
}

}  // namespace go2
