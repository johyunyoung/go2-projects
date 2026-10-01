// READ-ONLY: no publishers
#include "terminal.hpp"

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <cstring>

namespace term {

std::atomic<bool> g_quit{false};

namespace {

bool g_saved = false;
struct termios g_orig {};

void OnSignal(int) { g_quit = true; }

}  // namespace

void InstallSignalHandlers() {
  struct sigaction sa;
  std::memset(&sa, 0, sizeof(sa));
  sa.sa_handler = OnSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;  // SA_RESTART 없음: poll() 이 신호로 깨어나도록
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

void Restore() {
  if (g_saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig);
  }
  const char* s = "\033[0m\033[?25h";
  ssize_t r = ::write(STDOUT_FILENO, s, std::strlen(s));
  (void)r;
}

RawMode::RawMode() {
  if (!isatty(STDIN_FILENO)) return;
  if (tcgetattr(STDIN_FILENO, &g_orig) != 0) return;
  if (!g_saved) {
    g_saved = true;
    std::atexit(Restore);
  }
  struct termios raw = g_orig;
  raw.c_lflag &= ~(ICANON | ECHO);  // ISIG 는 유지 → Ctrl+C 동작
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

RawMode::~RawMode() { Restore(); }

int ReadKey(int timeout_ms) {
  struct pollfd pfd;
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN;
  pfd.revents = 0;
  const int r = ::poll(&pfd, 1, timeout_ms);
  if (r <= 0 || !(pfd.revents & POLLIN)) return -1;
  unsigned char c = 0;
  const ssize_t n = ::read(STDIN_FILENO, &c, 1);
  if (n == 0) {
    // stdin EOF (파이프 등): poll 이 즉시 반환되므로 바쁜 루프 방지
    ::usleep(static_cast<useconds_t>(timeout_ms > 0 ? timeout_ms : 1) * 1000);
    return -1;
  }
  if (n != 1) return -1;
  return c;
}

void FlushInput() {
  if (isatty(STDIN_FILENO)) tcflush(STDIN_FILENO, TCIFLUSH);
}

void Print(const std::string& s) {
  size_t off = 0;
  while (off < s.size()) {
    const ssize_t n = ::write(STDOUT_FILENO, s.data() + off, s.size() - off);
    if (n <= 0) break;
    off += static_cast<size_t>(n);
  }
}

}  // namespace term
