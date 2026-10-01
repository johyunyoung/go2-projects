// READ-ONLY: no publishers
//
// 터미널 입출력 유틸: raw 키 입력(비차단), ANSI 색/화면 제어, 종료 신호 처리.
#pragma once

#include <atomic>
#include <string>

namespace term {

// Ctrl+C / SIGTERM 수신 시 true
extern std::atomic<bool> g_quit;

void InstallSignalHandlers();

// 생성 시 stdin 을 비정규(non-canonical)/에코 없음 모드로, 소멸 시 원복.
// 비정상 종료에 대비해 atexit 으로도 원복한다.
class RawMode {
 public:
  RawMode();
  ~RawMode();
  RawMode(const RawMode&) = delete;
  RawMode& operator=(const RawMode&) = delete;
};

void Restore();  // 터미널 설정 원복 + 커서 표시

// timeout_ms 동안 키 입력 대기. 입력 없으면 -1.
int ReadKey(int timeout_ms);

// 버퍼에 쌓인 키 입력 버리기
void FlushInput();

void Print(const std::string& s);

// ANSI
constexpr const char* kReset = "\033[0m";
constexpr const char* kBold = "\033[1m";
constexpr const char* kReverse = "\033[7m";
constexpr const char* kRed = "\033[31m";
constexpr const char* kGreen = "\033[32m";
constexpr const char* kYellow = "\033[33m";
constexpr const char* kCyan = "\033[36m";
constexpr const char* kBoldRed = "\033[1;31m";
constexpr const char* kHome = "\033[H";
constexpr const char* kClearScreen = "\033[2J";
constexpr const char* kClearEol = "\033[K";
constexpr const char* kClearBelow = "\033[J";
constexpr const char* kHideCursor = "\033[?25l";
constexpr const char* kShowCursor = "\033[?25h";

}  // namespace term
