// READ-ONLY: no publishers
//
// 모드 공용 설정/도우미.
#pragma once

#include "hypothesis.hpp"
#include "state_cache.hpp"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

struct AppConfig {
  int domain_id = 0;
  std::string iface;
  std::string output_path = "go2_calibration.yaml";
  Clock::time_point start_time;
};

int RunMonitor(const AppConfig& cfg, StateCache& cache);
int RunGuided(const AppConfig& cfg, StateCache& cache);

// lowstate 가 이 시간 이상 안 오면 연결 점검 안내
constexpr double kLinkTimeoutSec = 2.0;

// printf 스타일 문자열 포맷 (format 속성으로 인자 형식 컴파일 검사)
inline std::string Fmt(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
inline std::string Fmt(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return std::string(buf);
}

// lowstate 수신 문제 여부 (시작 후 2초 이상 미수신 포함)
inline bool LowStateStale(const Snapshot& s, const AppConfig& cfg) {
  if (!s.has_low) {
    return std::chrono::duration<double>(s.now - cfg.start_time).count() >= kLinkTimeoutSec;
  }
  return s.LowAge() >= kLinkTimeoutSec;
}

inline std::vector<std::string> LinkHintLines(const AppConfig& cfg, const Snapshot& s) {
  std::vector<std::string> v;
  if (!s.has_low) {
    v.push_back(Fmt("!!! rt/lowstate 를 한 번도 받지 못했습니다 (iface=%s, domain=%d) !!!",
                    cfg.iface.c_str(), cfg.domain_id));
  } else {
    v.push_back(Fmt("!!! rt/lowstate 수신이 %.1f 초째 끊겼습니다 !!!", s.LowAge()));
  }
  v.push_back("  점검: 1) 네트워크 인터페이스 이름 (`ip a` 로 확인)");
  v.push_back("        2) DDS 도메인 ID (Go2 기본값 0)");
  v.push_back("        3) 랜선 연결 상태");
  v.push_back("        4) 이 PC 의 IP 가 192.168.123.x/24 대역인지 (`ip a` 의 inet 항목)");
  return v;
}

inline std::string Bits16(uint16_t v) {
  std::string s;
  for (int b = 15; b >= 0; --b) {
    s.push_back((v >> b) & 1 ? '1' : '0');
    if (b % 4 == 0 && b != 0) s.push_back('_');
  }
  return s;
}

// lowstate.wireless_remote 원시 바이트에서 keys 추출 (참고 가설 레이아웃)
inline uint16_t RawRemoteKeys(const LowState& low) {
  const auto& raw = low.wireless_remote();
  return static_cast<uint16_t>(raw[hyp::kRawRemoteKeyByteLo] |
                               (static_cast<uint16_t>(raw[hyp::kRawRemoteKeyByteHi]) << 8));
}
