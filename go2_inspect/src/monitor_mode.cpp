// READ-ONLY: no publishers
//
// 실시간 모니터 모드: 상태를 약 10Hz 로 화면에 덮어써 표시한다.
#include "app.hpp"
#include "hypothesis.hpp"
#include "imu_math.hpp"
#include "terminal.hpp"

#include <cmath>
#include <deque>
#include <string>

namespace {

constexpr int kRenderPeriodMs = 100;       // 화면 갱신 ~10Hz
constexpr int kPollMs = 5;                 // 키 입력 / 버튼 에지 확인 주기
constexpr double kHighlightMinRad = 0.01;  // 이보다 작은 변화는 노이즈로 보고 강조하지 않음
constexpr size_t kLogLines = 8;

struct MonitorState {
  bool has_base = false;
  std::array<float, hyp::kNumMotorSlots> base_q{};

  uint64_t last_remote_count = 0;
  bool prev_keys_valid = false;
  uint16_t prev_keys = 0;

  uint64_t last_low_count = 0;
  bool prev_raw_valid = false;
  uint16_t prev_raw_keys = 0;

  std::deque<std::string> log;
};

double SinceStart(const AppConfig& cfg, const Snapshot& s) {
  return std::chrono::duration<double>(s.now - cfg.start_time).count();
}

void PushLog(MonitorState& m, const std::string& l) {
  m.log.push_back(l);
  while (m.log.size() > kLogLines) m.log.pop_front();
}

void SetBaseline(MonitorState& m, const Snapshot& s) {
  for (int i = 0; i < hyp::kNumMotorSlots; ++i) m.base_q[i] = s.low.motor_state()[i].q();
  m.has_base = true;
}

void LogRising(MonitorState& m, double t, uint16_t rising, const char* src) {
  for (int b = 0; b < 16; ++b) {
    if (rising & (1u << b)) {
      PushLog(m, Fmt("[%8.2fs] %s bit %2d : 0->1   (참고 가설: %s)", t, src, b,
                     hyp::ButtonNameForBit(b)));
    }
  }
}

// 버튼 0→1 에지 검출 (렌더 주기보다 촘촘하게 호출)
void CheckEdges(MonitorState& m, const Snapshot& s, const AppConfig& cfg) {
  const double t = SinceStart(cfg, s);
  if (s.has_remote && s.remote_count != m.last_remote_count) {
    const uint16_t k = s.remote.keys();
    if (m.prev_keys_valid) LogRising(m, t, static_cast<uint16_t>(k & ~m.prev_keys), "keys");
    m.prev_keys = k;
    m.prev_keys_valid = true;
    m.last_remote_count = s.remote_count;
  }
  // rt/wirelesscontroller 가 안 들어올 때만 lowstate 원시 바이트로 대체 기록
  const bool remote_alive = s.has_remote && s.RemoteAge() < kLinkTimeoutSec;
  if (s.has_low && s.low_count != m.last_low_count) {
    const uint16_t k = RawRemoteKeys(s.low);
    if (m.prev_raw_valid && !remote_alive)
      LogRising(m, t, static_cast<uint16_t>(k & ~m.prev_raw_keys), "raw[2..3]");
    m.prev_raw_keys = k;
    m.prev_raw_valid = true;
    m.last_low_count = s.low_count;
  }
}

std::string SetBitsList(uint16_t v) {
  std::string out;
  for (int b = 0; b < 16; ++b) {
    if (v & (1u << b)) {
      if (!out.empty()) out += ", ";
      out += std::to_string(b);
    }
  }
  return out.empty() ? "-" : out;
}

std::string GravityLine(const char* label, const Vec3& g) {
  return Fmt("  %-22s (%+.3f, %+.3f, %+.3f)   (0,0,-1)과의 각도 %5.1f deg", label, g.x, g.y, g.z,
             AngleDeg(g, DownVector()));
}

std::string Render(const MonitorState& m, const Snapshot& s, const AppConfig& cfg) {
  using namespace term;
  std::string out = kHome;
  auto line = [&out](const std::string& l) {
    out += l;
    out += kClearEol;
    out += "\n";
  };

  line(Fmt("%sgo2_inspect [monitor]  READ-ONLY (no publishers)%s   iface=%s domain=%d   "
           "r: 기준값 재설정   q: 종료",
           kBold, kReset, cfg.iface.c_str(), cfg.domain_id));
  line("");

  // ---- 수신 상태 ----
  if (s.has_low) {
    line(Fmt("rt/lowstate           : %6.1f Hz | 마지막 수신 %8.1f ms 전 | tick %10u | 누적 %llu",
             s.low_hz, s.LowAge() * 1000.0, s.low.tick(),
             static_cast<unsigned long long>(s.low_count)));
  } else {
    line("rt/lowstate           : 수신 없음");
  }
  if (s.has_remote) {
    line(Fmt("rt/wirelesscontroller : %6.1f Hz | 마지막 수신 %8.1f ms 전 | 누적 %llu", s.remote_hz,
             s.RemoteAge() * 1000.0, static_cast<unsigned long long>(s.remote_count)));
  } else {
    line("rt/wirelesscontroller : 수신 없음");
  }
  if (LowStateStale(s, cfg)) {
    for (const auto& l : LinkHintLines(cfg, s)) line(std::string(kBoldRed) + l + kReset);
  }
  line("");

  if (!s.has_low) {
    out += kClearBelow;
    return out;
  }

  // ---- 모터 0~11 ----
  const auto& ms = s.low.motor_state();
  int max_i = -1;
  double max_abs = 0.0;
  for (int i = 0; i < hyp::kNumJoints; ++i) {
    const double d = m.has_base ? ms[i].q() - m.base_q[i] : 0.0;
    if (std::fabs(d) > max_abs) {
      max_abs = std::fabs(d);
      max_i = i;
    }
  }
  const bool highlight = max_i >= 0 && max_abs >= kHighlightMinRad;

  line(std::string(kCyan) +
       "idx  name*        q[rad]    q[deg]  dq[rad/s]  tau_est    dQ[rad]   dQ[deg]   "
       "(dQ = 기준값 대비)" + kReset);
  for (int i = 0; i < hyp::kNumJoints; ++i) {
    const double q = ms[i].q();
    const double d = m.has_base ? q - m.base_q[i] : 0.0;
    std::string row = Fmt("%3d  %-10s %+9.4f %+9.2f %+9.3f %+9.3f  %+9.4f %+9.2f", i,
                          hyp::kJointName[i], q, q * kRad2Deg, ms[i].dq(), ms[i].tau_est(), d,
                          d * kRad2Deg);
    if (highlight && i == max_i) row = std::string(kReverse) + row + "  <== 최대 변화" + kReset;
    line(row);
  }
  if (highlight) {
    line(Fmt("%s가장 많이 움직인 관절: #%d (%s*)  dQ = %+.4f rad (%+.2f deg)%s", kYellow, max_i,
             hyp::kJointName[max_i], ms[max_i].q() - m.base_q[max_i],
             (ms[max_i].q() - m.base_q[max_i]) * kRad2Deg, kReset));
  } else {
    line(Fmt("가장 많이 움직인 관절: 없음 (모두 %.2f rad 미만)", kHighlightMinRad));
  }
  line("  * 관절 이름은 가설 (미검증) — 안내형 모드(--mode guided)로 확인하세요");

  // ---- 모터 12~19 ----
  std::vector<std::string> extra;
  for (int i = hyp::kNumJoints; i < hyp::kNumMotorSlots; ++i) {
    const auto& mm = ms[i];
    if (mm.q() != 0.0f || mm.dq() != 0.0f || mm.tau_est() != 0.0f || mm.mode() != 0 ||
        mm.temperature() != 0) {
      extra.push_back(Fmt("  #%d mode=%u q=%+.4f dq=%+.3f tau=%+.3f temp=%u", i, mm.mode(), mm.q(),
                          mm.dq(), mm.tau_est(), mm.temperature()));
    }
  }
  if (extra.empty()) {
    line("모터 12~19 (Go2 미사용 칸으로 추정): 모두 0");
  } else {
    line(std::string(kYellow) + "모터 12~19 (Go2 미사용 칸으로 추정) 중 값이 있는 칸:" + kReset);
    for (const auto& l : extra) line(l);
  }
  line("");

  // ---- IMU ----
  const auto& imu = s.low.imu_state();
  const auto& q = imu.quaternion();
  const auto& rpy = imu.rpy();
  const auto& gy = imu.gyroscope();
  const auto& ac = imu.accelerometer();
  const double qn = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  const double an = std::sqrt(ac[0] * ac[0] + ac[1] * ac[1] + ac[2] * ac[2]);
  line(std::string(kCyan) + "IMU" + kReset);
  line(Fmt("  quaternion 원시 [0..3]: [%+.5f, %+.5f, %+.5f, %+.5f]   |q| = %.4f", q[0], q[1], q[2],
           q[3], qn));
  line(Fmt("  rpy [rad]   : (%+.4f, %+.4f, %+.4f)   [deg] (%+.2f, %+.2f, %+.2f)", rpy[0], rpy[1],
           rpy[2], rpy[0] * kRad2Deg, rpy[1] * kRad2Deg, rpy[2] * kRad2Deg));
  line(Fmt("  gyroscope   : (%+.4f, %+.4f, %+.4f) rad/s", gy[0], gy[1], gy[2]));
  line(Fmt("  accelerometer: (%+.4f, %+.4f, %+.4f) m/s^2   |acc| = %.3f", ac[0], ac[1], ac[2], an));
  const Vec3 ga = GravityWXYZ(q);
  const Vec3 gb = GravityXYZW(q);
  const Vec3 gc = GravityFromAcc(ac);
  line("  몸통 좌표계 중력 방향:");
  line(GravityLine("(a) q=(w,x,y,z) 해석", ga));
  line(GravityLine("(b) q=(x,y,z,w) 해석", gb));
  line(GravityLine("(참고) -acc/|acc|", gc));
  const double ea = AngleDeg(ga, DownVector());
  const double eb = AngleDeg(gb, DownVector());
  line(Fmt("  → 똑바로 놓였을 때 (0,0,-1)에 가까운 쪽이 올바른 해석. 현재 더 가까운 쪽: %s",
           (ea <= eb) ? "(a) wxyz" : "(b) xyzw"));
  line("");

  // ---- 리모컨 ----
  line(std::string(kCyan) + "리모컨" + kReset);
  if (s.has_remote) {
    const auto& r = s.remote;
    line(Fmt("  lx %+.3f  ly %+.3f  rx %+.3f  ry %+.3f", r.lx(), r.ly(), r.rx(), r.ry()));
    line(Fmt("  keys = 0b%s (0x%04X)   눌린 비트: %s", Bits16(r.keys()).c_str(), r.keys(),
             SetBitsList(r.keys()).c_str()));
  } else {
    line("  rt/wirelesscontroller 수신 없음");
  }
  const uint16_t rk = RawRemoteKeys(s.low);
  line(Fmt("  (참고) lowstate.wireless_remote[2..3] = 0b%s (0x%04X)   눌린 비트: %s",
           Bits16(rk).c_str(), rk, SetBitsList(rk).c_str()));
  line("");

  line(Fmt("%s버튼 로그 (0->1 순간, 최근 %zu개):%s", kCyan, kLogLines, kReset));
  for (const auto& l : m.log) line("  " + l);

  out += kClearBelow;
  return out;
}

}  // namespace

int RunMonitor(const AppConfig& cfg, StateCache& cache) {
  term::RawMode raw;
  term::Print(std::string(term::kClearScreen) + term::kHideCursor);

  MonitorState m;
  auto next_render = Clock::now();
  while (!term::g_quit) {
    const int c = term::ReadKey(kPollMs);
    const Snapshot s = cache.Get();

    if (!m.has_base && s.has_low) SetBaseline(m, s);  // 프로그램 시작 시 기준값

    if (c == 'q' || c == 'Q') break;
    if ((c == 'r' || c == 'R') && s.has_low) {
      SetBaseline(m, s);
      PushLog(m, Fmt("[%8.2fs] 기준값 재설정", SinceStart(cfg, s)));
    }

    CheckEdges(m, s, cfg);

    if (s.now >= next_render) {
      term::Print(Render(m, s, cfg));
      next_render = s.now + std::chrono::milliseconds(kRenderPeriodMs);
    }
  }
  term::Print("\n");
  return 0;
}
