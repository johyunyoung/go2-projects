// READ-ONLY: no publishers
//
// 안내형 확인 모드: 사용자에게 동작을 지시하고 관절/IMU/버튼 배정을 측정해 YAML 로 저장한다.
#include "app.hpp"
#include "calibration.hpp"
#include "hypothesis.hpp"
#include "imu_math.hpp"
#include "terminal.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <map>
#include <numeric>

namespace {

using term::kBold;
using term::kBoldRed;
using term::kClearEol;
using term::kCyan;
using term::kGreen;
using term::kRed;
using term::kReset;
using term::kYellow;

// ---- 판정 기준 ----
constexpr double kMinMoveRad = 0.10;       // 이보다 작게 움직이면 판정 신뢰도 낮음 (~5.7deg)
constexpr double kCrosstalkMinRad = 0.03;  // 다른 관절 동시 움직임 경고 최소값
constexpr double kCrosstalkRatio = 0.30;   // 1위 대비 비율
constexpr double kMaxGapSec = 0.20;        // 측정 중 수신 공백 경고
constexpr double kImuWindowSec = 1.0;      // IMU 평균 시간
constexpr int kImuMinSamples = 10;
constexpr double kImuStillGyro = 0.05;     // rad/s, 이보다 크면 움직임 경고
constexpr double kFlatTolDeg = 15.0;       // 평평할 때 (0,0,-1)과 허용 각도
constexpr double kTiltMin = 0.10;          // 기울인 자세에서 중력 성분 최소값 (~6deg)
constexpr double kButtonTimeoutSec = 20.0;
constexpr double kReleaseWaitSec = 3.0;

enum class Key { None, Enter, Retry, Skip, Quit };

Key Classify(int c) {
  switch (c) {
    case '\n':
    case '\r':
      return Key::Enter;
    case 'r':
    case 'R':
      return Key::Retry;
    case 's':
    case 'S':
      return Key::Skip;
    case 'q':
    case 'Q':
      return Key::Quit;
    default:
      return Key::None;
  }
}

void Say(const std::string& s) { term::Print(s + "\n"); }
std::string C(const char* color, const std::string& s) { return std::string(color) + s + kReset; }
void Status(const std::string& s) { term::Print("\r" + s + kClearEol); }
void EndStatus() { term::Print(std::string("\r") + kClearEol); }

double Secs(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

void Header(const std::string& t) {
  Say("");
  Say(C(kBold, "==================== " + t + " ===================="));
}

// 허용된 키가 눌릴 때까지 대기. 기다리는 동안 lowstate 끊김을 상태줄로 표시.
Key WaitKey(StateCache& cache, const AppConfig& cfg, bool allow_retry, bool allow_skip) {
  term::FlushInput();
  bool warn_shown = false;
  auto next_check = Clock::now();
  while (true) {
    if (term::g_quit) return Key::Quit;
    const Key k = Classify(term::ReadKey(50));
    if (k == Key::Enter || k == Key::Quit || (k == Key::Retry && allow_retry) ||
        (k == Key::Skip && allow_skip)) {
      if (warn_shown) EndStatus();
      return k;
    }
    const auto now = Clock::now();
    if (now >= next_check) {
      const Snapshot s = cache.Get();
      if (LowStateStale(s, cfg)) {
        Status(C(kBoldRed, "lowstate 수신 없음 — 인터페이스 이름 / 도메인 ID / 랜선 / IP(192.168.123.x) 확인"));
        warn_shown = true;
      } else if (warn_shown) {
        EndStatus();
        warn_shown = false;
      }
      next_check = now + std::chrono::milliseconds(500);
    }
  }
}

bool WaitForLowState(StateCache& cache, const AppConfig& cfg) {
  Say("rt/lowstate 수신 대기 중...   (q: 종료)");
  bool hints_shown = false;
  while (!term::g_quit) {
    const Snapshot s = cache.Get();
    if (s.has_low && s.LowAge() < 0.5) {
      Say(C(kGreen, Fmt("rt/lowstate 수신 확인 (tick %u)", s.low.tick())));
      return true;
    }
    if (!hints_shown && LowStateStale(s, cfg)) {
      for (const auto& l : LinkHintLines(cfg, s)) Say(C(kBoldRed, l));
      Say("(계속 기다립니다. q: 종료)");
      hints_shown = true;
    }
    if (Classify(term::ReadKey(100)) == Key::Quit) return false;
  }
  return false;
}

// 단계 시작 안내. Enter=시작, s=단계 건너뛰기, q=중단
Key StepIntro(StateCache& cache, const AppConfig& cfg, const std::string& title,
              const std::vector<std::string>& lines) {
  Header(title);
  for (const auto& l : lines) Say(l);
  Say(C(kCyan, "[Enter] 이 단계 시작   s: 이 단계 건너뛰기   q: 중단"));
  return WaitKey(cache, cfg, false, true);
}

std::string NowIso() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &tm);
  return buf;
}

std::string PartialPath(const std::string& p) {
  const std::string ext = ".yaml";
  if (p.size() > ext.size() && p.compare(p.size() - ext.size(), ext.size(), ext) == 0)
    return p.substr(0, p.size() - ext.size()) + ".partial.yaml";
  return p + ".partial.yaml";
}

// =====================================================================
// (1) 관절 확인
// =====================================================================

// 이름/가설 인덱스만 남기고 측정값 초기화
void ResetJoint(JointResult& r) {
  JointResult fresh;
  fresh.name = r.name;
  fresh.hyp_index = r.hyp_index;
  r = fresh;
}

std::string LegKo(const std::string& leg) {
  if (leg == "FR") return "앞-오른쪽(FR)";
  if (leg == "FL") return "앞-왼쪽(FL)";
  if (leg == "RR") return "뒤-오른쪽(RR)";
  if (leg == "RL") return "뒤-왼쪽(RL)";
  return leg;
}

std::string JointInstruction(const std::string& name) {
  const std::string leg = LegKo(name.substr(0, 2));
  const std::string part = name.substr(3);
  if (part == "hip") return leg + " 다리의 hip 을 로봇 몸 바깥쪽으로 천천히 벌리세요";
  if (part == "thigh") return leg + " 다리의 thigh 를 앞쪽으로 천천히 들어 올리세요 (다리를 앞으로)";
  if (part == "calf") return leg + " 다리의 calf 를 무릎이 더 접히는 방향으로 천천히 움직이세요";
  return leg + " 다리의 " + part + " 를 움직이세요";
}

void PrintJointResult(const JointResult& r) {
  const double deg = r.peak_delta * kRad2Deg;
  const std::string res = Fmt("  결과: 가장 크게 변한 모터 index %d, 부호 %+d, 변화 %+.4f rad (%+.1f deg)",
                              r.index, r.sign, r.peak_delta, deg);
  if (r.Mismatch()) {
    Say(C(kBoldRed, res));
    Say(C(kBoldRed, Fmt("  !!! 가설과 다름: 가설 index %d (%s) / 측정 index %d (가설상 이름 %s) !!!",
                        r.hyp_index, r.name.c_str(), r.index, hyp::kJointName[r.index])));
  } else {
    Say(C(kGreen, res + "   [가설과 일치]"));
  }
  for (const auto& w : r.warnings) Say(C(kYellow, "  경고: " + w));
}

// false 를 반환하면 사용자가 중단
bool MeasureJoint(StateCache& cache, const AppConfig& cfg, int n, int total, JointResult& r) {
  while (true) {
    Say("");
    Say(C(kBold, Fmt("[관절 %d/%d] %s   (가설 index %d)", n, total, r.name.c_str(), r.hyp_index)));
    Say("  ▶ " + JointInstruction(r.name));
    Say("  움직일 여유가 있도록 자세를 먼저 잡은 뒤 [Enter] 로 측정 시작   (s: 건너뛰기, q: 중단)");
    const Key k = WaitKey(cache, cfg, false, true);
    if (k == Key::Quit) return false;
    if (k == Key::Skip) {
      ResetJoint(r);
      r.warnings.push_back("건너뜀");
      Say(C(kYellow, "  건너뜀"));
      return true;
    }

    const Snapshot s0 = cache.Get();
    if (!s0.has_low || s0.LowAge() > 0.5) {
      Say(C(kBoldRed, "  lowstate 수신이 없어 측정할 수 없습니다. 연결을 확인한 뒤 다시 시도하세요."));
      continue;
    }

    std::array<double, hyp::kNumJoints> base{}, peak{}, cur{};
    for (int i = 0; i < hyp::kNumJoints; ++i) base[i] = s0.low.motor_state()[i].q();

    Say("  측정 중... 지시대로 천천히 움직인 뒤 [Enter] 로 완료   (r: 처음부터 다시, q: 중단)");
    term::FlushInput();
    uint64_t last_count = s0.low_count;
    Clock::time_point last_rx = s0.low_time;
    double max_gap = 0.0;
    auto next_status = Clock::now();
    bool restart = false;

    while (true) {
      if (term::g_quit) {
        EndStatus();
        return false;
      }
      const Key kk = Classify(term::ReadKey(5));
      if (kk == Key::Quit) {
        EndStatus();
        return false;
      }
      if (kk == Key::Retry) {
        restart = true;
        break;
      }
      const Snapshot s = cache.Get();
      if (s.has_low && s.low_count != last_count) {
        max_gap = std::max(max_gap, Secs(s.low_time - last_rx));
        last_rx = s.low_time;
        last_count = s.low_count;
        for (int i = 0; i < hyp::kNumJoints; ++i) {
          const double d = s.low.motor_state()[i].q() - base[i];
          cur[i] = d;
          if (std::fabs(d) > std::fabs(peak[i])) peak[i] = d;
        }
      } else {
        max_gap = std::max(max_gap, Secs(s.now - last_rx));
      }
      if (kk == Key::Enter) break;
      if (s.now >= next_status) {
        int top = 0;
        for (int i = 1; i < hyp::kNumJoints; ++i)
          if (std::fabs(cur[i]) > std::fabs(cur[top])) top = i;
        Status(Fmt("  현재 최대 변화: #%d (%s*) %+.3f rad (%+.1f deg)", top, hyp::kJointName[top],
                   cur[top], cur[top] * kRad2Deg));
        next_status = s.now + std::chrono::milliseconds(100);
      }
    }
    EndStatus();
    if (restart) {
      Say(C(kYellow, "  처음부터 다시 측정합니다."));
      continue;
    }

    // 최대 변화 순으로 정렬
    std::array<int, hyp::kNumJoints> idx{};
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(),
              [&peak](int a, int b) { return std::fabs(peak[a]) > std::fabs(peak[b]); });
    const int a = idx[0];
    const int b = idx[1];

    r.measured = true;
    r.index = a;
    r.peak_delta = peak[a];
    r.final_delta = cur[a];
    r.sign = peak[a] >= 0.0 ? +1 : -1;
    r.second_index = b;
    r.second_delta = peak[b];
    r.warnings.clear();
    if (std::fabs(peak[a]) < kMinMoveRad) {
      r.warnings.push_back(Fmt("움직임이 작음 (%.3f rad < %.2f rad) — 판정 신뢰도 낮음",
                               std::fabs(peak[a]), kMinMoveRad));
    }
    if (std::fabs(peak[b]) >= kCrosstalkMinRad &&
        std::fabs(peak[b]) >= kCrosstalkRatio * std::fabs(peak[a])) {
      r.warnings.push_back(Fmt("다른 관절도 함께 움직임: #%d (%s*) %+.3f rad (1위 대비 %.0f%%)", b,
                               hyp::kJointName[b], peak[b],
                               100.0 * std::fabs(peak[b]) / std::max(std::fabs(peak[a]), 1e-9)));
    }
    if (max_gap > kMaxGapSec) {
      r.warnings.push_back(Fmt("측정 중 lowstate 수신 공백 %.2f s", max_gap));
    }
    PrintJointResult(r);

    Say(C(kCyan, "  [Enter] 다음   r: 다시 측정   q: 중단"));
    const Key k2 = WaitKey(cache, cfg, true, false);
    if (k2 == Key::Quit) return false;
    if (k2 == Key::Retry) {
      ResetJoint(r);
      continue;
    }
    return true;
  }
}

void CheckDuplicateJoints(CalibrationData& d) {
  std::map<int, std::vector<std::string>> by_index;
  for (const auto& j : d.joints)
    if (j.measured) by_index[j.index].push_back(j.name);
  for (auto& j : d.joints) {
    if (!j.measured) continue;
    const auto& names = by_index[j.index];
    if (names.size() < 2) continue;
    std::string list;
    for (const auto& nm : names) list += (list.empty() ? "" : ", ") + nm;
    j.warnings.push_back(Fmt("같은 모터 index %d 가 여러 관절에 배정됨: %s", j.index, list.c_str()));
  }
  for (const auto& kv : by_index) {
    if (kv.second.size() < 2) continue;
    std::string list;
    for (const auto& nm : kv.second) list += (list.empty() ? "" : ", ") + nm;
    const std::string w = Fmt("모터 index %d 중복 배정: %s", kv.first, list.c_str());
    d.warnings.push_back(w);
    Say(C(kBoldRed, "경고: " + w));
  }
}

// =====================================================================
// (2) IMU 확인
// =====================================================================

std::string V3(const Vec3& v) { return Fmt("(%+.3f, %+.3f, %+.3f)", v.x, v.y, v.z); }

ImuPose MakePose(const std::string& key, const std::string& description) {
  ImuPose p;
  p.key = key;
  p.description = description;
  return p;
}

bool MeasurePose(StateCache& cache, const AppConfig& cfg, ImuPose& p) {
  while (true) {
    Say("");
    Say(C(kBold, "[IMU] " + p.description));
    Say("  자세를 잡고 움직이지 않게 고정한 뒤 [Enter] → 1초 평균 측정   (s: 건너뛰기, q: 중단)");
    const Key k = WaitKey(cache, cfg, false, true);
    if (k == Key::Quit) return false;
    if (k == Key::Skip) {
      p.measured = false;
      p.warnings = {"건너뜀"};
      Say(C(kYellow, "  건너뜀"));
      return true;
    }

    Say("  측정 중 (1초)... 움직이지 마세요");
    term::FlushInput();
    std::array<double, 4> sq{};
    std::array<double, 3> srpy{}, sg{}, sa{};
    double sgyro_norm = 0.0;
    int n = 0;
    uint64_t last = 0;
    const auto t0 = Clock::now();
    while (Secs(Clock::now() - t0) < kImuWindowSec) {
      if (term::g_quit || Classify(term::ReadKey(2)) == Key::Quit) return false;
      const Snapshot s = cache.Get();
      if (!s.has_low || s.low_count == last) continue;
      last = s.low_count;
      const auto& imu = s.low.imu_state();
      for (int i = 0; i < 4; ++i) sq[i] += imu.quaternion()[i];
      for (int i = 0; i < 3; ++i) {
        srpy[i] += imu.rpy()[i];
        sg[i] += imu.gyroscope()[i];
        sa[i] += imu.accelerometer()[i];
      }
      const auto& g = imu.gyroscope();
      sgyro_norm += std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
      ++n;
    }
    if (n < kImuMinSamples) {
      Say(C(kBoldRed, Fmt("  샘플 부족 (%d개) — lowstate 수신을 확인하고 다시 측정하세요.", n)));
      continue;
    }

    p.measured = true;
    p.samples = n;
    p.warnings.clear();
    for (int i = 0; i < 4; ++i) p.quat[i] = sq[i] / n;
    for (int i = 0; i < 3; ++i) {
      p.rpy[i] = srpy[i] / n;
      p.gyro[i] = sg[i] / n;
      p.acc[i] = sa[i] / n;
    }
    p.g_wxyz = GravityWXYZ(p.quat);
    p.g_xyzw = GravityXYZW(p.quat);
    p.g_acc = GravityFromAcc(p.acc);
    const double gyro_mean = sgyro_norm / n;
    if (gyro_mean > kImuStillGyro)
      p.warnings.push_back(Fmt("측정 중 움직임 감지 (평균 |gyro| = %.3f rad/s)", gyro_mean));

    Say(Fmt("  samples %d", n));
    Say(Fmt("  quaternion 원시 [0..3] 평균: [%+.5f, %+.5f, %+.5f, %+.5f]", p.quat[0], p.quat[1],
            p.quat[2], p.quat[3]));
    Say(Fmt("  rpy [deg]: (%+.2f, %+.2f, %+.2f)", p.rpy[0] * kRad2Deg, p.rpy[1] * kRad2Deg,
            p.rpy[2] * kRad2Deg));
    Say(Fmt("  accelerometer: (%+.3f, %+.3f, %+.3f)", p.acc[0], p.acc[1], p.acc[2]));
    Say(Fmt("  중력(몸통) (a) wxyz 해석: %s   (0,0,-1)과 %5.1f deg, -acc와 %5.1f deg",
            V3(p.g_wxyz).c_str(), AngleDeg(p.g_wxyz, DownVector()), AngleDeg(p.g_wxyz, p.g_acc)));
    Say(Fmt("  중력(몸통) (b) xyzw 해석: %s   (0,0,-1)과 %5.1f deg, -acc와 %5.1f deg",
            V3(p.g_xyzw).c_str(), AngleDeg(p.g_xyzw, DownVector()), AngleDeg(p.g_xyzw, p.g_acc)));
    Say(Fmt("  (참고) -acc/|acc|        : %s", V3(p.g_acc).c_str()));
    for (const auto& w : p.warnings) Say(C(kYellow, "  경고: " + w));

    Say(C(kCyan, "  [Enter] 다음   r: 다시 측정   q: 중단"));
    const Key k2 = WaitKey(cache, cfg, true, false);
    if (k2 == Key::Quit) return false;
    if (k2 == Key::Retry) {
      p.measured = false;
      continue;
    }
    return true;
  }
}

const ImuPose* FindPose(const ImuResult& r, const std::string& key) {
  for (const auto& p : r.poses)
    if (p.key == key && p.measured) return &p;
  return nullptr;
}

// 두 쿼터니언 해석 중 물리적으로 맞는 쪽 판정
void JudgeImu(ImuResult& r) {
  r.notes.clear();
  const ImuPose* flat = FindPose(r, "flat");
  const ImuPose* nose = FindPose(r, "nose_up");
  const ImuPose* right = FindPose(r, "right_up");
  r.measured = flat || nose || right;
  if (!r.measured) {
    r.order.clear();
    return;
  }

  std::array<bool, 2> pass{{false, false}};
  for (int i = 0; i < 2; ++i) {
    auto G = [i](const ImuPose* p) { return i == 0 ? p->g_wxyz : p->g_xyzw; };
    // 기준 1: 평평할 때 (0,0,-1)에 가까운가
    r.flat_err_deg[i] = flat ? AngleDeg(G(flat), DownVector()) : std::nan("");
    const bool flat_ok = !flat || r.flat_err_deg[i] < kFlatTolDeg;
    // 기준 2: 머리 들면 g.x < 0, 오른쪽 들면 g.y > 0
    bool tilt_ok = true;
    if (nose) tilt_ok = tilt_ok && G(nose).x < -kTiltMin;
    if (right) tilt_ok = tilt_ok && G(right).y > kTiltMin;
    r.tilt_sign_ok[i] = (nose || right) ? tilt_ok : false;
    // 참고: 가속도계 방향과의 평균 각도
    double sum = 0.0;
    int cnt = 0;
    for (const ImuPose* p : {flat, nose, right}) {
      if (!p) continue;
      const double e = AngleDeg(G(p), p->g_acc);
      if (std::isfinite(e)) {
        sum += e;
        ++cnt;
      }
    }
    r.acc_err_deg[i] = cnt ? sum / cnt : std::nan("");
    pass[i] = flat_ok && tilt_ok;
  }

  const char* names[2] = {"wxyz", "xyzw"};
  if (pass[0] != pass[1]) {
    const int w = pass[0] ? 0 : 1;
    r.order = names[w];
    r.confident = true;
    if (!(flat && (nose || right))) {
      r.confident = false;
      r.notes.push_back("일부 자세만 측정되어 판정 근거가 부족함");
    }
    const double ew = r.acc_err_deg[w], el = r.acc_err_deg[1 - w];
    if (std::isfinite(ew) && std::isfinite(el) && ew > el) {
      r.confident = false;
      r.notes.push_back("자세 기준 판정과 가속도계 비교 결과가 다름 — 재확인 필요");
    }
  } else {
    // 둘 다 통과/실패 → 가속도계 일치도로 보조 판정
    const double e0 = r.acc_err_deg[0], e1 = r.acc_err_deg[1];
    r.confident = false;
    if (std::isfinite(e0) && std::isfinite(e1)) {
      r.order = e0 <= e1 ? names[0] : names[1];
      r.notes.push_back(pass[0] ? "두 해석 모두 자세 기준 통과 — 가속도계 일치도로 판정 (불확실)"
                                : "두 해석 모두 자세 기준 실패 — 가속도계 일치도로 판정 (불확실)");
    } else {
      r.order.clear();
      r.notes.push_back("판정 불가");
    }
  }

  // 자세 지시가 제대로 수행됐는지 가속도계로 확인 + rpy 부호 기록
  if (flat && flat->acc[2] < 0.0)
    r.notes.push_back(Fmt("평평할 때 accelerometer z = %+.2f (음수) — '정지 시 +g' 가정과 다름",
                          flat->acc[2]));
  if (nose) {
    if (!(nose->g_acc.x < -kTiltMin))
      r.notes.push_back(Fmt("머리 들기 자세에서 가속도계 기준 g.x = %+.2f — 기울기가 작거나 방향이 다를 수 있음",
                            nose->g_acc.x));
    r.notes.push_back(Fmt("머리 들기 시 rpy = (%+.1f, %+.1f, %+.1f) deg", nose->rpy[0] * kRad2Deg,
                          nose->rpy[1] * kRad2Deg, nose->rpy[2] * kRad2Deg));
  }
  if (right) {
    if (!(right->g_acc.y > kTiltMin))
      r.notes.push_back(Fmt("오른쪽 들기 자세에서 가속도계 기준 g.y = %+.2f — 기울기가 작거나 방향이 다를 수 있음",
                            right->g_acc.y));
    r.notes.push_back(Fmt("오른쪽 들기 시 rpy = (%+.1f, %+.1f, %+.1f) deg", right->rpy[0] * kRad2Deg,
                          right->rpy[1] * kRad2Deg, right->rpy[2] * kRad2Deg));
  }
}

void PrintImuResult(const ImuResult& r) {
  Say("");
  Say(C(kBold, "[IMU 판정]"));
  Say(Fmt("  (a) wxyz: 평평 시 (0,0,-1)과 %5.1f deg, 기울기 부호 %s, -acc와 평균 %5.1f deg",
          r.flat_err_deg[0], r.tilt_sign_ok[0] ? "OK" : "NG", r.acc_err_deg[0]));
  Say(Fmt("  (b) xyzw: 평평 시 (0,0,-1)과 %5.1f deg, 기울기 부호 %s, -acc와 평균 %5.1f deg",
          r.flat_err_deg[1], r.tilt_sign_ok[1] ? "OK" : "NG", r.acc_err_deg[1]));
  if (r.order.empty()) {
    Say(C(kBoldRed, "  판정 불가"));
  } else {
    const std::string s = Fmt("  quaternion_order = %s (%s)", r.order.c_str(),
                              r.confident ? "확정" : "불확실");
    if (r.Mismatch()) {
      Say(C(kBoldRed, s + Fmt("   !!! 가설(%s)과 다름 !!!", hyp::kQuaternionOrder)));
    } else {
      Say(C(r.confident ? kGreen : kYellow, s + "   [가설과 일치]"));
    }
  }
  for (const auto& n : r.notes) Say(C(kYellow, "  참고: " + n));
}

// =====================================================================
// (3) 버튼 확인
// =====================================================================

enum class KeySrc { Remote, Raw };

bool ReadKeys(const Snapshot& s, KeySrc src, uint16_t& keys, uint64_t& seq) {
  if (src == KeySrc::Remote) {
    if (!s.has_remote) return false;
    keys = s.remote.keys();
    seq = s.remote_count;
    return true;
  }
  if (!s.has_low) return false;
  keys = RawRemoteKeys(s.low);
  seq = s.low_count;
  return true;
}

int LowestBit(uint16_t v) {
  for (int b = 0; b < 16; ++b)
    if (v & (1u << b)) return b;
  return -1;
}

// false 를 반환하면 사용자가 중단
bool MeasureButton(StateCache& cache, KeySrc src, int n, int total, ButtonResult& r) {
  Say(C(kBold, Fmt("  [%2d/%d] '%s' 버튼을 한 번 눌렀다 떼세요", n, total, r.name.c_str())) +
      "   (s: 건너뛰기, q: 중단)");
  term::FlushInput();

  int phase = 0;  // 0: 모두 뗄 때까지 대기, 1: 눌림 대기, 2: 뗄 때까지 추가 비트 수집
  uint16_t prev = 0, first = 0, all = 0;
  uint64_t last_seq = 0;
  const auto t0 = Clock::now();
  Clock::time_point t_press{};
  bool held_warned = false;

  while (true) {
    if (term::g_quit) {
      EndStatus();
      return false;
    }
    const Key k = Classify(term::ReadKey(5));
    if (k == Key::Quit) {
      EndStatus();
      return false;
    }
    if (k == Key::Skip) {
      EndStatus();
      r.measured = false;
      r.warnings.push_back("건너뜀");
      Say(C(kYellow, "         건너뜀"));
      return true;
    }
    const auto now = Clock::now();
    if (phase < 2 && Secs(now - t0) > kButtonTimeoutSec) {
      EndStatus();
      r.measured = false;
      r.warnings.push_back(Fmt("시간 초과 (%.0f s) — 입력 없음", kButtonTimeoutSec));
      Say(C(kYellow, "         시간 초과 — 건너뜀"));
      return true;
    }

    const Snapshot s = cache.Get();
    uint16_t keys = 0;
    uint64_t seq = 0;
    if (!ReadKeys(s, src, keys, seq)) continue;

    // phase 0 은 새 메시지 여부와 무관하게 최신 값으로 판단
    // (값이 바뀔 때만 발행되는 경우에도 진행되도록)
    if (phase != 0 && seq == last_seq) {
      if (phase == 2 && Secs(now - t_press) > kReleaseWaitSec) break;
      continue;
    }
    last_seq = seq;

    if (phase == 0) {
      if (keys == 0) {
        prev = 0;
        phase = 1;
        if (held_warned) EndStatus();
      } else if (Secs(now - t0) > kReleaseWaitSec) {
        // 계속 눌린 것처럼 보이는 비트가 있으면 현재 값 기준으로 진행
        prev = keys;
        phase = 1;
        r.warnings.push_back("시작 시 이미 1인 비트가 있었음: 0b" + Bits16(keys));
        EndStatus();
      } else if (!held_warned) {
        Status(C(kYellow, "         모든 버튼에서 손을 떼세요 (현재 0b" + Bits16(keys) + ")"));
        held_warned = true;
      }
      continue;
    }

    const uint16_t rising = static_cast<uint16_t>(keys & ~prev);
    prev = keys;
    if (phase == 1) {
      if (rising) {
        first = rising;
        all = rising;
        phase = 2;
        t_press = now;
      }
    } else {  // phase 2
      all |= rising;
      if ((keys & first) == 0 || Secs(now - t_press) > kReleaseWaitSec) break;
    }
  }

  r.measured = true;
  r.bit = LowestBit(first);
  for (int b = 0; b < 16; ++b)
    if ((all & (1u << b)) && b != r.bit) r.other_bits.push_back(b);
  if (!r.other_bits.empty()) {
    std::string list;
    for (int b : r.other_bits) list += (list.empty() ? "" : ", ") + std::to_string(b);
    r.warnings.push_back("다른 비트도 함께 0->1: " + list);
  }

  const std::string res = Fmt("         → bit %d   (참고 가설: bit %d)", r.bit, r.hyp_bit);
  if (r.Mismatch()) {
    Say(C(kBoldRed, res + Fmt("   !!! 가설과 다름 (가설상 bit %d = %s) !!!", r.bit,
                              hyp::ButtonNameForBit(r.bit))));
  } else {
    Say(C(kGreen, res + "   [가설과 일치]"));
  }
  for (const auto& w : r.warnings) Say(C(kYellow, "         경고: " + w));
  return true;
}

void CheckDuplicateButtons(CalibrationData& d) {
  std::map<int, std::vector<std::string>> by_bit;
  for (const auto& b : d.buttons)
    if (b.measured) by_bit[b.bit].push_back(b.name);
  for (auto& b : d.buttons) {
    if (!b.measured || by_bit[b.bit].size() < 2) continue;
    b.warnings.push_back(Fmt("같은 bit %d 가 여러 버튼에서 검출됨", b.bit));
  }
  for (const auto& kv : by_bit) {
    if (kv.second.size() < 2) continue;
    std::string list;
    for (const auto& nm : kv.second) list += (list.empty() ? "" : ", ") + nm;
    const std::string w = Fmt("bit %d 중복 검출: %s", kv.first, list.c_str());
    d.warnings.push_back(w);
    Say(C(kBoldRed, "경고: " + w));
  }
}

// =====================================================================
// 결과 요약
// =====================================================================

void PrintSummary(const CalibrationData& d) {
  Header("결과 요약");
  Say(C(kCyan, "관절            가설idx  측정idx  sign  변화[deg]"));
  for (const auto& j : d.joints) {
    if (!j.measured) {
      Say(C(kYellow, Fmt("  %-12s  %5d      -      -        -     미측정", j.name.c_str(), j.hyp_index)));
      continue;
    }
    const std::string row = Fmt("  %-12s  %5d  %7d   %+d   %+8.1f", j.name.c_str(), j.hyp_index,
                                j.index, j.sign, j.peak_delta * kRad2Deg);
    if (j.Mismatch())
      Say(C(kBoldRed, row + "   MISMATCH"));
    else
      Say(row + (j.warnings.empty() ? "   OK" : C(kYellow, "   OK (경고 있음)")));
  }

  Say("");
  if (!d.imu.measured || d.imu.order.empty()) {
    Say(C(kYellow, "IMU: 판정 없음"));
  } else {
    const std::string s = Fmt("IMU: quaternion_order = %s (%s), 가설 %s", d.imu.order.c_str(),
                              d.imu.confident ? "확정" : "불확실", hyp::kQuaternionOrder);
    Say(d.imu.Mismatch() ? C(kBoldRed, s + "   MISMATCH") : s + "   OK");
  }

  Say("");
  Say(C(kCyan, "버튼        측정bit  가설bit"));
  for (const auto& b : d.buttons) {
    if (!b.measured) {
      Say(C(kYellow, Fmt("  %-8s       -     %3d   미측정", b.name.c_str(), b.hyp_bit)));
      continue;
    }
    const std::string row = Fmt("  %-8s     %3d     %3d", b.name.c_str(), b.bit, b.hyp_bit);
    Say(b.Mismatch() ? C(kBoldRed, row + "   MISMATCH") : row + "   OK");
  }

  Say("");
  const int mm = d.MismatchCount();
  if (mm > 0)
    Say(C(kBoldRed, Fmt("!!! 가설과 다른 항목: %d개 — 위의 MISMATCH 와 yaml 주석을 확인하세요 !!!", mm)));
  else
    Say(C(kGreen, "가설과 다른 항목 없음 (측정된 항목 기준)"));
  for (const auto& w : d.warnings) Say(C(kYellow, "경고: " + w));
}

}  // namespace

// =====================================================================

int RunGuided(const AppConfig& cfg, StateCache& cache) {
  if (!isatty(STDIN_FILENO)) {
    term::Print("안내형 모드는 대화형 터미널에서 실행해야 합니다.\n");
    return 1;
  }
  term::RawMode raw;

  CalibrationData d;
  d.measured_at = NowIso();
  d.iface = cfg.iface;
  d.domain_id = cfg.domain_id;
  for (int i = 0; i < hyp::kNumJoints; ++i) {
    JointResult j;
    j.name = hyp::kJointName[i];
    j.hyp_index = i;
    d.joints.push_back(j);
  }
  d.imu.poses = {
      MakePose("flat", "평평한 바닥에 둔 상태: 엎드린 자세 그대로 평평한 바닥에 두고 손을 떼세요"),
      MakePose("nose_up",
               "앞쪽을 들어 올림(머리 들기): 로봇 앞쪽(머리)을 약 15~30도 들어 올린 채 고정하세요. "
               "뒤쪽은 바닥에 둡니다"),
      MakePose("right_up",
               "오른쪽을 들어 올림: 로봇 기준 오른쪽(머리 방향을 보고 오른편)을 약 15~30도 들어 올린 채 "
               "고정하세요. 왼쪽은 바닥에 둡니다"),
  };
  for (const char* name : hyp::kGuidedButtons) {
    ButtonResult b;
    b.name = name;
    b.hyp_bit = hyp::ButtonBitHyp(name);
    d.buttons.push_back(b);
  }

  Header("go2_inspect 안내형 확인 모드   READ-ONLY (no publishers)");
  Say("이 프로그램은 상태만 읽으며 로봇에 어떤 명령도 보내지 않습니다.");
  Say(C(kBoldRed, "측정 전 확인: 로봇을 바닥에 엎드린 댐핑 상태로 두세요. (서 있거나 보행 중인 상태에서 절대 사용 금지)"));
  Say("  - 관절은 손으로 천천히 움직이세요. 댐핑 상태에서는 빠르게 움직일수록 저항이 큽니다.");
  Say("  - 키: [Enter] 시작/완료,  r 다시 측정,  s 건너뛰기,  q 중단 (중단 시 부분 결과를 별도 파일로 저장)");
  Say(Fmt("  - iface=%s  domain=%d  저장 파일=%s", cfg.iface.c_str(), cfg.domain_id,
          cfg.output_path.c_str()));
  Say(C(kCyan, "[Enter] 계속   q: 종료"));
  if (WaitKey(cache, cfg, false, false) == Key::Quit) return 0;

  if (!WaitForLowState(cache, cfg)) return 1;

  bool quit = false;

  // ---- (1) 관절 ----
  {
    const Key k = StepIntro(cache, cfg, "1/3 관절 확인",
                            {"12개 관절을 하나씩 지시대로 움직이고, 가장 크게 변한 모터 인덱스와 부호를 기록합니다.",
                             "지시 방향: hip = 몸 바깥쪽으로 벌림, thigh = 다리를 앞으로 들어 올림, calf = 무릎을 더 접음",
                             "한 번에 한 관절만 움직이세요. 이름은 가설 순서대로 지시합니다."});
    if (k == Key::Quit) {
      quit = true;
    } else if (k == Key::Enter) {
      const int total = static_cast<int>(d.joints.size());
      for (int i = 0; i < total && !quit; ++i) {
        if (!MeasureJoint(cache, cfg, i + 1, total, d.joints[i])) quit = true;
      }
      if (!quit) {
        d.joints_done = true;
        CheckDuplicateJoints(d);
      }
    }
  }

  // ---- (2) IMU ----
  if (!quit) {
    const Key k = StepIntro(cache, cfg, "2/3 IMU 확인",
                            {"세 자세에서 각각 1초 평균을 기록하고, 쿼터니언 (w,x,y,z) / (x,y,z,w) 해석 중 맞는 쪽을 판정합니다.",
                             "순서: 평평한 바닥 → 앞쪽(머리) 들기 → 오른쪽 들기",
                             "로봇이 무거우니 두 사람이 함께 들고, 측정하는 1초 동안은 움직이지 마세요."});
    if (k == Key::Quit) {
      quit = true;
    } else if (k == Key::Enter) {
      for (auto& p : d.imu.poses) {
        if (!MeasurePose(cache, cfg, p)) {
          quit = true;
          break;
        }
      }
      JudgeImu(d.imu);
      PrintImuResult(d.imu);
      if (!quit) d.imu_done = true;
    }
  }

  // ---- (3) 버튼 ----
  if (!quit) {
    const Key k = StepIntro(cache, cfg, "3/3 리모컨 버튼 확인",
                            {"리모컨을 켜고, 지시하는 버튼을 하나씩 눌렀다 떼세요. 감지되면 자동으로 다음 버튼으로 넘어갑니다.",
                             "버튼: A B X Y L1 L2 R1 R2 start select 방향키(상 하 좌 우)",
                             "주의: 리모컨 버튼은 로봇의 기본 동작에도 연결될 수 있습니다. 로봇이 엎드린 댐핑 상태인지 다시 확인하세요."});
    if (k == Key::Quit) {
      quit = true;
    } else if (k == Key::Enter) {
      // 입력 소스 결정: rt/wirelesscontroller 우선
      KeySrc src = KeySrc::Remote;
      bool have_src = false;
      const auto t0 = Clock::now();
      while (Secs(Clock::now() - t0) < 2.0 && !term::g_quit) {
        const Snapshot s = cache.Get();
        if (s.has_remote && s.RemoteAge() < 1.0) {
          have_src = true;
          break;
        }
        term::ReadKey(50);
      }
      if (!have_src) {
        const Snapshot s = cache.Get();
        if (s.has_low) {
          src = KeySrc::Raw;
          have_src = true;
          const std::string w =
              "rt/wirelesscontroller 미수신 → lowstate.wireless_remote[2..3] (참고 가설 레이아웃) 사용";
          d.warnings.push_back(w);
          Say(C(kYellow, "경고: " + w));
        } else {
          const std::string w = "리모컨 상태를 받을 수 없어 버튼 단계를 건너뜀";
          d.warnings.push_back(w);
          Say(C(kBoldRed, w));
        }
      }
      if (have_src) {
        const int total = static_cast<int>(d.buttons.size());
        for (int i = 0; i < total; ++i) {
          d.buttons[i].source = (src == KeySrc::Remote) ? "wirelesscontroller" : "lowstate_raw";
          if (!MeasureButton(cache, src, i + 1, total, d.buttons[i])) {
            quit = true;
            break;
          }
        }
        if (!quit) {
          d.buttons_done = true;
          CheckDuplicateButtons(d);
        }
      }
    }
  }

  // ---- (4) 저장 ----
  d.complete = !quit;
  PrintSummary(d);

  const std::string path = d.complete ? cfg.output_path : PartialPath(cfg.output_path);
  std::string err;
  if (SaveCalibrationYaml(path, d, &err)) {
    Say(C(kGreen, "저장 완료: " + path) + (d.complete ? "" : C(kYellow, "  (중단된 부분 결과)")));
  } else {
    Say(C(kBoldRed, "저장 실패: " + err));
    return 1;
  }
  return 0;
}
