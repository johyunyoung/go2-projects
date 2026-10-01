// READ-ONLY: no publishers
//
// 안내형 모드 측정 결과 구조체와 YAML 저장.
#pragma once

#include "hypothesis.hpp"
#include "imu_math.hpp"

#include <array>
#include <cmath>
#include <string>
#include <vector>

struct JointResult {
  std::string name;   // 가설 이름 (지시 대상)
  int hyp_index = -1; // 가설 인덱스
  bool measured = false;
  int index = -1;            // 측정: 가장 크게 변한 모터 인덱스
  int sign = 0;              // 지시 방향으로 움직였을 때 q 변화 부호 (+1/-1)
  double peak_delta = 0.0;   // 그 모터의 기준 대비 최대 변화 [rad] (부호 포함)
  double final_delta = 0.0;  // 완료 시점 변화 [rad]
  int second_index = -1;     // 두 번째로 많이 변한 모터
  double second_delta = 0.0;
  std::vector<std::string> warnings;

  bool Mismatch() const { return measured && index != hyp_index; }
};

struct ImuPose {
  std::string key;          // flat / nose_up / right_up
  std::string description;  // 지시문
  bool measured = false;
  int samples = 0;
  std::array<double, 4> quat{};  // 원시 [0..3] 평균
  std::array<double, 3> rpy{};
  std::array<double, 3> gyro{};
  std::array<double, 3> acc{};
  Vec3 g_wxyz, g_xyzw, g_acc;
  std::vector<std::string> warnings;
};

struct ImuResult {
  bool measured = false;
  std::string order;        // "wxyz" / "xyzw" / "" (판정 불가)
  bool confident = false;
  // [0] = wxyz 해석, [1] = xyzw 해석
  std::array<double, 2> flat_err_deg{{std::nan(""), std::nan("")}};  // 평평할 때 (0,0,-1)과의 각도
  std::array<double, 2> acc_err_deg{{std::nan(""), std::nan("")}};   // -acc 방향과의 평균 각도
  std::array<bool, 2> tilt_sign_ok{{false, false}};                  // 기울인 자세의 부호 일치
  std::vector<std::string> notes;
  std::vector<ImuPose> poses;

  bool Mismatch() const { return measured && !order.empty() && order != hyp::kQuaternionOrder; }
};

struct ButtonResult {
  std::string name;
  int hyp_bit = -1;
  bool measured = false;
  int bit = -1;
  std::vector<int> other_bits;  // 함께 0→1 된 다른 비트
  std::string source;           // wirelesscontroller / lowstate_raw
  std::vector<std::string> warnings;

  bool Mismatch() const { return measured && bit != hyp_bit; }
};

struct CalibrationData {
  std::string measured_at;
  std::string iface;
  int domain_id = 0;
  bool complete = false;
  bool joints_done = false;
  bool imu_done = false;
  bool buttons_done = false;

  std::vector<JointResult> joints;
  ImuResult imu;
  std::vector<ButtonResult> buttons;
  std::vector<std::string> warnings;

  int MismatchCount() const {
    int n = 0;
    for (const auto& j : joints) n += j.Mismatch() ? 1 : 0;
    n += imu.Mismatch() ? 1 : 0;
    for (const auto& b : buttons) n += b.Mismatch() ? 1 : 0;
    return n;
  }
};

// 성공 시 true. 실패 시 err 에 사유.
bool SaveCalibrationYaml(const std::string& path, const CalibrationData& d, std::string* err);
