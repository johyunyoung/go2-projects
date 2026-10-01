// READ-ONLY: no publishers
//
// 검증 대상 "가설" 모음. 여기 적힌 값은 확인된 사실이 아니며,
// go2_inspect 의 안내형 모드로 실제 로봇에서 측정해 확인해야 한다.
#pragma once

#include <array>
#include <cstddef>

namespace hyp {

constexpr int kNumJoints = 12;      // Go2 다리 관절 수
constexpr int kNumMotorSlots = 20;  // LowState_::motor_state 배열 크기 (헤더 확인값)

// [가설] motor_state 인덱스 → 관절 이름
constexpr std::array<const char*, kNumJoints> kJointName = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
};

// [가설] 쿼터니언 순서. unitree 문서 기준 (w, x, y, z) 로 알려져 있으나 미검증.
constexpr const char* kQuaternionOrder = "wxyz";

// [참고 가설] 리모컨 keys 비트 번호.
// 출처: unitree_sdk2 example/wireless_controller/advanced_gamepad.hpp 의 xKeySwitchUnion
// 비트필드 선언 순서 (R1, L1, start, select, R2, L2, F1, F2, A, B, X, Y, up, right, down, left).
// GCC/little-endian 에서 첫 필드가 bit 0 이라고 가정한 값이다.
struct ButtonHyp {
  const char* name;
  int bit;
};

constexpr std::array<ButtonHyp, 16> kButtonBits = {{
    {"R1", 0}, {"L1", 1}, {"start", 2}, {"select", 3},
    {"R2", 4}, {"L2", 5}, {"F1", 6},    {"F2", 7},
    {"A", 8},  {"B", 9},  {"X", 10},    {"Y", 11},
    {"up", 12}, {"right", 13}, {"down", 14}, {"left", 15},
}};

// 안내형 모드에서 확인할 버튼 순서
constexpr std::array<const char*, 14> kGuidedButtons = {
    "A", "B", "X", "Y", "L1", "L2", "R1", "R2",
    "start", "select", "up", "down", "left", "right",
};

inline int ButtonBitHyp(const char* name) {
  for (const auto& b : kButtonBits) {
    const char* a = b.name;
    const char* c = name;
    while (*a && *a == *c) { ++a; ++c; }
    if (*a == '\0' && *c == '\0') return b.bit;
  }
  return -1;
}

inline const char* ButtonNameForBit(int bit) {
  for (const auto& b : kButtonBits)
    if (b.bit == bit) return b.name;
  return "?";
}

// [참고 가설] lowstate.wireless_remote[40] 원시 바이트 중 [2],[3] 이 keys 와 같은
// 16비트 버튼 필드라고 가정 (unitree 리모컨 원시 패킷 구조 기준, 미검증).
constexpr int kRawRemoteKeyByteLo = 2;
constexpr int kRawRemoteKeyByteHi = 3;

}  // namespace hyp
