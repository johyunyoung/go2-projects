// READ-ONLY: no publishers
//
// rt/lowstate, rt/wirelesscontroller 구독 전용 캐시.
// 수신 콜백은 최신 메시지 복사 + 수신 시각 기록만 하고 즉시 반환한다 (mutex 보호).
// 이 파일과 프로젝트 전체에서 ChannelSubscriber 외의 채널 객체는 만들지 않는다.
#pragma once

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>

using Clock = std::chrono::steady_clock;
using LowState = unitree_go::msg::dds_::LowState_;
using RemoteState = unitree_go::msg::dds_::WirelessController_;

constexpr const char* kTopicLowState = "rt/lowstate";
constexpr const char* kTopicRemote = "rt/wirelesscontroller";

// 수신 시각 링버퍼로 최근 1초 수신 주기 추정
class RateMeter {
 public:
  void Stamp(Clock::time_point t);
  double Hz(Clock::time_point now) const;

 private:
  static constexpr size_t kCap = 2048;
  std::array<Clock::time_point, kCap> stamps_{};
  size_t head_ = 0;   // 다음에 쓸 위치
  size_t count_ = 0;  // 저장된 개수 (최대 kCap)
};

struct Snapshot {
  Clock::time_point now;

  bool has_low = false;
  LowState low;
  Clock::time_point low_time;
  uint64_t low_count = 0;
  double low_hz = 0.0;

  bool has_remote = false;
  RemoteState remote;
  Clock::time_point remote_time;
  uint64_t remote_count = 0;
  double remote_hz = 0.0;

  // 마지막 수신 후 경과 시간 [s]. 수신 이력이 없으면 음수.
  double LowAge() const;
  double RemoteAge() const;
};

class StateCache {
 public:
  // ChannelFactory::Instance()->Init() 이후에 호출
  void Start();
  void Stop();
  Snapshot Get() const;

 private:
  void OnLowState(const void* msg);
  void OnRemote(const void* msg);

  mutable std::mutex mtx_;
  bool has_low_ = false;
  LowState low_;
  Clock::time_point low_time_;
  uint64_t low_count_ = 0;
  RateMeter low_rate_;

  bool has_remote_ = false;
  RemoteState remote_;
  Clock::time_point remote_time_;
  uint64_t remote_count_ = 0;
  RateMeter remote_rate_;

  unitree::robot::ChannelSubscriberPtr<LowState> low_sub_;
  unitree::robot::ChannelSubscriberPtr<RemoteState> remote_sub_;
};
