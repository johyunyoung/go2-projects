// READ-ONLY: no publishers
#include "state_cache.hpp"

#include <functional>

using unitree::robot::ChannelSubscriber;

// ---------------- RateMeter ----------------

void RateMeter::Stamp(Clock::time_point t) {
  stamps_[head_] = t;
  head_ = (head_ + 1) % kCap;
  if (count_ < kCap) ++count_;
}

double RateMeter::Hz(Clock::time_point now) const {
  // 최근 1초 안에 들어온 수신 시각만 사용
  const auto window = std::chrono::seconds(1);
  size_t n = 0;
  Clock::time_point newest{}, oldest{};
  for (size_t i = 0; i < count_; ++i) {
    const size_t idx = (head_ + kCap - 1 - i) % kCap;  // 최신 → 과거
    const auto t = stamps_[idx];
    if (now - t > window) break;
    if (n == 0) newest = t;
    oldest = t;
    ++n;
  }
  if (n < 2) return 0.0;
  const double span = std::chrono::duration<double>(newest - oldest).count();
  return span > 0.0 ? static_cast<double>(n - 1) / span : 0.0;
}

// ---------------- Snapshot ----------------

double Snapshot::LowAge() const {
  if (!has_low) return -1.0;
  return std::chrono::duration<double>(now - low_time).count();
}

double Snapshot::RemoteAge() const {
  if (!has_remote) return -1.0;
  return std::chrono::duration<double>(now - remote_time).count();
}

// ---------------- StateCache ----------------

void StateCache::Start() {
  // 구독자만 생성한다. (발행 채널 없음)
  low_sub_.reset(new ChannelSubscriber<LowState>(kTopicLowState));
  low_sub_->InitChannel(std::bind(&StateCache::OnLowState, this, std::placeholders::_1), 1);

  remote_sub_.reset(new ChannelSubscriber<RemoteState>(kTopicRemote));
  remote_sub_->InitChannel(std::bind(&StateCache::OnRemote, this, std::placeholders::_1), 1);
}

void StateCache::Stop() {
  if (low_sub_) low_sub_->CloseChannel();
  if (remote_sub_) remote_sub_->CloseChannel();
  low_sub_.reset();
  remote_sub_.reset();
}

// 콜백: 복사 + 시각 기록만 하고 반환
void StateCache::OnLowState(const void* msg) {
  const auto t = Clock::now();
  std::lock_guard<std::mutex> lk(mtx_);
  low_ = *static_cast<const LowState*>(msg);
  low_time_ = t;
  ++low_count_;
  has_low_ = true;
  low_rate_.Stamp(t);
}

void StateCache::OnRemote(const void* msg) {
  const auto t = Clock::now();
  std::lock_guard<std::mutex> lk(mtx_);
  remote_ = *static_cast<const RemoteState*>(msg);
  remote_time_ = t;
  ++remote_count_;
  has_remote_ = true;
  remote_rate_.Stamp(t);
}

Snapshot StateCache::Get() const {
  Snapshot s;
  std::lock_guard<std::mutex> lk(mtx_);
  s.now = Clock::now();
  s.has_low = has_low_;
  s.low = low_;
  s.low_time = low_time_;
  s.low_count = low_count_;
  s.low_hz = low_rate_.Hz(s.now);
  s.has_remote = has_remote_;
  s.remote = remote_;
  s.remote_time = remote_time_;
  s.remote_count = remote_count_;
  s.remote_hz = remote_rate_.Hz(s.now);
  return s;
}
