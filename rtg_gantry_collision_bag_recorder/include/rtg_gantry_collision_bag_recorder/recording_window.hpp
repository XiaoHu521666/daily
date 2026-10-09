#pragma once

#include <chrono>

namespace gantry_recorder
{
// 报警消息刷新空闲截止时间，首次触发固定最大截止时间。
class RecordingWindow
{
public:
  using Clock = std::chrono::steady_clock;

  bool trigger(Clock::time_point now, double seconds, double max_seconds = 120.0)
  {
    // 调用方先关闭到期的旧包；当前报警可直接开始新包。
    if (!armed_ && (now >= deadline_ || now >= max_deadline_)) {
      armed_ = true;
    }
    deadline_ = now + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
    if (!armed_) {
      return false;
    }
    armed_ = false;
    max_deadline_ = now + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(max_seconds));
    return true;
  }

  bool expired(Clock::time_point now) const {return !armed_ && now >= deadline_;}
  bool max_duration_reached(Clock::time_point now) const
  {
    return !armed_ && now >= max_deadline_;
  }

private:
  bool armed_{true};
  Clock::time_point deadline_{};
  Clock::time_point max_deadline_{};
};
}  // namespace gantry_recorder
