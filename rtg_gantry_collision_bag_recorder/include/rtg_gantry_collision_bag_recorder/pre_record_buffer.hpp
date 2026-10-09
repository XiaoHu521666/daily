#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <utility>

namespace gantry_recorder
{
// 仅管理缓存寿命和负载容量；消息格式及写盘由节点负责。
template<typename Message>
class PreRecordBuffer
{
public:
  using Clock = std::chrono::steady_clock;
  struct Entry
  {
    Message message;
    Clock::time_point received;
    std::size_t bytes;
  };

  PreRecordBuffer(double seconds = 2.0, std::size_t max_bytes = 128 * 1024 * 1024)
  : seconds_(seconds), max_bytes_(max_bytes) {}

  bool push(Message message, Clock::time_point now, std::size_t bytes)
  {
    expire(now);
    if (seconds_ == 0.0) {
      return true;
    }
    if (bytes > max_bytes_) {
      return false;
    }
    bool complete = true;
    while (!entries_.empty() && bytes_ > max_bytes_ - bytes) {
      pop_front();
      complete = false;
    }
    entries_.push_back({std::move(message), now, bytes});
    bytes_ += bytes;
    return complete;
  }

  void expire(Clock::time_point now)
  {
    while (!entries_.empty() &&
      std::chrono::duration<double>(now - entries_.front().received).count() > seconds_)
    {
      pop_front();
    }
  }

  const std::deque<Entry> & entries() const {return entries_;}

private:
  void pop_front()
  {
    bytes_ -= entries_.front().bytes;
    entries_.pop_front();
  }

  double seconds_;
  std::size_t max_bytes_;
  std::size_t bytes_{0};
  std::deque<Entry> entries_;
};
}  // namespace gantry_recorder
