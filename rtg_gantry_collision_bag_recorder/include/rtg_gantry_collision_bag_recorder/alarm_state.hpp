#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>

namespace gantry_recorder
{
// 管理指定方向的两区域：<16 m 报警，<5.5 m 停止。未知不能当作解除。
class AlarmState
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Level {Unknown, Clear, Slow, Stop};

  explicit AlarmState(std::array<int, 2> areas = {1, 3}) : areas_(areas) {}

  bool update(int area, double distance, double slow_distance, double stop_distance,
    Clock::time_point now = Clock::now())
  {
    if (area != areas_[0] && area != areas_[1]) {
      return false;
    }
    if (!std::isfinite(distance) || distance < 0.0) {
      // 无效距离保留报警状态，但中断解除确认。
      clear_since_.reset();
      return false;
    }
    levels_[area == areas_[0] ? 0 : 1] = distance < stop_distance ? Level::Stop :
      (distance < slow_distance ? Level::Slow : Level::Clear);
    if (!all_clear()) {
      clear_since_.reset();
    } else if (!clear_since_) {
      clear_since_ = now;
    }
    return true;
  }

  bool alarming() const
  {
    for (const auto level : levels_) {
      if (level == Level::Slow || level == Level::Stop) {
        return true;
      }
    }
    return false;
  }

  bool all_clear() const
  {
    for (const auto level : levels_) {
      if (level != Level::Clear) {
        return false;
      }
    }
    return true;
  }

  // 结果按变化发布：明确解除后由定时器推进，不要求连续收到解除消息。
  bool ready_to_close(Clock::time_point now, double hold_seconds) const
  {
    return clear_since_ &&
           std::chrono::duration<double>(now - *clear_since_).count() >= hold_seconds;
  }

  void invalidate()
  {
    levels_.fill(Level::Unknown);
    clear_since_.reset();
  }

private:
  std::array<int, 2> areas_;
  std::array<Level, 2> levels_{Level::Unknown, Level::Unknown};
  std::optional<Clock::time_point> clear_since_;
};
}  // namespace gantry_recorder
