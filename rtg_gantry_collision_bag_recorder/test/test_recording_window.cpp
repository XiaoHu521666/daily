#include "rtg_gantry_collision_bag_recorder/recording_window.hpp"
#include <cstdlib>
#include <iostream>
using gantry_recorder::RecordingWindow;
using namespace std::chrono_literals;
void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
int main()
{
  const auto start = RecordingWindow::Clock::time_point{};
  RecordingWindow window;
  require(window.trigger(start, 5, 120), "首次报警开包");
  require(!window.expired(start + 4999ms), "首次报警后五秒前不关闭");
  require(!window.trigger(start + 4s, 5, 120), "第二条报警续期但不新开包");
  require(!window.expired(start + 5s), "第二条报警延长了原截止时间");
  require(window.expired(start + 9s), "最后报警后五秒结束");
  require(window.trigger(start + 10s, 5, 120), "无需解除消息即可开始下一录制");
  require(!window.max_duration_reached(start + 129s), "新包重新计算最大时长");

  RecordingWindow continuous;
  require(continuous.trigger(start, 5, 120), "连续消息首次触发");
  for (int seconds = 1; seconds <= 7200; ++seconds) {
    const auto now = start + std::chrono::seconds(seconds);
    const bool boundary = seconds % 120 == 0;
    require(continuous.max_duration_reached(now) == boundary,
      "旧包每120秒达到上限，调用方先关闭旧包");
    require(continuous.trigger(now, 5, 120) == boundary,
      "到上限后的当前报警直接触发新包，其余消息仅续期");
    require(!continuous.expired(now), "每条消息刷新五秒截止时间");
    require(!continuous.max_duration_reached(now), "新包最大时长重新起算");
  }
  require(!continuous.expired(start + 7204999ms), "新包保留报警后五秒");
  require(continuous.expired(start + 7205s), "新包无报警满五秒后关闭");
  require(continuous.trigger(start + 7205s, 5, 120), "静默五秒后边界允许新包");
  require(!continuous.max_duration_reached(start + 7324999ms), "新包上限重新起算");
  require(continuous.max_duration_reached(start + 7325s), "新包120秒边界");
  std::cout << "recording_window tests passed\n";
}
