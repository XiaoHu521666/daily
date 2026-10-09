#include "rtg_gantry_collision_bag_recorder/pre_record_buffer.hpp"

#include <cstdlib>
#include <iostream>

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

int main()
{
  using Buffer = gantry_recorder::PreRecordBuffer<int>;
  using namespace std::chrono_literals;
  const auto start = Buffer::Clock::time_point{};
  Buffer buffer(2.0, 100);
  buffer.push(1, start, 20);
  buffer.push(2, start + 1s, 20);
  buffer.expire(start + 2s);
  require(buffer.entries().size() == 2, "保留预录边界上的消息");
  buffer.expire(start + 2001ms);
  require(buffer.entries().size() == 1 && buffer.entries().front().message == 2,
    "只淘汰时间窗口外的消息");
  require(buffer.push(3, start + 2100ms, 80), "恰好达到容量上限仍完整");
  require(!buffer.push(4, start + 2200ms, 30), "容量不足应报告历史截短");
  require(buffer.entries().size() == 1 && buffer.entries().front().message == 4,
    "容量不足优先淘汰最旧数据");
  require(!buffer.push(5, start + 2300ms, 101), "超大单帧不进入缓存");
  require(buffer.entries().front().message == 4, "超大帧不破坏已有缓存");
  buffer.expire(start + 5s);
  require(buffer.entries().empty(), "断流后定时清理过期数据");

  Buffer disabled(0.0, 100);
  disabled.push(1, start, 20);
  require(disabled.entries().empty(), "关闭预录不占用消息缓存");

  Buffer events(2.0, 100);
  events.push(10, start, 10);  // 触发报警的点云先于报警消息到达。
  events.push(11, start + 100ms, 10);
  int first_count = 0;
  for (const auto & entry : events.entries()) {
    require(entry.message == 10 + first_count, "首次事件保留点云与 PLC 回调顺序");
    ++first_count;
  }
  require(first_count == 2, "触发前的输入帧已保留");
  events.push(12, start + 200ms, 10);  // 首次事件触发帧只在补写之后入缓存。
  events.push(13, start + 300ms, 10);
  require(events.entries().size() == 4, "补写不会清空缓存，下一事件仍有完整历史");
  for (int i = 0; i < 10000; ++i) {
    events.push(i, start + 1s + i * 1ms, 10);
    require(events.entries().size() <= 10, "连续录制缓存容量保持有界");
  }
  std::cout << "pre_record_buffer tests passed\n";
}
