#include "rtg_gantry_collision_bag_recorder/alarm_state.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using gantry_recorder::AlarmState;

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

int main()
{
  AlarmState state({1, 3});
  require(!state.alarming() && !state.all_clear(), "未知状态不能判定解除");
  state.update(1, 10, 16, 5.5);
  require(state.alarming(), "减速区应开始事件");
  state.update(3, 3, 16, 5.5);
  state.update(1, 20, 16, 5.5);
  require(state.alarming(), "一区解除不能覆盖另一区报警");
  state.update(3, 16, 16, 5.5);
  require(state.all_clear(), "本组两区解除即结束，不等另一组");
  state.update(3, 5.5, 16, 5.5);
  require(state.alarming(), "5.5米仍处于减速报警区");
  require(!state.update(3, std::numeric_limits<double>::quiet_NaN(), 16, 5.5), "拒绝NaN");
  require(!state.update(3, -1, 16, 5.5), "拒绝负距离");
  require(!state.update(3, std::numeric_limits<double>::infinity(), 16, 5.5), "拒绝无穷距离");
  require(!state.update(7, 2, 16, 5.5), "侧方其他区域不纳入四雷达事件");
  require(state.alarming(), "无效数据不能清除已有报警");
  state.invalidate();
  require(!state.all_clear(), "断线必须使状态失效");
  for (int area = 1; area <= 4; ++area) {
    state.update(area, 9999, 16, 5.5);
  }
  require(state.all_clear(), "恢复本组结果后可结束事件");
  state.update(3, 0, 16, 5.5);
  require(state.alarming(), "近距离再次报警能够开始下一事件");
  AlarmState right({2, 4});
  require(!right.update(1, 1, 16, 5.5), "左侧报警不能启动右侧");
  require(!right.alarming(), "右侧保持未报警");
  right.update(2, 10, 16, 5.5);
  require(right.alarming() && state.alarming(), "两侧可同时独立报警");
  right.update(2, 20, 16, 5.5);
  require(!right.all_clear(), "本组另一未收到的区域不能当解除");
  right.update(4, 20, 16, 5.5);
  require(right.all_clear() && state.alarming(), "右侧解除不影响左侧事件");

  // 用可控单调时间验证解除防抖，不依赖真实等待或 ROS。
  using namespace std::chrono_literals;
  const auto start = AlarmState::Clock::time_point{};
  AlarmState debounce({1, 3});
  debounce.update(3, 20, 16, 5.5, start);
  debounce.update(1, 10, 16, 5.5, start);
  require(debounce.alarming(), "单帧报警仍立即触发，不能丢失触发证据");
  require(!debounce.ready_to_close(start + 10s, 3), "仍在报警不能超时关包");
  debounce.update(1, 20, 16, 5.5, start + 100ms);
  require(!debounce.ready_to_close(start + 200ms, 3), "下一帧解除不能立即关包");
  for (int i = 1; i <= 100; ++i) {
    const auto now = start + i * 200ms;
    debounce.update(1, 15.9, 16, 5.5, now);
    debounce.update(1, 16, 16, 5.5, now + 100ms);
    require(!debounce.ready_to_close(now + 199ms, 3), "反复抖动应始终合并在一个事件内");
  }
  const auto clear = start + 20100ms;
  debounce.update(3, 20, 16, 5.5, clear + 1s);
  require(!debounce.ready_to_close(clear + 2999ms, 3), "等待不足三秒不能关闭");
  require(debounce.ready_to_close(clear + 3s, 3), "重复正常消息不重置计时，边界到期可关闭");
  require(debounce.ready_to_close(clear, 0), "零等待兼容原有立即关闭行为");
  debounce.update(3, 2, 16, 5.5, clear + 3s);
  require(!debounce.ready_to_close(clear + 10s, 3), "本组另一区域报警也应取消等待");
  debounce.update(3, 20, 16, 5.5, clear + 4s);
  debounce.update(2, 1, 16, 5.5, clear + 5s);
  require(debounce.ready_to_close(clear + 7s, 3), "另一组报警不能影响本组关闭");
  debounce.update(1, std::numeric_limits<double>::quiet_NaN(), 16, 5.5, clear + 8s);
  require(!debounce.ready_to_close(clear + 20s, 3), "无效距离必须取消待关闭计时");
  debounce.update(1, 20, 16, 5.5, clear + 21s);
  require(!debounce.ready_to_close(clear + 23s, 3), "有效结果恢复后重新等待");
  require(debounce.ready_to_close(clear + 24s, 3), "按变化发布无需后续消息也可到期");
  debounce.invalidate();
  require(!debounce.ready_to_close(clear + 30s, 3), "发布者断线取消待关闭计时");
  debounce.update(1, 20, 16, 5.5, clear + 31s);
  require(!debounce.ready_to_close(clear + 40s, 3), "断线恢复只收到一区正常不能关闭");
  debounce.update(3, 20, 16, 5.5, clear + 41s);
  require(debounce.ready_to_close(clear + 44s, 3), "恢复两区结果并保持三秒才可关闭");
  std::cout << "alarm_state tests passed\n";
}
