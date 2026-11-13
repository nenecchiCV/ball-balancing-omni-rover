#include "manager/task_scheduler/task_scheduler.hpp"

#include <etl/algorithm.h>

#include "peripheral/timebase/timebase.hpp"

namespace bb {

void task_scheduler::init() {
  tick_count_ = 0;
  last_diag_ms_ = 0;
  overrun_count_ = 0;
  max_inner_us_ = 0;
  max_outer_us_ = 0;
}

[[noreturn]] void task_scheduler::run(const hooks& hooks) {
  for (;;) {
    control_tick::wait();
    if (k_sem_count_get(control_tick::sem()) > 0U) {
      // 処理が間に合わずティックが溜まった場合はオーバーランとして記録し、
      // 最新 1 ティック分のみ実行する（追いつき実行は位相歪みを招くため）。
      k_sem_reset(control_tick::sem());
      ++overrun_count_;
    }
    ++tick_count_;
    if ((tick_count_ % 5U) == 0U && hooks.outer_cb != nullptr) {
      const uint32_t c0 = timebase::cycles();
      hooks.outer_cb();
      max_outer_us_ = etl::max(max_outer_us_,
                               timebase::cycles_to_us(timebase::cycles() - c0));
    }
    if (hooks.inner_cb != nullptr) {
      const uint32_t c0 = timebase::cycles();
      hooks.inner_cb();
      max_inner_us_ = etl::max(max_inner_us_,
                               timebase::cycles_to_us(timebase::cycles() - c0));
    }
    const uint32_t now_ms = timebase::millis();
    if (hooks.diag_cb != nullptr && now_ms - last_diag_ms_ >= 200U) {
      last_diag_ms_ = now_ms;
      hooks.diag_cb();
    }
  }
}

}  // namespace bb
