#pragma once

#include <cstdint>

#include "peripheral/control_tick/control_tick.hpp"

namespace bb {

// 1 kHz ティック駆動の制御スケジューラ（static class）。
// 内側タスク（1 kHz）を毎ティック、外側タスク（200 Hz）を5ティック毎、
// 診断タスクを 200 ms 毎に呼び出す。
class task_scheduler {
 public:
  task_scheduler() = delete;

  struct hooks {
    void (*inner_cb)() = nullptr;
    void (*outer_cb)() = nullptr;
    void (*diag_cb)() = nullptr;
  };

  static void init();
  [[noreturn]] static void run(const hooks& hooks);

  static uint32_t tick_count() {
    return tick_count_;
  }
  static uint32_t overrun_count() {
    return overrun_count_;
  }
  static uint32_t max_inner_us() {
    return max_inner_us_;
  }
  static uint32_t max_outer_us() {
    return max_outer_us_;
  }

 private:
  static inline uint32_t tick_count_ = 0;
  static inline uint32_t overrun_count_ = 0;
  static inline uint32_t last_diag_ms_ = 0;
  static inline uint32_t max_inner_us_ = 0;
  static inline uint32_t max_outer_us_ = 0;
};

}  // namespace bb
