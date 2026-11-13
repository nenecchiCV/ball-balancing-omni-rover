#pragma once

#include <cstdint>

namespace bb {

// 制御ティック計時検証テスト（static class）。
// update_cb の実呼出間隔を micros() で計測し、1 秒ごとに
// 平均・最小・最大周期と欠落ティック数を表示する。
class tick_timing_test {
 public:
  tick_timing_test() = delete;

  static void init();

  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

 private:
  static void report();

  static inline uint32_t prev_us_ = 0;
  static inline bool prev_valid_ = false;
  static inline bool skip_next_ = false;
  static inline uint32_t n_ = 0;
  static inline uint32_t sum_us_ = 0;
  static inline uint32_t min_us_ = 0;
  static inline uint32_t max_us_ = 0;
  static inline uint32_t missed_ = 0;
};

}  // namespace bb
