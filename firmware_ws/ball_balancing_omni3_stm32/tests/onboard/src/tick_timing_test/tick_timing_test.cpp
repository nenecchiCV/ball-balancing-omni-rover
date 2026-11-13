#include "tick_timing_test/tick_timing_test.hpp"

#include <etl/algorithm.h>
#include <stdio.h>

#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"

namespace bb {

namespace {
// ティック間隔がこれを超えたら欠落ありとみなす [us]。
constexpr uint32_t MISS_THRESHOLD_US = 1500U;
// 1 秒ごとの統計報告。
constexpr uint32_t REPORT_TICKS = 1000U;
}  // namespace

void tick_timing_test::init() {}

bool tick_timing_test::start_cb(int argc, char** argv) {
  (void)argc;
  (void)argv;
  debug_uart::write_line(
      "tick test: period stats every 1 s (expected ~1000 us)");
  prev_valid_ = false;
  skip_next_ = false;
  n_ = 0;
  sum_us_ = 0;
  min_us_ = UINT32_MAX;
  max_us_ = 0;
  missed_ = 0;
  return true;
}

void tick_timing_test::update_cb() {
  // 1 kHz（period=1 tick）呼出。
  const uint32_t now = timebase::micros();
  const bool sample = prev_valid_ && !skip_next_;
  if (sample) {
    const uint32_t dt = now - prev_us_;
    sum_us_ += dt;
    min_us_ = etl::min(dt, min_us_);
    max_us_ = etl::max(dt, max_us_);
    if (dt > MISS_THRESHOLD_US) {
      missed_ += (dt / 1000U) - 1U;
    }
    ++n_;
  }
  skip_next_ = false;
  prev_us_ = now;
  prev_valid_ = true;
  // 報告直後のサンプルは printk のブロックで伸びるため捨てる。
  if (n_ >= REPORT_TICKS) {
    report();
    skip_next_ = true;
    n_ = 0;
    sum_us_ = 0;
    min_us_ = UINT32_MAX;
    max_us_ = 0;
    missed_ = 0;
  }
}

void tick_timing_test::report() {
  const uint32_t mean = (n_ > 0U) ? (sum_us_ / n_) : 0U;
  char line[128];
  const int len = snprintf(
      line, sizeof(line),
      "tick n=%lu mean_us=%lu min_us=%lu max_us=%lu "
      "missed=%lu\r\n",
      static_cast<unsigned long>(n_), static_cast<unsigned long>(mean),
      static_cast<unsigned long>(min_us_), static_cast<unsigned long>(max_us_),
      static_cast<unsigned long>(missed_));
  debug_uart::write(line, static_cast<uint32_t>(len));
}

void tick_timing_test::stop_cb() {
  debug_uart::write_line("tick test: stopped");
}

}  // namespace bb
