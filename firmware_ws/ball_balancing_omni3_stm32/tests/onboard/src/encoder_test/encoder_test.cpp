#include "encoder_test/encoder_test.hpp"

#include <etl/algorithm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device/estop_input/estop_input.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"
#include "test_runner/test_runner.hpp"

namespace bb {

namespace {
constexpr float UPDATE_DT = 0.1F;      // period=100 tick = 100 ms
constexpr float DEFAULT_SPIN_DUTY = 0.40F;  // 不感帯(~0.28)を確実に超える既定値
constexpr uint32_t SPIN_PHASE_TICKS = 30U;  // 3 s / 100 ms

void apply_duty(mdd3a_motor* const* motors, float duty) {
  for (int i = 0; i < 3; ++i) motors[i]->set_duty(duty);
}
}  // namespace

void encoder_test::init(const config& cfg) {
  for (int i = 0; i < 3; ++i) {
    counters_[i] = cfg.counters[i];
    encoders_[i] = cfg.encoders[i];
    motors_[i] = cfg.motors[i];
  }
}

bool encoder_test::start_cb(int argc, char** argv) {
  spin_ = (argc >= 1) && (strcmp(argv[0], "spin") == 0);
  if (argc >= 1 && !spin_) return false;
  spin_phase_ = 0;
  spin_duty_ = DEFAULT_SPIN_DUTY;
  if (argc >= 2) {
    spin_duty_ = etl::clamp(strtof(argv[1], nullptr), 0.05F, 0.8F);
  }
  if (spin_) {
    char line[96];
    const int n =
        snprintf(line, sizeof(line),
                 "enc spin: duty +%.2f (3 s) then -%.2f (3 s); "
                 "expect rate sign == duty sign\r\n",
                 static_cast<double>(spin_duty_),
                 static_cast<double>(spin_duty_));
    debug_uart::write(line, static_cast<uint32_t>(n));
  } else {
    debug_uart::write_line(
        "enc test: rotate each wheel by hand in the expected positive "
        "direction; displacement/rate should be positive");
  }
  return true;
}

void encoder_test::update_cb() {
  // spin 中は E-stop で即停止する。
  if (spin_ && estop_input::requested()) {
    apply_duty(motors_, 0.0F);
    debug_uart::write_line("enc spin: E-stop -> stopped");
    test_runner::request_stop();
    return;
  }

  wheel_encoder::sample s[3];
  for (int i = 0; i < 3; ++i) {
    s[i] = encoders_[i]->update(UPDATE_DT);
  }

  float duty = 0.0F;
  if (spin_) {
    // 0-30 更新: +duty、30-60 更新: -duty、以降: 終了。
    if (spin_phase_ < SPIN_PHASE_TICKS) {
      duty = spin_duty_;
    } else if (spin_phase_ < 2U * SPIN_PHASE_TICKS) {
      duty = -spin_duty_;
    } else {
      apply_duty(motors_, 0.0F);
      debug_uart::write_line("enc spin: done");
      test_runner::request_stop();
      return;
    }
    apply_duty(motors_, duty);
    ++spin_phase_;
  }

  char line[200];
  const int n = snprintf(
      line, sizeof(line),
      "enc t=%lu duty=%.2f cnt=[%lld %lld %lld] disp=[%.3f %.3f %.3f] "
      "rate=[%.2f %.2f %.2f]\r\n",
      static_cast<unsigned long>(timebase::millis()), static_cast<double>(duty),
      static_cast<long long>(counters_[0]->count()),
      static_cast<long long>(counters_[1]->count()),
      static_cast<long long>(counters_[2]->count()),
      static_cast<double>(s[0].displacement),
      static_cast<double>(s[1].displacement),
      static_cast<double>(s[2].displacement), static_cast<double>(s[0].rate),
      static_cast<double>(s[1].rate), static_cast<double>(s[2].rate));
  debug_uart::write(line, static_cast<uint32_t>(n));
}

void encoder_test::stop_cb() {
  apply_duty(motors_, 0.0F);
  debug_uart::write_line("enc test: stopped");
}

}  // namespace bb
