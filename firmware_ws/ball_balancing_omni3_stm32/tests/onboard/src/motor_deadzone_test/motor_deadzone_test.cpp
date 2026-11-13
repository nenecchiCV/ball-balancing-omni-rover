#include "motor_deadzone_test/motor_deadzone_test.hpp"

#include <etl/algorithm.h>
#include <stdio.h>
#include <stdlib.h>

#include "device/estop_input/estop_input.hpp"
#include "feature/wheel_speed_meter/wheel_speed_meter.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"
#include "test_runner/test_runner.hpp"

namespace bb {

namespace {
constexpr uint32_t METER_PERIOD_TICKS = 5U;
constexpr uint32_t LOG_PERIOD_TICKS = 50U;  // 20 Hz CSV
}  // namespace

void motor_deadzone_test::init(const config& cfg) {
  for (int i = 0; i < 3; ++i) motors_[i] = cfg.motors[i];
}

bool motor_deadzone_test::start_cb(int argc, char** argv) {
  dir_ = 1.0F;
  max_duty_ = 0.25F;
  ramp_s_ = 15.0F;
  if (argc >= 1) dir_ = (strtof(argv[0], nullptr) >= 0.0F) ? 1.0F : -1.0F;
  if (argc >= 2) {
    max_duty_ = etl::clamp(strtof(argv[1], nullptr), 0.01F, 0.8F);
  }
  if (argc >= 3) {
    ramp_s_ = etl::clamp(strtof(argv[2], nullptr), 1.0F, 120.0F);
  }
  tick_ = 0;
  meter_phase_ = 0;

  char line[160];
  const int n =
      snprintf(line, sizeof(line),
               "deadzone: dir=%+.0f max=%.2f ramp=%.1f s; wheels unloaded, "
               "hold the chassis\ndz,t_s,duty,w0,w1,w2\r\n",
               static_cast<double>(dir_), static_cast<double>(max_duty_),
               static_cast<double>(ramp_s_));
  debug_uart::write(line, static_cast<uint32_t>(n));
  return true;
}

void motor_deadzone_test::update_cb() {
  // 1 kHz 呼出。0→dir*max（ramp_s）→0（ramp_s）の三角掃引。
  if (estop_input::requested()) {
    for (auto* m : motors_) m->set_duty(0.0F);
    debug_uart::write_line("dz,E-stop");
    test_runner::request_stop();
    return;
  }

  const float t = static_cast<float>(tick_) * 1e-3F;
  const float total = 2.0F * ramp_s_;
  if (t >= total) {
    for (auto* m : motors_) m->set_duty(0.0F);
    debug_uart::write_line("dz,done");
    test_runner::request_stop();
    return;
  }
  const float phase = (t < ramp_s_) ? (t / ramp_s_) : (2.0F - t / ramp_s_);
  const float duty = dir_ * max_duty_ * phase;

  for (auto* m : motors_) m->set_duty(duty);
  if (++meter_phase_ >= METER_PERIOD_TICKS) {
    meter_phase_ = 0;
    (void)wheel_speed_meter::update();
  }
  ++tick_;

  if ((tick_ % LOG_PERIOD_TICKS) == 0U) {
    const vec3 w = wheel_speed_meter::rate();
    char line[160];
    const int n =
        snprintf(line, sizeof(line), "dz,%.2f,%.4f,%.3f,%.3f,%.3f\r\n",
                 static_cast<double>(t), static_cast<double>(duty),
                 static_cast<double>(w[0]), static_cast<double>(w[1]),
                 static_cast<double>(w[2]));
    debug_uart::write(line, static_cast<uint32_t>(n));
  }
}

void motor_deadzone_test::stop_cb() {
  for (auto* m : motors_) m->set_duty(0.0F);
  debug_uart::write_line("deadzone: stopped");
}

}  // namespace bb
