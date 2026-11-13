#include "omni_drive_test/omni_drive_test.hpp"

#include <etl/algorithm.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "device/estop_input/estop_input.hpp"
#include "feature/wheel_speed_meter/wheel_speed_meter.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"
#include "test_runner/test_runner.hpp"

namespace bb {

namespace {
// テスト固有サーボの比例ゲイン [duty/(rad/s)]。
constexpr float DUTY_P_GAIN = 0.03F;
// 不感帯補償（指令非ゼロ時に符号方向へ加算するデューティ）。
// motor_deadzone_test の結果で調整する。
constexpr float DEADZONE_COMP = 0.08F;
// 自動停止 [tick]（60 s）。
constexpr uint32_t MAX_TICKS = 60000U;
// 輪速計の更新間隔（製品の外側周期と同じ 5 ms）。
constexpr uint32_t METER_PERIOD_TICKS = 5U;
// ログ間隔（10 Hz）。
constexpr uint32_t LOG_PERIOD_TICKS = 100U;
}  // namespace

void omni_drive_test::init(const config& cfg) {
  p_ = cfg.params;
  for (int i = 0; i < 3; ++i) motors_[i] = cfg.motors[i];
  duty_ff_ = p_->motor.back_emf_constant / p_->driver.supply_voltage;
}

bool omni_drive_test::start_cb(int argc, char** argv) {
  if (argc < 3) return false;
  const float vx = strtof(argv[0], nullptr);
  const float vy = strtof(argv[1], nullptr);
  const float yaw = strtof(argv[2], nullptr);
  const float vmax = p_->controller.max_speed;
  const float wmax = p_->controller.max_yaw_rate;
  cmd_ = {etl::clamp(vx, -vmax, vmax), etl::clamp(vy, -vmax, vmax),
          etl::clamp(yaw, -wmax, wmax)};
  wheel_cmd_ = mul(p_->lqi.wheel_speed_from_planar_velocity, cmd_);
  tick_ = 0;
  meter_phase_ = 0;

  char line[192];
  const int n = snprintf(
      line, sizeof(line),
      "omni: cmd=[%.3f %.3f %.3f] wheel_cmd=[%.2f %.2f %.2f] rad/s; "
      "HOLD the chassis, wheels unloaded; auto-stop at 60 s\r\n",
      static_cast<double>(cmd_[0]), static_cast<double>(cmd_[1]),
      static_cast<double>(cmd_[2]), static_cast<double>(wheel_cmd_[0]),
      static_cast<double>(wheel_cmd_[1]), static_cast<double>(wheel_cmd_[2]));
  debug_uart::write(line, static_cast<uint32_t>(n));
  return true;
}

void omni_drive_test::update_cb() {
  // 1 kHz 呼出。
  if (estop_input::requested()) {
    for (auto* m : motors_) m->set_duty(0.0F);
    debug_uart::write_line("omni: E-stop -> stopped");
    test_runner::request_stop();
    return;
  }
  if (++tick_ >= MAX_TICKS) {
    for (auto* m : motors_) m->set_duty(0.0F);
    debug_uart::write_line("omni: 60 s auto-stop");
    test_runner::request_stop();
    return;
  }
  if (++meter_phase_ >= METER_PERIOD_TICKS) {
    meter_phase_ = 0;
    (void)wheel_speed_meter::update();
  }

  // 輪速サーボ: duty = FF*wcmd + P*(wcmd - w) + 不感帯補償。
  const vec3 w = wheel_speed_meter::rate();
  vec3 duty{};
  for (int i = 0; i < 3; ++i) {
    const float err = wheel_cmd_[i] - w[i];
    float d = duty_ff_ * wheel_cmd_[i] + DUTY_P_GAIN * err;
    if (wheel_cmd_[i] != 0.0F) {
      d += copysignf(DEADZONE_COMP, wheel_cmd_[i]);
    }
    duty[i] =
        etl::clamp(d, -p_->driver.max_duty_cycle, p_->driver.max_duty_cycle);
    motors_[i]->set_duty(duty[i]);
  }

  if ((tick_ % LOG_PERIOD_TICKS) == 0U) {
    char line[200];
    const int n = snprintf(
        line, sizeof(line),
        "omni t=%lu wcmd=[%.2f %.2f %.2f] w=[%.2f %.2f %.2f] "
        "duty=[%.3f %.3f %.3f]\r\n",
        static_cast<unsigned long>(timebase::millis()),
        static_cast<double>(wheel_cmd_[0]), static_cast<double>(wheel_cmd_[1]),
        static_cast<double>(wheel_cmd_[2]), static_cast<double>(w[0]),
        static_cast<double>(w[1]), static_cast<double>(w[2]),
        static_cast<double>(duty[0]), static_cast<double>(duty[1]),
        static_cast<double>(duty[2]));
    debug_uart::write(line, static_cast<uint32_t>(n));
  }
}

void omni_drive_test::stop_cb() {
  for (auto* m : motors_) m->set_duty(0.0F);
  debug_uart::write_line("omni: stopped");
}

}  // namespace bb
