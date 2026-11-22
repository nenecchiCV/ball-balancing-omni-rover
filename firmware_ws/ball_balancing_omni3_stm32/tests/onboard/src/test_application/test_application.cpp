#include "test_application/test_application.hpp"

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "device/estop_input/estop_input.hpp"
#include "device/lsm6dso/lsm6dso.hpp"
#include "device/status_led/status_led.hpp"
#include "encoder_test/encoder_test.hpp"
#include "estop_test/estop_test.hpp"
#include "feature/state_estimator/state_estimator.hpp"
#include "feature/wheel_speed_meter/wheel_speed_meter.hpp"
#include "imu_attitude_test/imu_attitude_test.hpp"
#include "motor_deadzone_test/motor_deadzone_test.hpp"
#include "omni_drive_test/omni_drive_test.hpp"
#include "peripheral/control_tick/control_tick.hpp"
#include "test_runner/test_runner.hpp"
#include "tick_timing_test/tick_timing_test.hpp"

namespace bb {

namespace {
#define PINS_NODE DT_NODELABEL(bb_pins)
}  // namespace

bool test_application::init() {
  p_ = &get_params();

  // peripheral（robot_application と同配線）
  led_pin_.init({GPIO_DT_SPEC_GET(PINS_NODE, led_gpios), true});
  estop_ext_pin_.init({GPIO_DT_SPEC_GET(PINS_NODE, estop_gpios)});
  estop_btn_pin_.init({GPIO_DT_SPEC_GET(PINS_NODE, estop_btn_gpios)});
  dir_pins_[0].init({GPIO_DT_SPEC_GET(PINS_NODE, dir0_gpios), true});
  dir_pins_[1].init({GPIO_DT_SPEC_GET(PINS_NODE, dir1_gpios), true});
  dir_pins_[2].init({GPIO_DT_SPEC_GET(PINS_NODE, dir2_gpios), true});
  pwm_ch_[0].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 0)});
  pwm_ch_[1].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 1)});
  pwm_ch_[2].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 2)});
  enc_count_[0].init(
      {reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers2)))});
  enc_count_[1].init(
      {reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers1)))});
  enc_count_[2].init(
      {reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers3)))});

  // device
  const bool imu_ok =
      lsm6dso::init({DEVICE_DT_GET(DT_ALIAS(imu0)), mat_identity3()});
  motors_[0].init({&pwm_ch_[0], &dir_pins_[0]});
  motors_[1].init({&pwm_ch_[1], &dir_pins_[1]});
  motors_[2].init({&pwm_ch_[2], &dir_pins_[2]});
  encoders_[0].init({&enc_count_[0], p_->encoder.quadrature_factor,
                     p_->encoder.counts_per_rev});
  encoders_[1].init({&enc_count_[1], p_->encoder.quadrature_factor,
                     p_->encoder.counts_per_rev});
  encoders_[2].init({&enc_count_[2], p_->encoder.quadrature_factor,
                     p_->encoder.counts_per_rev});
  (void)estop_input::init({&estop_ext_pin_, &estop_btn_pin_});
  status_led::init({&led_pin_});
  // 起動中の生存確認ハートビート: 無条件に 1 秒周期点滅とする。
  // IMU 健全性は `test imu` のログで確認する。
  status_led::set(status_led::pattern::SLOW_BLINK);

  // feature（テストで使うもののみ）
  wheel_speed_meter::init({{&encoders_[0], &encoders_[1], &encoders_[2]},
                           p_->controller.sample_time});
  state_estimator::init({p_});

  // test components
  estop_test::init({&estop_ext_pin_, &estop_btn_pin_});
  tick_timing_test::init();
  imu_attitude_test::init({p_});
  encoder_test::init({{&enc_count_[0], &enc_count_[1], &enc_count_[2]},
                      {&encoders_[0], &encoders_[1], &encoders_[2]},
                      {&motors_[0], &motors_[1], &motors_[2]}});
  omni_drive_test::init({p_, {&motors_[0], &motors_[1], &motors_[2]}});
  motor_deadzone_test::init({{&motors_[0], &motors_[1], &motors_[2]}});

  // runner + tick
  control_tick::init();
  control_tick::start();
  return imu_ok;
}

void test_application::run() {
  test_runner::run();
}

}  // namespace bb
