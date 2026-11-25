// wheel_servo の duty 変換テスト。
#include <math.h>

#include "common/control_params/control_params.hpp"
#include "common/param_store/param_store.hpp"
#include "feature/wheel_servo/wheel_servo.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

wheel_servo::input base_input() {
  wheel_servo::input in{};
  in.mode = 1;
  in.enable = true;
  return in;
}

void test_zero_input_zero_duty() {
  wheel_servo::init({&get_params()});
  const auto out = wheel_servo::step(base_input());
  for (int i = 0; i < 3; ++i) {
    CHECK_NEAR(out.duty[i], 0.0F, 1e-6F);
  }
}

void test_duty_sign_matches_torque() {
  // 線形性の検証は不感帯補償なしで行う（FR-16 の底上げは比を変える）。
  param_store::init_defaults();
  for (uint8_t k = 0; k < 6; ++k) (void)param_store::set_key(k, 0.0F);
  const auto& p = get_params();
  wheel_servo::init({&p});
  // 昇順ランプを避けるため数回回す（prop_torque=0 のまま）。
  wheel_servo::input in = base_input();
  // 線形領域の確認のため全輪を牽引キャップ（0.065 N*m）未満に抑える。
  in.attitude_torque = {0.03F, -0.03F, 0.06F};
  wheel_servo::output out{};
  for (int i = 0; i < 50; ++i) out = wheel_servo::step(in);
  CHECK_TRUE(out.duty[0] > 0.0F);
  CHECK_TRUE(out.duty[1] < 0.0F);
  CHECK_TRUE(out.duty[2] > 0.0F);
  // トルク/duty は概ね線形: duty2 ≈ 2 * |duty1|
  CHECK_NEAR(fabsf(out.duty[2] / out.duty[1]), 2.0F, 0.3F);
}

void test_duty_clamped() {
  const auto& p = get_params();
  wheel_servo::init({&p});
  wheel_servo::input in = base_input();
  in.attitude_torque = {100.0F, -100.0F, 50.0F};
  const auto out = wheel_servo::step(in);
  for (int i = 0; i < 3; ++i) {
    CHECK_TRUE(fabsf(out.duty[i]) <= p.driver.max_duty_cycle + 1e-6F);
    CHECK_FINITE(out.duty[i]);
  }
}

void test_mode0_no_propulsion() {
  const auto& p = get_params();
  wheel_servo::init({&p});
  wheel_servo::input in = base_input();
  in.mode = 0;
  in.wheel_rate_command = {5.0F, 5.0F, 5.0F};
  const auto out = wheel_servo::step(in);
  // attitude_torque 0 かつ推進無効 → duty はゼロ（背起電力項のみ、輪速0）
  for (int i = 0; i < 3; ++i) {
    CHECK_NEAR(out.duty[i], 0.0F, 1e-3F);
  }
}

void test_disabled_outputs_zero_duty() {
  // enable=false（STANDBY/FAULT）は計測輪速に依らず duty=0。
  // エンコーダ浮き等で wheel_rate が不定値でもモータを駆動しない。
  const auto& p = get_params();
  wheel_servo::init({&p});
  wheel_servo::input in{};
  in.enable = false;
  in.mode = 1;
  in.attitude_torque = {0.1F, -0.1F, 0.05F};
  in.wheel_rate = {30.0F, -25.0F, 20.0F};
  auto out = wheel_servo::step(in);
  for (int i = 0; i < 3; ++i) {
    CHECK_NEAR(out.duty[i], 0.0F, 1e-6F);
  }
  for (int i = 0; i < 200; ++i) out = wheel_servo::step(in);
  for (int i = 0; i < 3; ++i) {
    CHECK_NEAR(out.duty[i], 0.0F, 1e-6F);
  }
}

void test_speed_lowpass() {
  const auto& p = get_params();
  wheel_servo::init({&p});
  wheel_servo::input in = base_input();
  in.wheel_rate = {10.0F, 0.0F, 0.0F};
  auto out = wheel_servo::step(in);
  const float blend = p.controller.servo_sample_time /
                      (p.torque_drive.back_emf_speed_time_constant +
                       p.controller.servo_sample_time);
  CHECK_NEAR(out.filtered_speed[0], 10.0F * blend, blend * 0.1F);
  // 定常輪速を入れ続けるとフィルタ出力は輪速へ近づく
  for (int i = 0; i < 2000; ++i) out = wheel_servo::step(in);
  CHECK_NEAR(out.filtered_speed[0], 10.0F, 0.05F);
}

}  // namespace

void wheel_servo_tests() {
  test_zero_input_zero_duty();
  test_duty_sign_matches_torque();
  test_duty_clamped();
  test_mode0_no_propulsion();
  test_disabled_outputs_zero_duty();
  test_speed_lowpass();
}

}  // namespace bb
