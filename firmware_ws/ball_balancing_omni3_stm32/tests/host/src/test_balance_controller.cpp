// balance_controller のモード遷移・ラッチ・出力健全性テスト。
#include <math.h>

#include "common/control_params/control_params.hpp"
#include "feature/balance_controller/balance_controller.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

state_estimator::output upright_estimate() {
  state_estimator::output e{};
  e.euler = {0.0F, 0.0F, 0.0F};
  e.gyro_body = vec3{};
  e.velocity_world2 = vec2{};
  e.ball_rate_world = vec3{};
  e.rel_pos_body2 = vec2{};
  e.bias_learning_enabled = true;
  e.valid = true;
  return e;
}

void check_output_finite(const balance_controller::output& out) {
  for (int i = 0; i < 3; ++i) {
    CHECK_FINITE(out.attitude_torque[i]);
    CHECK_FINITE(out.wheel_rate_command[i]);
  }
  CHECK_FINITE(out.velocity_integral[0]);
  CHECK_FINITE(out.velocity_integral[1]);
}

void test_disabled_goes_mode0() {
  balance_controller::init({&get_params()});
  const auto out =
      balance_controller::step(upright_estimate(), vec3{}, false, vec3{});
  CHECK_TRUE(out.mode == 0);
  CHECK_TRUE(out.attitude_torque == vec3{});
  CHECK_TRUE(out.wheel_rate_command == vec3{});
}

void test_upright_goes_mode1() {
  balance_controller::init({&get_params()});
  const auto out =
      balance_controller::step(upright_estimate(), vec3{}, true, vec3{});
  CHECK_TRUE(out.mode == 1);
  check_output_finite(out);
}

void test_fallen_latches_mode0() {
  const auto& p = get_params();
  balance_controller::init({&p});
  state_estimator::output e = upright_estimate();
  e.euler[0] = p.controller.fallen_tilt + 0.01F;
  auto out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(out.mode == 0);
  // mode0 ラッチ後は倒れが解消しても復帰しない
  out = balance_controller::step(upright_estimate(), vec3{}, true, vec3{});
  CHECK_TRUE(out.mode == 0);
  CHECK_TRUE(out.attitude_torque == vec3{});
}

void test_recovery_mode_and_exit_hysteresis() {
  const auto& p = get_params();
  balance_controller::init({&p});
  state_estimator::output e = upright_estimate();
  // recovery_tilt(5deg) 超・fallen_tilt(35deg) 未満 → mode 2
  e.euler[0] = 0.3F;
  auto out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(out.mode == 2);
  check_output_finite(out);
  // recovery_exit_tilt(2deg) 未満になるまでは mode 2 を維持
  e.euler[0] = p.controller.recovery_exit_tilt + 0.01F;
  out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(out.mode == 2);
  // recovery_exit_tilt 未満 → mode 1 へ復帰
  e.euler[0] = p.controller.recovery_exit_tilt * 0.5F;
  out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(out.mode == 1);
}

void test_yaw_bias_ready_latch() {
  const auto& p = get_params();
  balance_controller::init({&p});
  state_estimator::output e = upright_estimate();
  // bias 学習未許可ではラッチしない
  e.bias_learning_enabled = false;
  auto out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(!out.yaw_bias_ready);
  // 学習許可 + ヨーレート小 → ラッチ
  e.bias_learning_enabled = true;
  e.gyro_body[2] = p.controller.yaw_bias_ready_rate_threshold * 0.5F;
  out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(out.yaw_bias_ready);
  // ラッチは保持される
  e.gyro_body[2] = 1.0F;
  out = balance_controller::step(e, vec3{}, true, vec3{});
  CHECK_TRUE(out.yaw_bias_ready);
}

void test_long_run_stays_finite() {
  balance_controller::init({&get_params()});
  state_estimator::output e = upright_estimate();
  // 小さな傾き + コマンド速度を与えて長時間ステップ
  e.euler[0] = 0.01F;
  e.euler[1] = -0.01F;
  const vec3 cmd = {0.1F, 0.0F, 0.0F};
  balance_controller::output out{};
  for (int i = 0; i < 4000; ++i) {
    out = balance_controller::step(e, cmd, true, vec3{});
  }
  CHECK_TRUE(out.mode == 1);
  check_output_finite(out);
}

}  // namespace

void balance_controller_tests() {
  test_disabled_goes_mode0();
  test_upright_goes_mode1();
  test_fallen_latches_mode0();
  test_recovery_mode_and_exit_hysteresis();
  test_yaw_bias_ready_latch();
  test_long_run_stays_finite();
}

}  // namespace bb
