// position_controller ホストテスト: 比例指令・速度飽和・到達・slew・STOP 保持。
#include "feature/position_controller/position_controller.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

position_controller::config cfg() {
  return {&get_params()};
}

void inactive_returns_zero() {
  position_controller::init(cfg());
  const auto o = position_controller::update(0.0F, 0.0F, 0.005F);
  CHECK_TRUE(!o.active);
  CHECK_NEAR(o.command[0], 0.0F, 1e-6F);
  CHECK_NEAR(o.command[1], 0.0F, 1e-6F);
  CHECK_NEAR(o.command[2], 0.0F, 1e-6F);
}

void proportional_command_slew_limited() {
  position_controller::init(cfg());
  position_controller::set_target(1.0F, 0.0F, 0.4F);
  // 1 ステップ目は accel_limit*dt = 0.5*0.005 = 0.0025 m/s まで。
  auto o = position_controller::update(0.0F, 0.0F, 0.005F);
  CHECK_TRUE(o.active);
  CHECK_NEAR(o.command[0], 0.0025F, 1e-5F);
  CHECK_NEAR(o.command[1], 0.0F, 1e-6F);
  CHECK_NEAR(o.command[2], 0.0F, 1e-6F);
  // 十分ステップ後は誤差比例（kp=1 → v=距離[m/s]、ただし上限）。
  for (int i = 0; i < 400; ++i) {
    o = position_controller::update(0.0F, 0.0F, 0.005F);
  }
  // 飽和は min(引数 0.4, max_speed=0.2) 側に効く。
  CHECK_NEAR(o.command[0], get_params().controller.max_speed, 0.02F);
}

void speed_limit_clamped_to_max() {
  position_controller::init(cfg());
  position_controller::set_target(0.1F, 0.0F, 99.0F);  // 上限は max_speed
  for (int i = 0; i < 400; ++i) {
    position_controller::update(0.0F, 0.0F, 0.005F);
  }
  const auto o = position_controller::update(0.0F, 0.0F, 0.005F);
  const float norm =
      sqrtf(o.command[0] * o.command[0] + o.command[1] * o.command[1]);
  CHECK_TRUE(norm <= get_params().controller.max_speed + 1e-6F);
}

void arrives_and_holds() {
  position_controller::init(cfg());
  position_controller::set_target(0.01F, 0.01F, 0.4F);  // < arrive_radius
  // 到達判定後にホールド。
  for (int i = 0; i < 100; ++i) {
    position_controller::update(0.01F, 0.01F, 0.005F);
  }
  const auto o = position_controller::update(0.01F, 0.01F, 0.005F);
  CHECK_TRUE(o.active);
  CHECK_TRUE(o.arrived);
  CHECK_NEAR(o.command[0], 0.0F, 1e-6F);
}

void hold_at_replaces_target() {
  position_controller::init(cfg());
  position_controller::set_target(5.0F, 5.0F, 0.4F);
  position_controller::hold_at(0.02F, 0.02F);  // STOP → 現在地保持
  const auto o = position_controller::update(0.02F, 0.02F, 0.005F);
  CHECK_TRUE(o.active);
  // 目標は現在地 → 誤差ほぼゼロ → 指令は小さい。
  const float norm =
      sqrtf(o.command[0] * o.command[0] + o.command[1] * o.command[1]);
  CHECK_TRUE(norm < 0.05F);
}

void clear_target_deactivates() {
  position_controller::init(cfg());
  position_controller::set_target(1.0F, 0.0F, 0.4F);
  position_controller::clear_target();
  const auto o = position_controller::update(0.0F, 0.0F, 0.005F);
  CHECK_TRUE(!o.active);
}

}  // namespace

void position_controller_tests() {
  inactive_returns_zero();
  proportional_command_slew_limited();
  speed_limit_clamped_to_max();
  arrives_and_holds();
  hold_at_replaces_target();
  clear_target_deactivates();
}

}  // namespace bb
