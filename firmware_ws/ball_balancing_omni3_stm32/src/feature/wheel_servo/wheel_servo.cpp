#include "feature/wheel_servo/wheel_servo.hpp"

#include <etl/algorithm.h>
#include <math.h>

namespace bb {

void wheel_servo::init(const config& cfg) {
  p_ = cfg.params;
  filtered_speed_ = vec3{};
  ramp_clock_ = 0.0F;
  ramp_started_ = false;
}

wheel_servo::output wheel_servo::step(const input& in) {
  const auto& td = p_->torque_drive;
  const float dt = p_->controller.servo_sample_time;

  // 逆起電力補償用のローパス輪速（MATLAB previous_integral 相当）。
  const float blend = dt / (td.back_emf_speed_time_constant + dt);
  filtered_speed_ = filtered_speed_ + blend * (in.wheel_rate - filtered_speed_);

  const bool elevated_authority = (in.mode == 2) || in.settle_witness;
  float torque_limit = etl::min(p_->driver.continuous_torque_limit,
                                p_->controller.traction_limited_wheel_torque);
  if (elevated_authority) {
    torque_limit = etl::min(p_->driver.continuous_torque_limit,
                            p_->controller.recovery_wheel_torque_limit);
  }

  // 推進輪速サーボ（mode 1 のみ）。
  vec3 prop_torque{};
  if (in.mode == 1 && td.propulsion_gain != 0.0F) {
    const float slip_deadzone = td.filtered_slip_deadzone;
    vec3 rate_active{};
    for (int i = 0; i < 3; ++i) {
      const float err = in.wheel_rate_command[i] - filtered_speed_[i];
      rate_active[i] =
          copysignf(etl::max(fabsf(err) - slip_deadzone, 0.0F), err);
    }
    for (int i = 0; i < 3; ++i) {
      prop_torque[i] =
          etl::clamp(td.propulsion_gain * rate_active[i],
                     -td.propulsion_torque_limit, td.propulsion_torque_limit);
      const float headroom =
          etl::max(torque_limit - fabsf(in.attitude_torque[i]), 0.0F);
      const float share =
          etl::max(headroom, td.propulsion_min_share * torque_limit);
      prop_torque[i] =
          copysignf(etl::min(fabsf(prop_torque[i]), share), prop_torque[i]);
    }
    if (in.settle_witness) prop_torque = vec3{};
    prop_torque = servo_startup_ramp(prop_torque, in.enable);
  }

  vec3 torque{};
  for (int i = 0; i < 3; ++i) {
    torque[i] = etl::clamp(in.attitude_torque[i] + prop_torque[i],
                           -torque_limit, torque_limit);
  }

  const float resistance =
      p_->motor.armature_resistance + p_->driver.bridge_on_resistance;
  output out{};
  for (int i = 0; i < 3; ++i) {
    const float current = torque[i] / p_->motor.torque_constant;
    float duty = (resistance * current +
                  p_->motor.back_emf_constant * filtered_speed_[i]) /
                 p_->driver.supply_voltage;
    duty =
        etl::clamp(duty, -p_->driver.max_duty_cycle, p_->driver.max_duty_cycle);
    // FR-16 不感帯補償: 非ゼロ指令をモータ実効デューティへ底上げする。
    // duty==0（無指令）は 0 を維持する。
    if (duty != 0.0F) {
      duty = copysignf(fabsf(duty) + p_->drive.deadzone_comp[i], duty);
      duty = etl::clamp(duty, -p_->driver.max_duty_cycle,
                        p_->driver.max_duty_cycle);
    }
    out.duty[i] = duty;
  }
  out.filtered_speed = filtered_speed_;
  return out;
}

vec3 wheel_servo::servo_startup_ramp(const vec3& value, bool enable) {
  const auto& td = p_->torque_drive;
  if (td.startup_torque_ramp_time <= 0.0F) return value;
  if (!ramp_started_ || !enable) {
    ramp_clock_ = 0.0F;
    ramp_started_ = true;
  } else {
    ramp_clock_ += p_->controller.servo_sample_time;
  }
  const float scale = etl::min(1.0F, ramp_clock_ / td.startup_torque_ramp_time);
  return value * scale;
}

}  // namespace bb
