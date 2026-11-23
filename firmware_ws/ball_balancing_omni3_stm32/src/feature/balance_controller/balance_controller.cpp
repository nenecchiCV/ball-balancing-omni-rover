#include "feature/balance_controller/balance_controller.hpp"

#include <etl/algorithm.h>
#include <math.h>

namespace bb {

namespace {

inline vec2 rot_body_from_world(const vec2& v, float yaw) {
  const float c = cosf(yaw);
  const float s = sinf(yaw);
  return {c * v[0] + s * v[1], -s * v[0] + c * v[1]};
}

inline vec3 mat_vec(const float m[3][10], const vec10& v) {
  vec3 r{};
  for (int i = 0; i < 3; ++i) {
    float s = 0.0F;
    for (int j = 0; j < 10; ++j) s += m[i][j] * v[j];
    r[i] = s;
  }
  return r;
}

}  // namespace

void balance_controller::init(const config& cfg) {
  p_ = cfg.params;
  reset();
}

void balance_controller::reset() {
  prev_mode_ = 0;
  mode0_latch_ = false;
  elevated_authority_ = false;
  settle_wheel_rate_ = vec3{};
  settle_kin_prev_ = vec2{};
  settle_kin_accel_ = vec2{};
  elevated_init_ = false;
  body_rate_filter_ = vec3{};
  body_rate_filter_init_ = false;
  wheel_rate_filter_ = vec3{};
  wheel_rate_filter_init_ = false;
  prev_command_velocity_ = vec2{};
  prev_cmd_init_ = false;
  acceleration_demand_slew_ = vec2{};
  slew_init_ = false;
  static_lean_bias_ = vec2{};
  drift_filter_ = vec2{};
  drift_rate_ = 0.0F;
  prev_drift_mag_ = 0.0F;
  seen_still_ = false;
  adapt_init_ = false;
  prev_adapt_rest_ = false;
  tilt_integral_ = vec2{};
  tilt_integral_init_ = false;
  yaw_bias_ready_ = false;
  ramp_clock_ = 0.0F;
  ramp_started_ = false;
  velocity_integral_ = vec2{};
}

balance_controller::output balance_controller::step(
    const state_estimator::output& est, const vec3& command_in, bool enable,
    const vec3& wheel_rate) {
  step_context ctx{};
  ctx.est = &est;
  ctx.command = command_in;
  ctx.enable = enable;
  ctx.wheel_rate = wheel_rate;
  ctx.dt = p_->controller.sample_time;
  ctx.g = p_->physical.gravity;
  ctx.roll = est.euler[0];
  ctx.pitch = est.euler[1];
  ctx.yaw = est.euler[2];
  ctx.body_rate = est.gyro_body;
  ctx.velocity_world = est.velocity_world2;
  ctx.tilt_magnitude = sqrtf(ctx.roll * ctx.roll + ctx.pitch * ctx.pitch);

  update_yaw_bias_latch(ctx);

  // モード判定（mode 0 は終端ラッチ）
  const auto& c = p_->controller;
  if (!enable || ctx.tilt_magnitude >= c.fallen_tilt || mode0_latch_) {
    ctx.mode = 0;
    mode0_latch_ = true;
  } else if (ctx.tilt_magnitude >= c.recovery_tilt ||
             (prev_mode_ == 2 && ctx.tilt_magnitude >= c.recovery_exit_tilt)) {
    ctx.mode = 2;
  } else {
    ctx.mode = 1;
  }
  prev_mode_ = ctx.mode;

  update_settle_tracking(ctx);
  prepare_commands(ctx);
  update_ball_velocity(ctx);
  apply_follow_brake(ctx);
  integrate_velocity(ctx);

  // 状態ベクトル組立
  ctx.state[0] = ctx.velocity_body_used[0];
  ctx.state[1] = ctx.velocity_body_used[1];
  ctx.state[2] = ctx.pitch;
  ctx.state[3] = -ctx.roll;
  ctx.state[4] = ctx.body_rate[1];
  ctx.state[5] = -ctx.body_rate[0];
  ctx.state[6] = ctx.body_rate[2];
  ctx.state[7] = wheel_rate[0];
  ctx.state[8] = wheel_rate[1];
  ctx.state[9] = wheel_rate[2];

  ctx.yaw_rate_command =
      etl::clamp(ctx.command[2], -c.max_yaw_rate, c.max_yaw_rate);
  if (!yaw_bias_ready_ &&
      fabsf(ctx.command[2]) <= c.yaw_bias_command_bypass_threshold) {
    ctx.yaw_rate_command = 0.0F;
  }

  output out{};
  if (!p_->torque_drive.enabled) {
    // レガシー速度経路（torque_drive 無効）は実装対象外。
    out.mode = ctx.mode;
    out.yaw_bias_ready = yaw_bias_ready_;
    out.settle_witness = true;
    out.enable = enable;
    return out;
  }

  update_filters(ctx);
  apply_yaw_gate(ctx);
  compute_acceleration_demand(ctx);
  synthesize_torque(ctx);
  resolve_output(ctx);
  update_static_lean_adaptation(ctx);
  apply_anti_windup(ctx);

  out.attitude_torque = ctx.attitude_torque;
  out.velocity_integral = velocity_integral_;
  out.mode = ctx.mode;
  out.yaw_bias_ready = yaw_bias_ready_;
  out.wheel_rate_command = ctx.wheel_rate_command;
  out.settle_witness = ctx.settle_witness;
  out.enable = enable;
  return out;
}

void balance_controller::update_yaw_bias_latch(step_context& ctx) {
  // ヨーバイアス準備ラッチ（ballbot_yaw_bias_startup_guard）
  yaw_bias_ready_ =
      yaw_bias_ready_ ||
      (ctx.est->bias_learning_enabled &&
       fabsf(ctx.body_rate[2]) <= p_->controller.yaw_bias_ready_rate_threshold);
}

void balance_controller::update_settle_tracking(step_context& ctx) {
  const auto& c = p_->controller;
  // エレベーテッド権限 + settle 輪速フィルタ
  if (!elevated_init_ || !ctx.enable || ctx.mode == 0) {
    elevated_authority_ = false;
    settle_wheel_rate_ = vec3{};
    settle_kin_prev_ = vec2{};
    settle_kin_accel_ = vec2{};
    elevated_init_ = true;
  }
  if (settle_wheel_rate_ == vec3{}) settle_wheel_rate_ = ctx.wheel_rate;
  const float settle_blend = ctx.dt / (0.25F + ctx.dt);
  settle_wheel_rate_ =
      settle_wheel_rate_ + settle_blend * (ctx.wheel_rate - settle_wheel_rate_);
  const float clear_rate = c.recovery_authority_clear_rate;
  if (ctx.mode == 2) {
    elevated_authority_ = true;
    settle_kin_prev_ = vec2{};
    settle_kin_accel_ = vec2{};
  } else if (ctx.mode == 1 && elevated_authority_ &&
             max_abs(settle_wheel_rate_) < clear_rate) {
    elevated_authority_ = false;
  }
}

void balance_controller::prepare_commands(step_context& ctx) {
  const auto& c = p_->controller;
  ctx.velocity_body = rot_body_from_world(ctx.velocity_world, ctx.yaw);
  ctx.command_velocity_body =
      rot_body_from_world({ctx.command[0], ctx.command[1]}, ctx.yaw);
  ctx.command_speed = norm(ctx.command_velocity_body);

  // 連成 (v, yaw_rate) 実現性リミット
  if (c.coupled_command_limit_enabled) {
    const float wheel_rate_demand =
        sinf(p_->physical.contact_latitude) * ctx.command_speed +
        p_->rover.chassis_radius * fabsf(ctx.command[2]);
    const float wheel_rate_budget =
        p_->physical.wheel_radius * p_->motor.speed_command_limit;
    if (wheel_rate_demand > wheel_rate_budget) {
      const float scale = wheel_rate_budget / wheel_rate_demand;
      ctx.command_velocity_body = ctx.command_velocity_body * scale;
      ctx.command[2] *= scale;
      ctx.command_speed = norm(ctx.command_velocity_body);
    }
  }
  if (ctx.command_speed > c.max_speed) {
    ctx.command_velocity_body =
        ctx.command_velocity_body * (c.max_speed / ctx.command_speed);
  }
}

void balance_controller::update_ball_velocity(step_context& ctx) {
  const auto& td = p_->torque_drive;
  // ボール速度（体軸）: r_bw * [rb*w_ball_y; -rb*w_ball_x]
  const float rb = p_->physical.ball_radius;
  ctx.ball_velocity_body = rot_body_from_world(
      {rb * ctx.est->ball_rate_world[1], -rb * ctx.est->ball_rate_world[0]},
      ctx.yaw);
  ctx.command_velocity_body_user = ctx.command_velocity_body;
  ctx.velocity_body_used = ctx.velocity_body;

  // ポストリカバリ settle: 推定器のボール速度チャネルが高値化するため、
  // エンコーダ由来の輪速へ切替える。
  ctx.w_inv = inverse3(p_->lqi.wheel_speed_from_planar_velocity);
  ctx.settle_active =
      ctx.mode == 1 && elevated_authority_ && td.settle_follow_gain > 0.0F;
  ctx.settle_witness = ctx.mode == 1 && elevated_authority_;
  if (ctx.settle_witness) {
    const vec3 settle_kin_rate = mul(ctx.w_inv, settle_wheel_rate_);
    const vec2 raw_kin_accel =
        (vec2{settle_kin_rate[0], settle_kin_rate[1]} - settle_kin_prev_) *
        (1.0F / ctx.dt);
    settle_kin_prev_ = {settle_kin_rate[0], settle_kin_rate[1]};
    const float settle_blend = ctx.dt / (0.25F + ctx.dt);
    settle_kin_accel_ =
        settle_kin_accel_ + settle_blend * (raw_kin_accel - settle_kin_accel_);
    ctx.ball_velocity_body = {settle_kin_rate[0], settle_kin_rate[1]};
    ctx.velocity_body_used = {settle_kin_rate[0], settle_kin_rate[1]};
  }
  ctx.chase_body = ctx.ball_velocity_body;
}

void balance_controller::apply_follow_brake(step_context& ctx) {
  const auto& td = p_->torque_drive;
  // フォロースルーブレーキ
  if (td.enabled && td.stop_follow_gain > 0.0F) {
    const float follow_gain =
        ctx.settle_active ? td.settle_follow_gain : td.stop_follow_gain;
    const float chase_speed = norm(ctx.chase_body);
    if (ctx.command_speed < follow_gain * chase_speed) {
      ctx.command_velocity_body = ctx.chase_body * follow_gain;
      ctx.command_speed = norm(ctx.command_velocity_body);
    }
  }
}

void balance_controller::integrate_velocity(step_context& ctx) {
  const auto& c = p_->controller;
  const auto& td = p_->torque_drive;
  // 速度積分（デッドゾーン + 指令ゲート + 逆ワインド）
  const bool integral_gate_open =
      norm(ctx.command_velocity_body_user) > td.integral_command_gate;
  ctx.candidate_integral = velocity_integral_;
  if (ctx.mode == 1 && integral_gate_open) {
    vec2 velocity_error =
        ctx.command_velocity_body_user - ctx.ball_velocity_body;
    if (td.integral_unwind_floor_enabled) {
      for (int i = 0; i < 2; ++i) {
        const bool opposing =
            (velocity_error[i] > 0) != (ctx.candidate_integral[i] > 0) &&
            fabsf(velocity_error[i]) > td.integral_deadzone;
        if (opposing) {
          velocity_error[i] = copysignf(
              etl::max(fabsf(velocity_error[i]), td.integral_unwind_floor),
              velocity_error[i]);
        }
      }
    }
    if (norm(velocity_error) < td.integral_deadzone) velocity_error = vec2{};
    ctx.candidate_integral = ctx.candidate_integral + ctx.dt * velocity_error;
    ctx.candidate_integral[0] =
        etl::clamp(ctx.candidate_integral[0], -c.velocity_integral_limit[0],
                   c.velocity_integral_limit[0]);
    ctx.candidate_integral[1] =
        etl::clamp(ctx.candidate_integral[1], -c.velocity_integral_limit[1],
                   c.velocity_integral_limit[1]);
  } else if (ctx.mode == 1) {
    ctx.candidate_integral = velocity_integral_;
  } else {
    ctx.candidate_integral = vec2{};
  }
}

void balance_controller::update_filters(step_context& ctx) {
  const auto& td = p_->torque_drive;
  // ジャイロレートのローパス
  ctx.filtered_body_rate = ctx.body_rate;
  if (td.rate_filter_time_constant > 0.0F) {
    if (!body_rate_filter_init_ || ctx.mode != 1) {
      body_rate_filter_ = ctx.body_rate;
      body_rate_filter_init_ = true;
    } else {
      const float w = ctx.dt / (td.rate_filter_time_constant + ctx.dt);
      body_rate_filter_ =
          body_rate_filter_ + w * (ctx.body_rate - body_rate_filter_);
    }
    ctx.filtered_body_rate = body_rate_filter_;
  }
  ctx.state[4] = ctx.filtered_body_rate[1];
  ctx.state[5] = -ctx.filtered_body_rate[0];
  ctx.state[6] = ctx.filtered_body_rate[2];

  // 輪速のローパス（オブザーバは既定無効）
  ctx.filtered_wheel_rate = ctx.wheel_rate;
  if (td.wheel_rate_filter_time_constant > 0.0F) {
    if (!wheel_rate_filter_init_ || ctx.mode != 1) {
      wheel_rate_filter_ = ctx.wheel_rate;
      wheel_rate_filter_init_ = true;
    } else {
      const float w = ctx.dt / (td.wheel_rate_filter_time_constant + ctx.dt);
      wheel_rate_filter_ =
          wheel_rate_filter_ + w * (ctx.wheel_rate - wheel_rate_filter_);
    }
    ctx.filtered_wheel_rate = wheel_rate_filter_;
  }
}

void balance_controller::apply_yaw_gate(step_context& ctx) {
  const auto& c = p_->controller;
  const auto& td = p_->torque_drive;
  // ヨーゲート
  ctx.command_norm12 =
      sqrtf(ctx.command[0] * ctx.command[0] + ctx.command[1] * ctx.command[1]);
  // 指令はクランプ後の ctx.command ではなく元指令で判定する
  // （MATLAB と同じく command_in の x,y ノルムを使う）
  const bool stationary =
      norm(ctx.velocity_body_used) < td.yaw_gate.max_speed &&
      ctx.tilt_magnitude < td.yaw_gate.max_tilt && ctx.command_speed == 0.0F;
  ctx.yaw_active = fabsf(ctx.command[2]) > td.yaw_gate.command_threshold &&
                   (stationary || ctx.command_speed > 0.0F);
  if (ctx.yaw_active) {
    ctx.state[6] = ctx.state[6] - ctx.yaw_rate_command;
  } else {
    ctx.state[6] = 0.0F;
  }

  // 速度状態フィードバックは settle 中のみ
  if (ctx.settle_witness) {
    ctx.state[0] =
        ctx.ball_velocity_body[0] - ctx.command_velocity_body_user[0];
    ctx.state[1] =
        ctx.ball_velocity_body[1] - ctx.command_velocity_body_user[1];
  } else {
    ctx.state[0] = 0.0F;
    ctx.state[1] = 0.0F;
  }
  if (ctx.tilt_magnitude > c.tilt_priority_start) {
    ctx.candidate_integral = vec2{};
  }

  float integral_leak = td.integral_leak;
  if (ctx.command_speed > td.integral_command_gate) {
    integral_leak = td.integral_drive_leak;
  }
  ctx.candidate_integral =
      ctx.candidate_integral * (1.0F - integral_leak * ctx.dt);
  if (!td.integral_enabled) ctx.candidate_integral = vec2{};
  if (!td.wheel_rate_feedback_enabled) {
    ctx.state[7] = 0.0F;
    ctx.state[8] = 0.0F;
    ctx.state[9] = 0.0F;
  }
}

void balance_controller::compute_acceleration_demand(step_context& ctx) {
  const auto& td = p_->torque_drive;
  const float g = ctx.g;
  // 外ループ: 速度誤差 -> リーン要求
  float velocity_lean_gain = td.velocity_lean_gain;
  if (ctx.settle_active && td.settle_lean_gain > 0.0F) {
    velocity_lean_gain = td.settle_lean_gain;
  }
  ctx.acceleration_demand = vec2{};
  if (velocity_lean_gain != 0.0F) {
    const vec2 velocity_error_body =
        ctx.command_velocity_body_user - ctx.ball_velocity_body;
    ctx.acceleration_demand = velocity_lean_gain * velocity_error_body;
    if (ctx.settle_active) {
      ctx.acceleration_demand =
          ctx.acceleration_demand - td.settle_lean_damping * settle_kin_accel_;
    }
    // 指令加速度 FF（片側）
    if (td.accel_feedforward_gain != 0.0F) {
      if (!prev_cmd_init_ || ctx.mode != 1) {
        prev_command_velocity_ = ctx.command_velocity_body_user;
        prev_cmd_init_ = true;
      }
      const vec2 command_accel_body =
          (ctx.command_velocity_body_user - prev_command_velocity_) *
          (1.0F / ctx.dt);
      prev_command_velocity_ = ctx.command_velocity_body_user;
      const float accel_mag = norm(command_accel_body);
      if (accel_mag > 1e-4F &&
          dot(velocity_error_body, command_accel_body * (1.0F / accel_mag)) >
              -0.02F) {
        ctx.acceleration_demand =
            ctx.acceleration_demand +
            td.accel_feedforward_gain * command_accel_body;
      }
    }
    // 抗力 FF（片側）
    if (td.drag_lean_feedforward != 0.0F) {
      vec2 drag_dir{};
      const float drag_ball_speed = norm(ctx.ball_velocity_body);
      if (drag_ball_speed > 0.02F) {
        drag_dir = ctx.ball_velocity_body * (1.0F / drag_ball_speed);
      } else if (norm(ctx.command_velocity_body_user) > 0.01F) {
        drag_dir = ctx.command_velocity_body_user *
                   (1.0F / norm(ctx.command_velocity_body_user));
      }
      if (dot(velocity_error_body, drag_dir) > -0.02F) {
        ctx.acceleration_demand =
            ctx.acceleration_demand + td.drag_lean_feedforward *
                                          p_->physical.rolling_friction * g *
                                          drag_dir;
      }
    }
    float lean_limit = td.lean_setpoint_limit;
    if (ctx.settle_active && td.settle_lean_cap < lean_limit) {
      lean_limit = td.settle_lean_cap;
    }
    ctx.acceleration_demand[0] =
        etl::clamp(ctx.acceleration_demand[0], -lean_limit * g, lean_limit * g);
    ctx.acceleration_demand[1] =
        etl::clamp(ctx.acceleration_demand[1], -lean_limit * g, lean_limit * g);
    // リーン要求のスルーレート制限
    if (td.lean_rate_limit > 0.0F) {
      if (!slew_init_ || ctx.mode != 1) {
        acceleration_demand_slew_ = ctx.acceleration_demand;
        slew_init_ = true;
      } else {
        const float step_limit = td.lean_rate_limit * g * ctx.dt;
        vec2 delta = ctx.acceleration_demand - acceleration_demand_slew_;
        delta[0] = etl::clamp(delta[0], -step_limit, step_limit);
        delta[1] = etl::clamp(delta[1], -step_limit, step_limit);
        acceleration_demand_slew_ = acceleration_demand_slew_ + delta;
      }
      ctx.acceleration_demand = acceleration_demand_slew_;
    }
    // コーンガード
    const bool deepening =
        ctx.settle_active || (ctx.acceleration_demand[0] * ctx.pitch -
                                  ctx.acceleration_demand[1] * ctx.roll >
                              0.0F);
    if (deepening) {
      const float excess_tilt = etl::max(ctx.tilt_magnitude - lean_limit, 0.0F);
      const float fade = etl::max(1.0F - excess_tilt / td.lean_fade_tilt, 0.0F);
      ctx.acceleration_demand = ctx.acceleration_demand * fade;
    }
    // エキセントリックガード
    if (td.accel_rel_pos_guard_gain != 0.0F && ctx.command_speed > 0.01F) {
      const vec2 command_dir =
          ctx.command_velocity_body * (1.0F / ctx.command_speed);
      const float eccentricity = etl::max(
          -dot(ctx.est->rel_pos_body2, command_dir) - td.accel_rel_pos_margin,
          0.0F);
      const float guard_scale =
          etl::max(td.accel_rel_pos_guard_floor,
                   1.0F - td.accel_rel_pos_guard_gain * eccentricity);
      const float accel_component = dot(ctx.acceleration_demand, command_dir);
      if (accel_component > 0.0F) {
        ctx.acceleration_demand =
            ctx.acceleration_demand -
            accel_component * command_dir * (1.0F - guard_scale);
      }
    }
    // オーバースピード・コーストゲート
    if (!ctx.settle_active) {
      const float coast_ball_speed = norm(ctx.ball_velocity_body);
      const float coast_floor = td.overspeed_coast_floor;
      if (coast_ball_speed > norm(ctx.command_velocity_body_user) +
                                 td.overspeed_coast_margin &&
          coast_ball_speed > coast_floor) {
        const vec2 coast_dir =
            ctx.ball_velocity_body * (1.0F / coast_ball_speed);
        ctx.acceleration_demand =
            ctx.acceleration_demand -
            dot(ctx.acceleration_demand, coast_dir) * coast_dir;
      }
    }
    ctx.state[2] -= ctx.acceleration_demand[0] / g;
    ctx.state[3] += ctx.acceleration_demand[1] / g;
  }

  // 静的リーン適応のバイアス注入（指令 ≈ 0 のときのみ）
  const bool static_lean_adapt = td.static_lean_adapt_gain != 0.0F;
  if (static_lean_adapt && ctx.mode == 1 && ctx.command_norm12 < 0.01F) {
    ctx.state[2] -= static_lean_bias_[0];
    ctx.state[3] -= static_lean_bias_[1];
  }
}

void balance_controller::synthesize_torque(step_context& ctx) {
  const auto& td = p_->torque_drive;
  const mat33& wmap = p_->lqi.wheel_speed_from_planar_velocity;
  // 傾斜ゲインスケジューリング
  float stiffen_start = td.lean_setpoint_limit;
  if (ctx.settle_active) {
    stiffen_start = etl::min(stiffen_start, td.settle_lean_cap);
  }
  const float boost =
      1.0F + (td.tilt_stiffen_gain - 1.0F) *
                 etl::min(etl::max(ctx.tilt_magnitude - stiffen_start, 0.0F) /
                              td.tilt_stiffen_tilt,
                          1.0F);
  for (int i = 2; i < 6; ++i) ctx.state[i] *= boost;

  // LQI 状態フィードバック
  ctx.raw_wheel_command =
      mat_vec(p_->torque_lqi.state_gain, ctx.state) * (-1.0F);
  // 傾斜積分
  if (td.tilt_integral_gain != 0.0F) {
    if (!tilt_integral_init_ || ctx.mode != 1) {
      tilt_integral_ = vec2{};
      tilt_integral_init_ = true;
    } else {
      tilt_integral_[0] += ctx.dt * ctx.state[2];
      tilt_integral_[1] += ctx.dt * ctx.state[3];
      tilt_integral_ = tilt_integral_ * (1.0F - td.tilt_integral_leak * ctx.dt);
      tilt_integral_[0] = etl::clamp(tilt_integral_[0], -td.tilt_integral_limit,
                                     td.tilt_integral_limit);
      tilt_integral_[1] = etl::clamp(tilt_integral_[1], -td.tilt_integral_limit,
                                     td.tilt_integral_limit);
    }
    for (int i = 0; i < 3; ++i) {
      ctx.raw_wheel_command[i] -=
          td.tilt_integral_gain *
          (p_->torque_lqi.state_gain[i][2] * tilt_integral_[0] +
           p_->torque_lqi.state_gain[i][3] * tilt_integral_[1]);
    }
  }
  // 重力保持フィードフォワード
  float velocity_lean_gain = td.velocity_lean_gain;
  if (ctx.settle_active && td.settle_lean_gain > 0.0F) {
    velocity_lean_gain = td.settle_lean_gain;
  }
  if (velocity_lean_gain != 0.0F) {
    const vec2 demand_dir = {ctx.acceleration_demand[0] / ctx.g,
                             -ctx.acceleration_demand[1] / ctx.g};
    vec3 hold{};
    for (int i = 0; i < 3; ++i) {
      hold[i] = td.lean_hold_torque[i][0] * demand_dir[0] +
                td.lean_hold_torque[i][1] * demand_dir[1];
    }
    ctx.raw_wheel_command =
        ctx.raw_wheel_command + startup_ramp(hold, ctx.enable, ctx.mode);
  }
  // コモンモード除去
  if (!ctx.yaw_active) {
    const float mean = (ctx.raw_wheel_command[0] + ctx.raw_wheel_command[1] +
                        ctx.raw_wheel_command[2]) /
                       3.0F;
    for (auto& v : ctx.raw_wheel_command) v -= mean;
  }
  // キネマティック輪速
  const vec3 kinematic_rate = mul(
      wmap,
      {ctx.velocity_body_used[0], ctx.velocity_body_used[1], ctx.body_rate[2]});
  // ボール z-spin ブレーキ
  if (td.ball_yaw_gain != 0.0F) {
    float ball_yaw_torque = td.ball_yaw_gain * ctx.est->ball_rate_world[2];
    ball_yaw_torque = etl::clamp(ball_yaw_torque, -td.ball_yaw_torque_limit,
                                 td.ball_yaw_torque_limit);
    for (auto& v : ctx.raw_wheel_command) v += ball_yaw_torque;
  }
  // 移動中ヨーダンパー
  if (td.yaw_damping_gain != 0.0F && !ctx.yaw_active &&
      ctx.command_speed > 0.0F) {
    for (auto& v : ctx.raw_wheel_command) {
      v -= td.yaw_damping_gain * ctx.filtered_body_rate[2];
    }
  }
  // スリップダンパー
  const vec3 slip_rate = ctx.filtered_wheel_rate - kinematic_rate;
  const float slip_deadzone = td.filtered_slip_deadzone;
  ctx.slip_active = vec3{};
  for (int i = 0; i < 3; ++i) {
    ctx.slip_active[i] = copysignf(
        etl::max(fabsf(slip_rate[i]) - slip_deadzone, 0.0F), slip_rate[i]);
  }
  for (int i = 0; i < 3; ++i) {
    ctx.raw_wheel_command[i] -= td.slip_damping_gain * ctx.slip_active[i];
  }
}

void balance_controller::resolve_output(step_context& ctx) {
  const auto& c = p_->controller;
  const auto& td = p_->torque_drive;
  const mat33& wmap = p_->lqi.wheel_speed_from_planar_velocity;
  // 輪速コマンド
  ctx.wheel_rate_command = mul(
      wmap, {ctx.command_velocity_body[0], ctx.command_velocity_body[1], 0.0F});
  ctx.torque_cap = elevated_authority_
                       ? etl::min(p_->driver.continuous_torque_limit,
                                  c.recovery_wheel_torque_limit)
                       : etl::min(p_->driver.continuous_torque_limit,
                                  c.traction_limited_wheel_torque);
  ctx.attitude_torque = vec3{};
  for (int i = 0; i < 3; ++i) {
    ctx.attitude_torque[i] =
        etl::clamp(ctx.raw_wheel_command[i], -ctx.torque_cap, ctx.torque_cap);
  }
  if (ctx.mode == 2) {
    vec10 state_recovery = ctx.state;
    state_recovery[0] = 0.0F;
    state_recovery[1] = 0.0F;
    state_recovery[6] = 0.0F;
    state_recovery[7] = 0.0F;
    state_recovery[8] = 0.0F;
    state_recovery[9] = 0.0F;
    ctx.attitude_torque = mat_vec(p_->torque_lqi.state_gain, state_recovery) *
                          (-c.recovery_gain_scale);
    for (int i = 0; i < 3; ++i) {
      ctx.attitude_torque[i] -= td.slip_damping_gain * ctx.slip_active[i];
      ctx.attitude_torque[i] =
          etl::clamp(ctx.attitude_torque[i], -ctx.torque_cap, ctx.torque_cap);
    }
    ctx.wheel_rate_command = vec3{};
  } else if (ctx.mode == 0) {
    ctx.attitude_torque = vec3{};
    ctx.wheel_rate_command = vec3{};
  }
}

void balance_controller::update_static_lean_adaptation(step_context& ctx) {
  const auto& td = p_->torque_drive;
  const bool static_lean_adapt = td.static_lean_adapt_gain != 0.0F;
  if (!static_lean_adapt) return;
  // 静的リーン適応の更新
  const vec2 drift_raw = rot_body_from_world(ctx.velocity_world, ctx.yaw);
  const float adapt_gate = td.static_lean_adapt_gate;
  const bool adapt_rest = ctx.enable && ctx.mode == 1 &&
                          ctx.command_norm12 < 0.01F &&
                          norm(drift_raw) < adapt_gate;
  if (!adapt_init_) {
    prev_adapt_rest_ = false;
    adapt_init_ = true;
  }
  if (!adapt_rest) {
    seen_still_ = false;
  } else if (!prev_adapt_rest_) {
    const vec3 wheel_ball_vel = mul(ctx.w_inv, ctx.wheel_rate);
    seen_still_ = norm(drift_raw) < 0.02F &&
                  sqrtf(wheel_ball_vel[0] * wheel_ball_vel[0] +
                        wheel_ball_vel[1] * wheel_ball_vel[1]) < 0.03F;
  }
  prev_adapt_rest_ = adapt_rest;
  const float adapt_tau = td.static_lean_adapt_time_constant;
  if (adapt_rest) {
    const float drift_w = ctx.dt / (adapt_tau + ctx.dt);
    drift_filter_ = drift_filter_ + drift_w * (drift_raw - drift_filter_);
    const float drift_mag = norm(drift_filter_);
    const float inst_rate = (drift_mag - prev_drift_mag_) / ctx.dt;
    prev_drift_mag_ = drift_mag;
    drift_rate_ = drift_rate_ + drift_w * (inst_rate - drift_rate_);
    const float adapt_deadband = td.static_lean_adapt_deadband;
    const float adapt_max_rate = td.static_lean_adapt_max_rate;
    if (drift_mag > adapt_deadband && drift_rate_ > 0.0F &&
        drift_rate_ < adapt_max_rate && seen_still_) {
      const vec2 drift_vec =
          drift_filter_ * ((drift_mag - adapt_deadband) / drift_mag);
      static_lean_bias_ =
          static_lean_bias_ - td.static_lean_adapt_gain * ctx.dt * drift_vec;
    }
    const float bias_limit = td.static_lean_adapt_limit;
    static_lean_bias_[0] =
        etl::clamp(static_lean_bias_[0], -bias_limit, bias_limit);
    static_lean_bias_[1] =
        etl::clamp(static_lean_bias_[1], -bias_limit, bias_limit);
  } else {
    drift_filter_ = vec2{};
    drift_rate_ = 0.0F;
    prev_drift_mag_ = 0.0F;
    static_lean_bias_ =
        static_lean_bias_ * (1.0F - td.static_lean_adapt_leak * ctx.dt);
  }
}

void balance_controller::apply_anti_windup(step_context& ctx) {
  const auto& c = p_->controller;
  const mat33& wmap = p_->lqi.wheel_speed_from_planar_velocity;
  const float rb = p_->physical.ball_radius;
  // 逆ワインド（残余 → 速度積分へリーク）
  vec2 next_velocity_integral{};
  if (ctx.mode == 1) {
    const float mass_effective = p_->rover.mass + p_->physical.ball_mass +
                                 p_->physical.ball_inertia11 / (rb * rb);
    const vec3 residual = ctx.attitude_torque - ctx.raw_wheel_command;
    // accel_residual = W(:,1:2)' * residual / mass_effective
    const vec2 accel_residual = {
        (wmap[0][0] * residual[0] + wmap[1][0] * residual[1] +
         wmap[2][0] * residual[2]) /
            mass_effective,
        (wmap[0][1] * residual[0] + wmap[1][1] * residual[1] +
         wmap[2][1] * residual[2]) /
            mass_effective};
    ctx.candidate_integral = ctx.candidate_integral +
                             ctx.dt * p_->lqi.anti_windup_gain * accel_residual;
    next_velocity_integral = {
        etl::clamp(ctx.candidate_integral[0], -c.velocity_integral_limit[0],
                   c.velocity_integral_limit[0]),
        etl::clamp(ctx.candidate_integral[1], -c.velocity_integral_limit[1],
                   c.velocity_integral_limit[1])};
  } else {
    next_velocity_integral = vec2{};
  }
  if (ctx.mode == 1 && elevated_authority_) next_velocity_integral = vec2{};

  velocity_integral_ = next_velocity_integral;
}

vec3 balance_controller::startup_ramp(const vec3& value, bool enable,
                                      uint8_t mode) {
  const auto& td = p_->torque_drive;
  if (td.startup_torque_ramp_time <= 0.0F) return value;
  if (!ramp_started_ || !enable || mode == 0) {
    ramp_clock_ = 0.0F;
    ramp_started_ = true;
  } else {
    ramp_clock_ += p_->controller.sample_time;
  }
  const float scale = etl::min(1.0F, ramp_clock_ / td.startup_torque_ramp_time);
  return value * scale;
}

}  // namespace bb
