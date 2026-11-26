#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"
#include "feature/state_estimator/state_estimator.hpp"

namespace bb {

// ballbot_lqi_controller_update_rev2 の移植（200 hz 外側タスク、static
// class）。 step() は処理ブロックごとにプライベートヘルパへ分割されている。
class balance_controller {
 public:
  balance_controller() = delete;

  struct config {
    const param_set* params;
  };

  struct output {
    vec3 attitude_torque{};
    vec2 velocity_integral{};
    uint8_t mode = 0;
    bool yaw_bias_ready = false;
    vec3 wheel_rate_command{};
    bool settle_witness = false;
    bool enable = false;
  };

  static void init(const config& cfg);
  // 全内部状態（モードラッチ・積分・フィルタ・ランプ時計等）を初期状態へ
  // 戻す（START_OPERATION 受理時、FR-12）。パラメータ参照は保持する。
  static void reset();
  static output step(const state_estimator::output& est, const vec3& command,
                     bool enable, const vec3& wheel_rate);

 private:
  // step() 1 回分の作業領域。ヘルパ間でローカル状態を引き回すための集約型。
  struct step_context {
    const state_estimator::output* est;
    vec3 command;
    bool enable;
    vec3 wheel_rate;
    float dt;
    float g;
    float roll;
    float pitch;
    float yaw;
    float tilt_magnitude;
    vec3 body_rate;
    vec2 velocity_world;
    uint8_t mode;
    vec2 velocity_body;
    vec2 command_velocity_body;
    float command_speed;
    vec2 ball_velocity_body;
    vec2 velocity_body_used;
    vec2 command_velocity_body_user;
    vec2 chase_body;
    mat33 w_inv;
    bool settle_active;
    bool settle_witness;
    vec2 candidate_integral;
    vec10 state;
    vec3 filtered_body_rate;
    vec3 filtered_wheel_rate;
    float yaw_rate_command;
    bool yaw_active;
    float command_norm12;
    vec2 acceleration_demand;
    vec3 raw_wheel_command;
    vec3 slip_active;
    vec3 attitude_torque;
    vec3 wheel_rate_command;
    float torque_cap;
  };

  static void update_yaw_bias_latch(step_context& ctx);
  static void update_settle_tracking(step_context& ctx);
  static void prepare_commands(step_context& ctx);
  static void update_ball_velocity(step_context& ctx);
  static void apply_reversal_guard(step_context& ctx);
  static void apply_follow_brake(step_context& ctx);
  static void integrate_velocity(step_context& ctx);
  static void update_filters(step_context& ctx);
  static void apply_yaw_gate(step_context& ctx);
  static void compute_acceleration_demand(step_context& ctx);
  static void synthesize_torque(step_context& ctx);
  static void resolve_output(step_context& ctx);
  static void update_static_lean_adaptation(step_context& ctx);
  static void apply_anti_windup(step_context& ctx);
  static vec3 startup_ramp(const vec3& value, bool enable, uint8_t mode);

  static inline const param_set* p_ = nullptr;

  // MATLAB persistent の写像
  static inline uint8_t prev_mode_ = 0;
  static inline bool mode0_latch_ = false;
  static inline bool elevated_authority_ = false;
  static inline vec3 settle_wheel_rate_{};
  static inline vec2 settle_kin_prev_{};
  static inline vec2 settle_kin_accel_{};
  static inline bool elevated_init_ = false;
  static inline vec3 body_rate_filter_{};
  static inline bool body_rate_filter_init_ = false;
  static inline vec3 wheel_rate_filter_{};
  static inline bool wheel_rate_filter_init_ = false;
  static inline vec2 prev_command_velocity_{};
  static inline bool prev_cmd_init_ = false;
  static inline vec2 acceleration_demand_slew_{};
  static inline bool slew_init_ = false;
  static inline vec2 static_lean_bias_{};
  static inline vec2 drift_filter_{};
  static inline float drift_rate_ = 0.0F;
  static inline float prev_drift_mag_ = 0.0F;
  static inline bool seen_still_ = false;
  static inline bool prev_adapt_rest_ = false;
  static inline bool adapt_init_ = false;
  static inline vec2 tilt_integral_{};
  static inline bool tilt_integral_init_ = false;
  static inline bool yaw_bias_ready_ = false;
  static inline float ramp_clock_ = 0.0F;
  static inline bool ramp_started_ = false;
  static inline vec2 velocity_integral_{};
};

}  // namespace bb
