#pragma once

#include "common/control_params/control_params.hpp"
#include "common/imu_sample/imu_sample.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// ballbot_estimator_step の移植（static class）。
// 状態: [q_wb(4); v_wb(3); omega_ball_w(3); rover_minus_ball_b_xy(2);
//        gyro_bias_z; qualification_time]
class state_estimator {
 public:
  state_estimator() = delete;

  struct config {
    const param_set* params;
  };

  struct output {
    // 次状態（14 要素相当）。
    vec4 quaternion{};
    vec3 velocity_world{};
    vec3 ball_rate_world{};
    vec2 relative_position_body{};
    float gyro_bias_z = 0.0F;
    float qualification_time = 0.0F;
    // estimate 相当。
    vec3 euler{};
    vec3 gyro_body{};
    vec2 velocity_world2{};
    vec2 rel_pos_body2{};
    // diagnostics 相当。
    vec3 accel_world{};
    float updated_gyro_bias_z = 0.0F;
    bool bias_learning_enabled = false;
    float updated_qualification_time = 0.0F;
    bool valid = true;
  };

  static void init(const config& cfg);
  static output step(const imu_sample& imu, const vec3& wheel_rate, float dt);
  // ジャイロバイアス学習状態（gyro_bias_z / EKF bias / qualification_time）
  // をリセットする（CALIBRATE_IMU、FR-13）。姿勢・速度等は保持する。
  // 外側タスクから呼ぶ（step と同一スレッドのため排他不要）。
  static void reset_bias();

 private:
  static inline const param_set* p_ = nullptr;

  // 状態
  static inline vec4 quaternion_{};
  static inline vec3 velocity_world_{};
  static inline vec3 ball_rate_world_{};
  static inline vec2 relative_position_body_{};
  static inline float gyro_bias_z_ = 0.0F;
  static inline float qualification_time_ = 0.0F;

  // EKF 内部
  static inline mat55 ekf_covariance_{};
  static inline vec2 ekf_bias_xy_{};
  static inline vec3 ekf_accel_world_{};
  static inline vec3 prev_ball_velocity_world_{};
  static inline bool ekf_init_ = false;

  // 補助フィルタ
  static inline vec3 filtered_wheel_rate_for_still_{};
  static inline bool still_init_ = false;
  static inline float stillness_hold_ = 0.0F;
  static inline bool stillness_hold_init_ = false;
  static inline vec2 ekf_rel_vel_body_{};
  static inline vec2 ekf_rel_acc_body_{};
  static inline bool ekf_rel_init_ = false;
  static inline vec3 ekf_gyro_dot_body_{};
  static inline vec3 ekf_prev_gyro_body_{};
  static inline bool ekf_gyro_init_ = false;
  static inline float ekf_wheel_slew_ = 0.0F;
  static inline vec3 ekf_prev_wheel_rate_{};
  static inline bool ekf_slew_init_ = false;
  static inline float inertial_hold_ = 0.0F;
};

}  // namespace bb
