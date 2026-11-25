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
    // 待機中慣性オドメトリ（FR-11）の ZUPT 用静止候補。
    // 車輪・ジャイロ・水平加速度残差が全て小さいとき true。
    bool still = false;
    float updated_gyro_bias_z = 0.0F;
    bool bias_learning_enabled = false;
    float updated_qualification_time = 0.0F;
    bool valid = true;
  };

  static void init(const config& cfg);
  static output step(const imu_sample& imu, const vec3& wheel_rate, float dt);
  // ジャイロバイアス学習状態（gyro_bias_z / EKF bias / qualification_time）
  // をリセットし、静止資格ウィンドウの生ジャイロ平均をバイアスとして
  // 採用する捕捉を武装する（CALIBRATE_IMU、FR-13）。姿勢・速度等は保持。
  // 外側タスクから呼ぶ（step と同一スレッドのため排他不要）。
  static void reset_bias();
  // FR-15 永続化された IMU 特性の読み出し・書き戻し。
  // 外側タスクから呼ぶ（step と同一スレッドのため排他不要）。
  static void get_bias(float* gyro_bias_z, float* ekf_bias_x,
                       float* ekf_bias_y);
  static void set_bias(float gyro_bias_z, float ekf_bias_x,
                       float ekf_bias_y);
  // 初期位置確定（SET_POSE）に伴う推定リセットを要求する。リンクスレッド
  // から呼び、制御スレッドの step() 先頭で消費される（irq_lock 保護）。
  static void request_reset();
  // 動的推定状態を初期値へ戻す。速度・輪速・相対位置・補助フィルタを
  // リセットし ekf_init_=false とすることで次の静止サンプルで姿勢が
  // 加速度から再シードされる（yaw だけは現行値を保持し、再シードへ
  // 引き継ぐ。加速度からは観測できないため）。学習済みバイアス
  // （gyro_bias_z_ / ekf_bias_xy_）は保持する。
  // keep_qualification=true のとき qualification_time_ も保持する
  // （START_OPERATION 受理時の動作開始リセット用。受理判定で資格
  // 成立を確認済みのため、0.5 s の再証明待ちを挟まない）。
  // 外側タスクから呼ぶ（step と同一スレッドのため排他不要）。
  static void reset_estimate(bool keep_qualification = false);

 private:
  // EKF 内部状態をリセットし、比力からロール/ピッチをシードした姿勢を
  // 返す（yaw は current の値を維持）。初期化・固着復帰で共用。
  static vec4 seed_ekf(const vec3& specific_force, const vec4& current);

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
  static inline float inertial_reject_hold_ = 0.0F;  // 種ズレ固着検出用
  static inline bool reset_req_ = false;  // request_reset の受信箱

  // CALIBRATE_IMU 捕捉（reset_bias で武装）。静止資格ウィンドウの生
  // ジャイロ累積で、連続静止が bias_qualification_time に達した時点で
  // 時間平均をバイアス確定値として採用する。
  static inline vec3 bias_capture_sum_{};
  static inline uint32_t bias_capture_count_ = 0;
  static inline bool bias_capture_active_ = false;
};

}  // namespace bb
