#pragma once

#include <cstdint>

#include "common/vector_math/vector_math.hpp"

namespace bb {

// ballbot_rev1_parameters() のスナップショット。dump_fw_params.m の JSON 出力を
// constexpr 構造体へ転記した。MATLAB 側のフィールド名をそのまま保持する。
struct param_set {
  struct physical {
    float gravity = 9.80665F;
    float ball_radius = 0.050F;
    float ball_mass = 0.285F;
    float ball_inertia11 = 0.000475F;
    float wheel_radius = 0.024F;
    float contact_latitude = 0.5759586531581288F;  // 33 deg
    float contact_preload = 0.0004F;
    float rolling_friction = 0.008F;
    float normal_load_nominal = 11.218703867678462F;
    float contact_torque_limit = 0.2692488928242831F;
  } physical;

  struct motor {
    float torque_constant = 0.3064578125F;
    float back_emf_constant = 0.2617530403350636F;
    float armature_resistance = 1.875F;
    float no_load_speed = 21.991148575128552F;
    float speed_command_limit = 18.325957145940457F;
  } motor;

  struct driver {
    float supply_voltage = 7.2F;
    float max_duty_cycle = 0.8333333333333333F;
    float continuous_torque_limit = 0.9193734375F;
    float bridge_on_resistance = 0.35F;
  } driver;

  struct encoder {
    // モデル基準の分解能（出力軸 1 回転あたりのモデルカウント数）。
    float counts_per_rev = 341.2F;
    float quantum = 0.018414962799471239F;  // 2*pi/counts_per_rev
    // TIM エンコーダモードの x4 デコード係数。生カウント/モデルカウントの換算に
    // 使う。PPR がチャネルあたりの場合は生カウント = モデルカウント×4。
    float quadrature_factor = 4.0F;
  } encoder;

  struct rover {
    float mass = 1.869183885F;
    float center_above_ball = 0.1039999360060391F;
    float com_above_ball = 0.1421691104366836F;
    float chassis_radius = 0.08F;
    vec3 body_inertia = {0.004141030173655482F, 0.004141030173655482F,
                         0.0040234768362113F};
    float tilt_inertia_com = 0.02062288465417033F;
  } rover;

  struct geom {
    mat33 normal_body = {
        {{0.838670567945424F, -0.4193352839727119F, -0.4193352839727124F},
         {0.0F, 0.7263100172470603F, -0.7263100172470602F},
         {0.5446390350150271F, 0.5446390350150271F, 0.5446390350150271F}}};
    mat33 rolling_body = {{{0.0F, -0.8660254037844387F, 0.8660254037844385F},
                           {1.0F, -0.5F, -0.5F},
                           {0.0F, 0.0F, 0.0F}}};
    mat33 axle_body = {
        {{-0.5446390350150271F, 0.2723195175075134F, 0.2723195175075138F},
         {0.0F, -0.4716712402156559F, 0.4716712402156558F},
         {0.838670567945424F, 0.8386705679454238F, 0.8386705679454243F}}};
    mat33 center_from_ball = {
        {{0.06172615380078322F, -0.0308630769003916F, -0.03086307690039164F},
         {0.0F, 0.05345641726938365F, -0.05345641726938364F},
         {0.040085432977106F, 0.040085432977106F, 0.040085432977106F}}};
    mat33 center_from_body = {
        {{0.06172615380078322F, -0.0308630769003916F, -0.03086307690039164F},
         {0.0F, 0.05345641726938365F, -0.05345641726938364F},
         {-0.0639145030289331F, -0.0639145030289331F, -0.0639145030289331F}}};
  } geom;

  struct ekf {
    bool enabled = true;
    bool bias_enabled = true;
    bool use_legacy_correction = false;
    float gyro_noise_density = 0.0001989675347F;
    float accel_noise_density = 0.00073549875F;
    float bias_random_walk = 0.0002F;
    float model_sigma_move = 0.15F;
    float model_sigma_static = 0.05F;
    float accel_filter_time_constant = 0.25F;
    float rel_acc_filter_time_constant = 0.08F;
    float gyro_dot_filter_time_constant = 0.1F;
    float accel_gate = 0.15F;
    float gyro_rate_gate = 0.15F;
    float wheel_slew_filter_time_constant = 0.05F;
    float wheel_slew_gate = 3.0F;
    float inertial_settle_time = 0.3F;
    float max_tilt_step = 0.004F;
    float residual_gate = 1.5F;
    float tilt_var_floor = 2.5e-5F;
    float init_tilt_std = 0.05F;
    float init_bias_std = 0.02F;
  } ekf;

  struct estimator {
    float sample_time = 0.005F;
    float attitude_correction_gain = 2.5F;
    float accel_norm_gate = 2.4516625F;
    float accel_horizontal_gate = 0.03F;
    float velocity_leak = 0.9995F;
    float ball_rate_time_constant = 0.08F;
    float relative_position_leak = 0.9967F;
    float velocity_anchor_time_constant = 0.5F;
    float velocity_anchor_min_speed = 0.05F;
    float stillness_filter_time_constant = 0.15F;
    float ball_rate_still_wheel_rate_filtered = 0.8F;
    float still_velocity_drain_time_constant = 0.1F;
    float still_velocity_drain_hold_time = 1.0F;
    float ball_rate_still_gyro_rate = 0.15F;
    float relative_position_limit = 0.12F;
    float kinematic_regularization = 1e-8F;
    float bias_wheel_rate_threshold = 0.1F;
    float bias_accel_norm_threshold = 0.2941995F;
    float bias_roll_pitch_rate_threshold = 0.02F;
    float bias_yaw_rate_threshold = 0.05F;
    float bias_qualification_time = 0.5F;
    float bias_qualification_hysteresis = 1.25F;
    float bias_time_constant = 1.0F;
    float bias_maximum = 0.1F;
    float bias_maximum_update_per_sample = 0.0001F;
  } estimator;

  struct torque_drive {
    bool enabled = true;
    float slip_damping_gain = 0.02F;
    float slip_deadzone = 4.419591072F;
    float back_emf_speed_time_constant = 0.001F;
    float rate_filter_time_constant = 0.02F;
    float wheel_rate_filter_time_constant = 0.03F;
    float filtered_slip_deadzone = 1.0F;
    float integral_leak = 2.0F;
    float integral_drive_leak = 0.1F;
    float integral_deadzone = 0.02F;
    float integral_command_gate = 0.02F;
    bool integral_enabled = true;
    // MATLAB では isfield でのみ有効。無効時は false。
    bool integral_unwind_floor_enabled = false;
    float integral_unwind_floor = 0.0F;
    float velocity_state_qv = 1000.0F;
    float drag_lean_feedforward = 1.0F;
    float settle_lean_cap = 0.015F;
    float overspeed_coast_margin = 0.12F;
    float overspeed_coast_floor = 0.5F;
    float velocity_lean_gain = 0.3F;
    float lean_setpoint_limit = 0.022F;
    float accel_feedforward_gain = 1.0F;
    float lean_fade_tilt = 0.02F;
    float lean_rate_limit = 0.05F;
    float tilt_stiffen_gain = 2.0F;
    float tilt_stiffen_tilt = 0.006F;
    float wheel_rate_observer_gain = 0.15F;
    float wheel_rate_observer_drive_gain = 306.0F;
    float wheel_rate_observer_compliance = 0.134F;
    float tilt_integral_gain = 1.0F;
    float tilt_integral_limit = 0.03F;
    float tilt_integral_leak = 0.5F;
    float startup_torque_ramp_time = 0.4F;
    float propulsion_gain = 0.02F;
    float propulsion_torque_limit = 0.045F;
    float propulsion_min_share = 0.25F;
    float stop_follow_gain = 0.75F;
    float settle_follow_gain = 1.0F;
    float settle_lean_damping = 1.0F;
    float settle_lean_gain = 1.0F;
    float accel_rel_pos_guard_gain = 12.0F;
    float accel_rel_pos_margin = 0.0F;
    float accel_rel_pos_guard_floor = 0.1F;
    float static_lean_adapt_gain = 0.5F;
    float static_lean_adapt_limit = 0.1F;
    float static_lean_adapt_time_constant = 0.3F;
    float static_lean_adapt_deadband = 0.02F;
    float static_lean_adapt_gate = 0.3F;
    float static_lean_adapt_max_rate = 1.0F;
    float static_lean_adapt_leak = 0.3F;
    float ball_yaw_gain = 0.12F;
    float ball_yaw_torque_limit = 0.025F;
    float yaw_damping_gain = 0.3F;
    bool wheel_rate_observer_enabled = false;
    bool wheel_rate_feedback_enabled = true;
    struct yaw_gate {
      float max_speed = 0.03F;
      float max_tilt = 0.02F;
      float command_threshold = 0.001F;
    } yaw_gate;
    // 3x2 行列。lean_hold_torque(:,i)*demand_i を車輪トルクへ写像する。
    // MATLAB: lean_hold_torque*[acceleration_demand(1);
    // -acceleration_demand(2)]/g
    float lean_hold_torque[3][2] = {{-0.002259759286F, -1.094245997F},
                                    {0.9486184733F, 0.5454573916F},
                                    {-0.9466042491F, 0.5489415641F}};
  } torque_drive;

  struct lqi {
    mat33 wheel_speed_from_planar_velocity = {
        {{0.0F, -22.693293125626127F, 3.333333333333333F},
         {19.65296833583689F, 11.346646562813063F, 3.333333333333333F},
         {-19.65296833583689F, 11.346646562813063F, 3.333333333333333F}}};
    float anti_windup_gain = 8.0F;
    float tracking_gain_scale = 1.0F;
    mat33 state_gain = {};     // legacy speed path: 未使用
    mat33 integral_gain = {};  // legacy speed path: 未使用
  } lqi;

  struct torque_lqi {
    // 3x10 の状態ゲイン（ballbot_velocity_torque_gains 出力）。
    float state_gain[3][10] = {
        {0.0004270727783F, 0.3895562143F, 0.006293553993F, 3.256247369F,
         0.0005010505336F, 0.2590549413F, 0.1014563286F, 0.0F, 0.0F, 0.0F},
        {-0.3374904197F, -0.1945313644F, -2.823260328F, -1.623061294F,
         -0.2246074796F, -0.1291243523F, 0.1015845489F, 0.0F, 0.0F, 0.0F},
        {0.3373009444F, -0.1951315372F, 2.816515328F, -1.633493287F,
         0.224073427F, -0.1299553466F, 0.1012795774F, 0.0F, 0.0F, 0.0F}};
    float integral_gain[3][2] = {{0.0F, 0.0F}, {0.0F, 0.0F}, {0.0F, 0.0F}};
  } torque_lqi;

  struct controller {
    float max_speed = 0.4F;
    float max_yaw_rate = 0.8F;
    float max_planar_acceleration = 0.6F;
    float recovery_tilt = 0.0872664626F;        // 5 deg
    float recovery_exit_tilt = 0.03490658504F;  // 2 deg
    float fallen_tilt = 0.6108652382F;          // 35 deg
    float recovery_gain_scale = 3.0F;
    float recovery_authority_clear_rate = 1.5F;
    float tilt_priority_start = 0.05235987756F;  // 3 deg
    float yaw_bias_ready_rate_threshold = 0.002F;
    float yaw_bias_command_bypass_threshold = 1e-6F;
    bool coupled_command_limit_enabled = true;
    float sample_time = 0.005F;
    float servo_sample_time = 0.001F;
    float traction_limited_wheel_torque = 0.16224F;
    float recovery_wheel_torque_limit = 0.16224F;
    vec2 velocity_integral_limit = {0.2F, 0.2F};
  } controller;

  struct command {
    vec2 velocity_world = {0.0F, 0.0F};
    float yaw_rate = 0.0F;
    bool enable = true;
    bool profile_enabled = false;
    float profile_start = 0.5F;
    float profile_stop = 2.5F;
    vec2 profile_velocity_world = {0.12F, 0.0F};
    // リンク未確立のまま bb start した場合の自動走行は危険なため既定では無効。
    // true にすると cycle プロファイル（移動＋ヨーパルス）を実行する。
    bool cycles_enabled = false;
    float cycle_start = 0.5F;
    float cycle_move_duration = 12.0F;
    float cycle_stop_duration = 4.0F;
    float cycle_yaw_delay = 0.0F;
    float cycle_yaw_duration = 1.0F;
    float cycle_yaw_every = 2.0F;
    float cycle_count = 3.0F;
    float acceleration_limit = 0.06F;
    float deceleration_limit = 0.03F;
    float yaw_pulse_duration = 0.0F;
    float profile_yaw_rate = 0.2F;
  } command;

  // FR-10 位置外ループ（ファーム側の追加パラメータ、MATLAB 非対応）。
  struct position {
    float kp = 1.0F;              // 位置誤差→速度比例ゲイン [1/s]
    float accel_limit = 0.5F;     // 指令速度の変化率上限 [m/s^2]
    float arrive_radius = 0.03F;  // 到達判定半径 [m]
  } position;

  // FR-09 ロボットリンク安全監視 + FR-13/FR-14 運転管理。
  struct link {
    uint32_t watchdog_ms = 500;      // SET_VELOCITY 指令の有効期限
    uint32_t idle_ms = 1000;         // UART 無受信フェイルセーフ
    uint32_t event_batch_ms = 100;   // イベントバッチ送出周期（FR-14）
    uint32_t calib_timeout_ms = 5000;  // CALIBRATE_IMU 完了期限（FR-13）
  } link;

  // FR-16 駆動不感帯補償（車輪ごと・回転方向ごと、既定は
  // omni_drive_test::DEADZONE_COMP と同値）。duty 符号で pos/neg を選択する。
  struct drive {
    float deadzone_comp_pos[3] = {0.08F, 0.08F, 0.08F};  // duty > 0
    float deadzone_comp_neg[3] = {0.08F, 0.08F, 0.08F};  // duty < 0
  } drive;

  // FR-15 IMU 特性（永続化対象。CALIBRATE_IMU の学習結果を保存）。
  struct imu {
    float gyro_bias_z = 0.0F;   // ジャイロ z バイアス [rad/s]
    float ekf_bias_x = 0.0F;    // EKF 推定 x 軸バイアス [rad/s]
    float ekf_bias_y = 0.0F;    // EKF 推定 y 軸バイアス [rad/s]
  } imu;

  // FR-17 同定セッション定数。
  struct tune {
    float wake_rate = 2.0F;     // 輪速立上り判定閾値 [rad/s]
  } tune;
};

// 動作値（param_store の可変コピー）への参照を返す。
// param_store::init_defaults() 呼出し前は constexpr 既定値と同値。
const param_set& get_params();

}  // namespace bb
