#include "feature/state_estimator/state_estimator.hpp"

#include <etl/algorithm.h>
#include <math.h>
#include <zephyr/irq.h>

namespace bb {

namespace {

inline float hypot2(float a, float b) {
  return sqrtf(a * a + b * b);
}

// 静止証明済みなのに残差がゲート超過し続ける場合の再シード待ち時間 [s]。
constexpr float K_RESEED_HOLD_TIME = 2.0F;

// 待機中慣性オドメトリの静止候補とみなす水平加速度残差上限 [m/s²]。
// 手持ち微振（~0.2–0.4）と真の静止残差（~0.1）の間に置く。
constexpr float K_STILL_ODOM_ACCEL_XY = 0.3F;

// 推定水平速度の物理包絡 [m/s]。車輪飽和由来の理論上限 ~0.9
// （param_store::derive の theory_max_speed 相当）を大きく超える推定値は
// 実運動ではなく幻影（姿勢残差の重力混入・非接触時のアンカー注入等）と
// みなして飽和する。幻影速度が leak 減衰を待たずオドメトリ・制御器へ
// 残り続けるのを防ぐ。制御器はこの速度をフィードバックに使うため、
// 実運動をクリップしないよう理論上限に対し余裕を持たせる。
constexpr float K_VELOCITY_WORLD_LIMIT = 1.5F;

// 静止時の比力（≈重力の反対向き）からロール/ピッチを定めた姿勢を作る。
// yaw は current の値を維持する（yaw は加速度から観測できない）。
vec4 tilt_quat_from_accel(const vec3& specific_force_body,
                          const vec4& current) {
  const float an = etl::max(norm(specific_force_body), 1e-6F);
  const vec3 u = specific_force_body * (1.0F / an);  // body 系の +z_world
  // u → ez への最短回転: q = [1 + u_z, u×ez]（u_z≈-1 は退化するので
  // 水平軸の 180° 回転を選ぶ）
  vec4 tilt = u[2] > -0.9999F
                  ? vec4{1.0F + u[2], u[1], -u[0], 0.0F}
                  : vec4{0.0F, 1.0F, 0.0F, 0.0F};
  const float n = etl::max(norm(tilt), 1e-6F);
  for (auto& v : tilt) v /= n;
  const float yaw = rotation_to_euler(quaternion_to_rotation(current))[2];
  const vec4 qy = {cosf(0.5F * yaw), 0.0F, 0.0F, sinf(0.5F * yaw)};
  return quaternion_multiply(qy, tilt);
}

}  // namespace

void state_estimator::init(const config& cfg) {
  p_ = cfg.params;
  reset_estimate();
  gyro_bias_z_ = 0.0F;
  ekf_bias_xy_ = vec2{};
  bias_capture_active_ = false;
  bias_capture_sum_ = vec3{};
  bias_capture_count_ = 0;
}

void state_estimator::request_reset() {
  const unsigned int key = irq_lock();
  reset_req_ = true;
  irq_unlock(key);
}

void state_estimator::reset_estimate(bool keep_qualification) {
  // 再シードは yaw を current から引き継ぐため、ここで yaw 成分のみ
  // 残したクォータニオンへ置き換える（リセット前後で yaw が跳ばず、
  // オドメトリの yaw オフセットとの整合が保たれる）。ゼロ初期状態では
  // quaternion_to_rotation 内の norm クランプにより恒等相当となる。
  const float yaw = rotation_to_euler(quaternion_to_rotation(quaternion_))[2];
  quaternion_ = {cosf(0.5F * yaw), 0.0F, 0.0F, sinf(0.5F * yaw)};
  velocity_world_ = vec3{};
  ball_rate_world_ = vec3{};
  relative_position_body_ = vec2{};
  if (!keep_qualification) {
    qualification_time_ = 0.0F;
  }
  ekf_covariance_ = mat55{};
  ekf_accel_world_ = vec3{};
  prev_ball_velocity_world_ = vec3{};
  ekf_init_ = false;
  filtered_wheel_rate_for_still_ = vec3{};
  still_init_ = false;
  stillness_hold_init_ = false;
  stillness_hold_ = 0.0F;
  ekf_rel_vel_body_ = vec2{};
  ekf_rel_acc_body_ = vec2{};
  ekf_rel_init_ = false;
  ekf_gyro_dot_body_ = vec3{};
  ekf_prev_gyro_body_ = vec3{};
  ekf_gyro_init_ = false;
  ekf_wheel_slew_ = 0.0F;
  ekf_prev_wheel_rate_ = vec3{};
  ekf_slew_init_ = false;
  inertial_hold_ = 0.0F;
  inertial_reject_hold_ = 0.0F;
  reset_req_ = false;
}

vec4 state_estimator::seed_ekf(const vec3& specific_force,
                               const vec4& current) {
  const auto& ek = p_->ekf;
  // ekf_bias_xy_ はシードし直さない: 永続化値・学習値はそのままの
  // 初期推定として残す（ゼロにすると CALIBRATE_IMU 成果が失われる）。
  ekf_accel_world_ = vec3{};
  prev_ball_velocity_world_ = vec3{};
  ekf_covariance_ = mat55{};
  for (int i = 0; i < 3; ++i)
    ekf_covariance_[i][i] = ek.init_tilt_std * ek.init_tilt_std;
  for (int i = 3; i < 5; ++i)
    ekf_covariance_[i][i] = ek.init_bias_std * ek.init_bias_std;
  return tilt_quat_from_accel(specific_force, current);
}

void state_estimator::reset_bias() {
  gyro_bias_z_ = 0.0F;
  ekf_bias_xy_ = vec2{};
  qualification_time_ = 0.0F;
  bias_capture_active_ = true;
  bias_capture_sum_ = vec3{};
  bias_capture_count_ = 0;
}

void state_estimator::get_bias(float* gyro_bias_z, float* ekf_bias_x,
                               float* ekf_bias_y) {
  if (gyro_bias_z != nullptr) *gyro_bias_z = gyro_bias_z_;
  if (ekf_bias_x != nullptr) *ekf_bias_x = ekf_bias_xy_[0];
  if (ekf_bias_y != nullptr) *ekf_bias_y = ekf_bias_xy_[1];
}

void state_estimator::set_bias(float gyro_bias_z, float ekf_bias_x,
                               float ekf_bias_y) {
  gyro_bias_z_ = gyro_bias_z;
  ekf_bias_xy_ = {ekf_bias_x, ekf_bias_y};
}

state_estimator::output state_estimator::step(const imu_sample& imu,
                                              const vec3& wheel_rate,
                                              float dt) {
  // SET_POSE 由来の推定リセット要求を消費（リンクスレッド→制御スレッド）。
  {
    bool do_reset = false;
    const unsigned int key = irq_lock();
    do_reset = reset_req_;
    reset_req_ = false;
    irq_unlock(key);
    if (do_reset) reset_estimate();
  }
  const auto& e = p_->estimator;
  const auto& ek = p_->ekf;
  output out{};

  vec4 quaternion = quaternion_;
  vec3 velocity_world = velocity_world_;
  const vec3 ball_rate_world_previous = ball_rate_world_;
  const vec2 relative_position_body_previous = relative_position_body_;
  const float gyro_bias_z_previous = gyro_bias_z_;
  const float qualification_time_previous = qualification_time_;

  const vec3 specific_force_body = imu.specific_force;
  const vec3 gyro_body_raw = imu.gyro;

  const bool ekf_enabled = ek.enabled;
  if (ekf_enabled && !ekf_init_) {
    // 比力が重力と整合（≈静止）するサンプルが来るまで初期化を遅延し、
    // その方向からロール/ピッチをシードする。床置き等の大傾斜起動でも
    // 初期残差がゲート内に収まり、以後の加速度更新が機能する。
    // 静止サンプルが得られない間は従来どおり予測のみ（共分散ゼロのため
    // 加速度更新も実質無効）。
    if (imu.valid &&
        fabsf(norm(specific_force_body) - p_->physical.gravity) <=
            ek.residual_gate) {
      quaternion = seed_ekf(specific_force_body, quaternion);
      ekf_init_ = true;
    }
  }

  vec3 gyro_body = gyro_body_raw;
  if (ekf_enabled) {
    const float scale = ek.bias_enabled ? 1.0F : 0.0F;
    gyro_body[0] -= ekf_bias_xy_[0] * scale;
    gyro_body[1] -= ekf_bias_xy_[1] * scale;
    gyro_body[2] -= gyro_bias_z_previous;
  } else {
    gyro_body[2] -= gyro_bias_z_previous;
  }

  const float acceleration_norm = norm(specific_force_body);
  const bool quasi_static =
      max_abs(wheel_rate) <= e.bias_wheel_rate_threshold &&
      hypot2(gyro_body[0], gyro_body[1]) <= e.bias_roll_pitch_rate_threshold &&
      fabsf(gyro_body[2]) <= e.bias_yaw_rate_threshold;

  mat33 rotation_world_from_body{};
  if (ekf_enabled) {
    // EKF 予測: quaternion 積分 + 共分散伝播
    quaternion = integrate_quaternion(quaternion, gyro_body, dt);
    rotation_world_from_body = quaternion_to_rotation(quaternion);
    mat55 f = mat_identity5();
    const mat33 sw = skew(gyro_body);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        f[i][j] = (i == j ? 1.0F : 0.0F) - dt * sw[i][j];
    f[0][3] = -dt;
    f[1][4] = -dt;
    // P = F P F' + Q
    mat55 q{};
    for (int i = 0; i < 3; ++i)
      q[i][i] = ek.gyro_noise_density * ek.gyro_noise_density * dt;
    for (int i = 3; i < 5; ++i)
      q[i][i] = ek.bias_random_walk * ek.bias_random_walk * dt;
    ekf_covariance_ = mat_add(mul(mul(f, ekf_covariance_), transpose(f)), q);
  } else {
    // レガシー補正
    rotation_world_from_body = quaternion_to_rotation(quaternion);
    vec3 correction{};
    if (fabsf(acceleration_norm - p_->physical.gravity) <= e.accel_norm_gate &&
        quasi_static) {
      const vec3 measured_up_body =
          specific_force_body * (1.0F / etl::max(acceleration_norm, 1e-12F));
      const vec3 predicted_up_body = {rotation_world_from_body[0][2],
                                      rotation_world_from_body[1][2],
                                      rotation_world_from_body[2][2]};
      correction = e.attitude_correction_gain *
                   cross(measured_up_body, predicted_up_body);
      const float cm = norm(correction);
      if (cm > 0.5F) correction = correction * (0.5F / cm);
    }
    const vec3 corrected_rate = gyro_body + correction;
    quaternion = integrate_quaternion(quaternion, corrected_rate, dt);
    rotation_world_from_body = quaternion_to_rotation(quaternion);
  }

  const vec3 gravity_world = {0.0F, 0.0F, -p_->physical.gravity};
  const vec3 acceleration_world =
      mul(rotation_world_from_body, specific_force_body) + gravity_world;
  velocity_world = velocity_world * e.velocity_leak + acceleration_world * dt;
  velocity_world[2] = 0.0F;

  const mat33& normal_body = p_->geom.normal_body;
  const mat33& rolling_body = p_->geom.rolling_body;
  const mat33& center_from_body = p_->geom.center_from_body;
  const vec3 body_rate_world = mul(rotation_world_from_body, gyro_body);
  mat33 normal_world = mat_mul(rotation_world_from_body, normal_body);
  mat33 rolling_world = mat_mul(rotation_world_from_body, rolling_body);
  mat33 wheel_center_world =
      mat_mul(rotation_world_from_body, center_from_body);

  // 静止証人: フィルタ済み輪速で判定
  if (!still_init_) {
    filtered_wheel_rate_for_still_ = wheel_rate;
    still_init_ = true;
  } else {
    const float w_still = dt / (e.stillness_filter_time_constant + dt);
    filtered_wheel_rate_for_still_ =
        filtered_wheel_rate_for_still_ +
        w_still * (wheel_rate - filtered_wheel_rate_for_still_);
  }

  const float ball_radius = p_->physical.ball_radius;
  const mat33 ground_rolling = {{{0.0F, ball_radius, 0.0F},
                                 {-ball_radius, 0.0F, 0.0F},
                                 {0.0F, 0.0F, 0.0F}}};
  // kinematic_matrix (3x3), right side (3)
  mat33 kin{};
  vec3 rhs{};
  for (int w = 0; w < 3; ++w) {
    const vec3 n_w = {normal_world[0][w], normal_world[1][w],
                      normal_world[2][w]};
    const vec3 r_w = {rolling_world[0][w], rolling_world[1][w],
                      rolling_world[2][w]};
    const vec3 c_w = {wheel_center_world[0][w], wheel_center_world[1][w],
                      wheel_center_world[2][w]};
    // surface_velocity_matrix = ground_rolling - ball_radius*skew(n_w)
    mat33 surf = ground_rolling;
    const mat33 sn = skew(n_w);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) surf[i][j] -= ball_radius * sn[i][j];
    // kin(w,:) = r_w' * surf
    for (int j = 0; j < 3; ++j)
      kin[w][j] =
          r_w[0] * surf[0][j] + r_w[1] * surf[1][j] + r_w[2] * surf[2][j];
    const vec3 wc_vel = velocity_world + cross(body_rate_world, c_w);
    rhs[w] = dot(r_w, wc_vel) -
             p_->physical.wheel_radius_effective * wheel_rate[w];
  }
  // ball_rate_kinematic = (K'K + reg I) \ K' rhs
  mat33 kk{};
  vec3 ktr{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      kk[i][j] =
          kin[0][i] * kin[0][j] + kin[1][i] * kin[1][j] + kin[2][i] * kin[2][j];
    ktr[i] = kin[0][i] * rhs[0] + kin[1][i] * rhs[1] + kin[2][i] * rhs[2];
    kk[i][i] += e.kinematic_regularization;
  }
  vec3 ball_rate_kinematic = mul(inverse3(kk), ktr);

  const bool assembly_still = max_abs(filtered_wheel_rate_for_still_) <=
                                  e.ball_rate_still_wheel_rate_filtered &&
                              norm(gyro_body) <= e.ball_rate_still_gyro_rate;
  // 待機中慣性オドメトリ（ZUPT）用の静止候補。車輪・ジャイロに加えて
  // 水平加速度残差も小さいときだけ立てる（等速搬送は検知不可）。
  const bool still =
      assembly_still && hypot2(acceleration_world[0], acceleration_world[1]) <=
                            K_STILL_ODOM_ACCEL_XY;
  if (!stillness_hold_init_) {
    stillness_hold_ = 0.0F;
    stillness_hold_init_ = true;
  }
  stillness_hold_ = assembly_still ? (stillness_hold_ + dt) : 0.0F;
  if (stillness_hold_ >= e.still_velocity_drain_hold_time) {
    ball_rate_kinematic = vec3{};
  }

  const float filter_weight = dt / (e.ball_rate_time_constant + dt);
  const vec3 ball_rate_world =
      ball_rate_world_previous +
      filter_weight * (ball_rate_kinematic - ball_rate_world_previous);
  const vec3 ball_velocity_world = mul(ground_rolling, ball_rate_world);

  if (ekf_enabled) {
    // 運動モデル加速度（ボール速度微分 + 相対加速度 + co_m スウェイ）
    const float accel_tau = ek.accel_filter_time_constant;
    const vec3 ball_accel_raw =
        (ball_velocity_world - prev_ball_velocity_world_) * (1.0F / dt);
    prev_ball_velocity_world_ = ball_velocity_world;
    const float accel_weight = dt / (accel_tau + dt);
    ekf_accel_world_ =
        ekf_accel_world_ + accel_weight * (ball_accel_raw - ekf_accel_world_);

    if (!ekf_rel_init_) {
      ekf_rel_vel_body_ = vec2{};
      ekf_rel_acc_body_ = vec2{};
      ekf_rel_init_ = true;
    }
    // rel_vel_body = R(1:2,1:2)' * (v(1:2) - v_ball(1:2))
    const float dxv = velocity_world[0] - ball_velocity_world[0];
    const float dyv = velocity_world[1] - ball_velocity_world[1];
    const vec2 rel_vel_now = {rotation_world_from_body[0][0] * dxv +
                                  rotation_world_from_body[1][0] * dyv,
                              rotation_world_from_body[0][1] * dxv +
                                  rotation_world_from_body[1][1] * dyv};
    const vec2 rel_acc_raw = (rel_vel_now - ekf_rel_vel_body_) * (1.0F / dt);
    ekf_rel_vel_body_ = rel_vel_now;
    const float rel_acc_weight = dt / (ek.rel_acc_filter_time_constant + dt);
    ekf_rel_acc_body_ =
        ekf_rel_acc_body_ + rel_acc_weight * (rel_acc_raw - ekf_rel_acc_body_);
    const vec3 rel_acc_world =
        mul(rotation_world_from_body,
            {ekf_rel_acc_body_[0], ekf_rel_acc_body_[1], 0.0F});

    if (!ekf_gyro_init_) {
      ekf_gyro_dot_body_ = vec3{};
      ekf_prev_gyro_body_ = gyro_body;
      ekf_gyro_init_ = true;
    }
    const vec3 gyro_dot_raw = (gyro_body - ekf_prev_gyro_body_) * (1.0F / dt);
    ekf_prev_gyro_body_ = gyro_body;
    const float gyro_dot_weight = dt / (ek.gyro_dot_filter_time_constant + dt);
    ekf_gyro_dot_body_ = ekf_gyro_dot_body_ +
                         gyro_dot_weight * (gyro_dot_raw - ekf_gyro_dot_body_);
    const vec3 gyro_dot_world =
        mul(rotation_world_from_body, ekf_gyro_dot_body_);
    const vec3 com_sway_world =
        p_->rover.center_above_ball *
        vec3{gyro_dot_world[1], -gyro_dot_world[0], 0.0F};
    const vec3 accel_prediction =
        ekf_accel_world_ + rel_acc_world + com_sway_world;

    if (!ekf_slew_init_) {
      ekf_wheel_slew_ = 0.0F;
      ekf_prev_wheel_rate_ = wheel_rate;
      ekf_slew_init_ = true;
    }
    float wheel_slew_raw = 0.0F;
    for (int i = 0; i < 3; ++i)
      wheel_slew_raw = etl::max(wheel_slew_raw,
                                fabsf(wheel_rate[i] - ekf_prev_wheel_rate_[i]));
    wheel_slew_raw /= dt;
    ekf_prev_wheel_rate_ = wheel_rate;
    const float slew_weight = dt / (ek.wheel_slew_filter_time_constant + dt);
    ekf_wheel_slew_ =
        ekf_wheel_slew_ + slew_weight * (wheel_slew_raw - ekf_wheel_slew_);

    const bool inertial_now =
        norm(accel_prediction) <= ek.accel_gate &&
        hypot2(gyro_body[0], gyro_body[1]) <= ek.gyro_rate_gate &&
        ekf_wheel_slew_ <= ek.wheel_slew_gate;

    inertial_hold_ =
        (quasi_static || inertial_now) ? (inertial_hold_ + dt) : 0.0F;
    const bool update_allowed = inertial_hold_ >= ek.inertial_settle_time;
    const float model_var = quasi_static
                                ? ek.model_sigma_static * ek.model_sigma_static
                                : ek.model_sigma_move * ek.model_sigma_move;

    const vec3 expected_force_world = {0.0F, 0.0F, p_->physical.gravity};
    const mat33 rt = transpose(rotation_world_from_body);
    const vec3 predicted_force_body = mul(rt, expected_force_world);
    const vec3 residual = specific_force_body - predicted_force_body;
    const bool residual_ok = norm(residual) <= ek.residual_gate;

    if (update_allowed && residual_ok) {
      // H = [skew(predicted_force_body), 0(3x2)]
      const mat33 h = skew(predicted_force_body);
      const mat33 ht = transpose(h);
      // S = H P H' + R = h * ptt * h' + R
      mat33 ptt{};
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) ptt[i][j] = ekf_covariance_[i][j];
      const float accel_var =
          ek.accel_noise_density * sqrtf(1.0F / (2.0F * dt));
      const float meas_var = accel_var * accel_var + model_var;
      mat33 s = mat_mul(mat_mul(h, ptt), ht);
      for (int i = 0; i < 3; ++i) s[i][i] += meas_var;
      const mat33 sinv = inverse3(s);
      // K = P H' S^-1 ; P H' は P(:,0:2) * h'
      float kgain[5][3]{};
      for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 3; ++j) {
          float acc = 0.0F;
          for (int k = 0; k < 3; ++k) acc += ekf_covariance_[i][k] * ht[k][j];
          kgain[i][j] = acc;
        }
      float kf[5][3]{};
      for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 3; ++j) {
          float acc = 0.0F;
          for (int k = 0; k < 3; ++k) acc += kgain[i][k] * sinv[k][j];
          kf[i][j] = acc;
        }
      // delta_state = K r (5)
      vec5 delta{};
      for (int i = 0; i < 5; ++i)
        delta[i] = kf[i][0] * residual[0] + kf[i][1] * residual[1] +
                   kf[i][2] * residual[2];
      // 傾斜ステップのクランプ
      const float tilt_step_norm = sqrtf(
          delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
      if (tilt_step_norm > ek.max_tilt_step) {
        const float s2 = ek.max_tilt_step / tilt_step_norm;
        delta[0] *= s2;
        delta[1] *= s2;
        delta[2] *= s2;
      }
      // 注入: delta quat を乗算、bias を吸収
      vec4 dq = {1.0F, 0.5F * delta[0], 0.5F * delta[1], 0.5F * delta[2]};
      const float dq_n = etl::max(norm(dq), 1e-30F);
      for (auto& v : dq) v /= dq_n;
      quaternion = quaternion_multiply(quaternion, dq);
      const float bias_scale = ek.bias_enabled ? 1.0F : 0.0F;
      ekf_bias_xy_[0] += delta[3] * bias_scale;
      ekf_bias_xy_[1] += delta[4] * bias_scale;
      // joseph 形: P = (I-KH) P (I-KH)' + K R K'
      // KH = K(5x3) * H(3x5) -> 5x5 （列 3,4 は 0）
      float ikh[5][5]{};
      for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
          float v = (i == j) ? 1.0F : 0.0F;
          if (j < 3)
            v -= kf[i][0] * h[0][j] + kf[i][1] * h[1][j] + kf[i][2] * h[2][j];
          ikh[i][j] = v;
        }
      }
      // K R K' （R = meas_var*I3）: (K K') * meas_var
      mat55 p_new{};
      // tmp = IKH * P
      float tmp[5][5]{};
      for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
          float acc = 0.0F;
          for (int k = 0; k < 5; ++k) acc += ikh[i][k] * ekf_covariance_[k][j];
          tmp[i][j] = acc;
        }
      for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
          float acc = 0.0F;
          for (int k = 0; k < 5; ++k) acc += tmp[i][k] * ikh[j][k];
          // K R K' 項
          float krk = 0.0F;
          for (int k = 0; k < 3; ++k) krk += kf[i][k] * kf[j][k];
          p_new[i][j] = acc + meas_var * krk;
        }
      ekf_covariance_ = p_new;
      for (int i = 0; i < 3; ++i)
        ekf_covariance_[i][i] =
            etl::max(ekf_covariance_[i][i], ek.tilt_var_floor);
      rotation_world_from_body = quaternion_to_rotation(quaternion);
      inertial_reject_hold_ = 0.0F;
    } else if (update_allowed && imu.valid) {
      // 静止証明済みなのに残差がゲート超過し続ける＝姿勢の種ズレ固着
      // （大傾斜起動・運搬中のドリフト蓄積等）。加速度計でロール/ピッチを
      // 再シードしてフィルタを復帰させる。
      inertial_reject_hold_ += dt;
      if (inertial_reject_hold_ >= K_RESEED_HOLD_TIME) {
        quaternion = seed_ekf(specific_force_body, quaternion);
        rotation_world_from_body = quaternion_to_rotation(quaternion);
        inertial_reject_hold_ = 0.0F;
      }
    } else {
      inertial_reject_hold_ = 0.0F;
    }
  }

  // 速度アンカー / 静止ドレイン
  const float anchor_min_speed = e.velocity_anchor_min_speed;
  if (stillness_hold_ >= e.still_velocity_drain_hold_time) {
    const float drain_tau = e.still_velocity_drain_time_constant;
    const float keep = drain_tau / (drain_tau + dt);
    velocity_world[0] *= keep;
    velocity_world[1] *= keep;
  } else if (e.velocity_anchor_time_constant > 0.0F &&
             norm(ball_velocity_world) > anchor_min_speed) {
    const float anchor_weight = dt / (e.velocity_anchor_time_constant + dt);
    velocity_world[0] +=
        anchor_weight * (ball_velocity_world[0] - velocity_world[0]);
    velocity_world[1] +=
        anchor_weight * (ball_velocity_world[1] - velocity_world[1]);
  }

  // 水平速度の物理包絡: 理論上限を大きく超える推定値は幻影とみなして
  // 等比飽和する。アンカー/ドレイン適用後の最終値に掛けることで、
  // 保持状態・出力・相対速度のすべてを bounded にする。
  const float velocity_xy = hypot2(velocity_world[0], velocity_world[1]);
  if (velocity_xy > K_VELOCITY_WORLD_LIMIT) {
    const float s = K_VELOCITY_WORLD_LIMIT / velocity_xy;
    velocity_world[0] *= s;
    velocity_world[1] *= s;
  }

  const vec3 relative_velocity_world = velocity_world - ball_velocity_world;
  const vec3 yaw_e = rotation_to_euler(rotation_world_from_body);
  const float cy = cosf(yaw_e[2]);
  const float sy = sinf(yaw_e[2]);
  const vec2 rel_pos_world_prev = {cy * relative_position_body_previous[0] -
                                       sy * relative_position_body_previous[1],
                                   sy * relative_position_body_previous[0] +
                                       cy * relative_position_body_previous[1]};
  vec2 relative_position_world =
      rel_pos_world_prev * e.relative_position_leak +
      vec2{relative_velocity_world[0] * dt, relative_velocity_world[1] * dt};
  const float rel_pos_norm =
      sqrtf(relative_position_world[0] * relative_position_world[0] +
            relative_position_world[1] * relative_position_world[1]);
  if (rel_pos_norm > e.relative_position_limit) {
    relative_position_world =
        relative_position_world * (e.relative_position_limit / rel_pos_norm);
  }
  const vec2 relative_position_body = {
      cy * relative_position_world[0] + sy * relative_position_world[1],
      -sy * relative_position_world[0] + cy * relative_position_world[1]};

  // ヨーバイアス資格化と学習
  const bool qualified_previously =
      qualification_time_previous >= e.bias_qualification_time;
  const float threshold_scale =
      qualified_previously ? e.bias_qualification_hysteresis : 1.0F;
  const bool qualification_candidate =
      max_abs(wheel_rate) <= threshold_scale * e.bias_wheel_rate_threshold &&
      fabsf(acceleration_norm - p_->physical.gravity) <=
          threshold_scale * e.bias_accel_norm_threshold &&
      hypot2(gyro_body[0], gyro_body[1]) <=
          threshold_scale * e.bias_roll_pitch_rate_threshold &&
      fabsf(gyro_body[2]) <= threshold_scale * e.bias_yaw_rate_threshold;

  float qualification_time = qualification_time_previous;
  if (qualification_candidate) {
    qualification_time =
        etl::min(qualification_time + dt, e.bias_qualification_time);
  } else {
    qualification_time = 0.0F;
  }

  const bool bias_learning_enabled =
      qualification_candidate && qualified_previously;
  const float bias_filter_weight = dt / (e.bias_time_constant + dt);
  float bias_update = (bias_learning_enabled ? 1.0F : 0.0F) *
                      bias_filter_weight *
                      (gyro_body_raw[2] - gyro_bias_z_previous);
  bias_update = etl::clamp(bias_update, -e.bias_maximum_update_per_sample,
                           e.bias_maximum_update_per_sample);
  float gyro_bias_z = etl::clamp(gyro_bias_z_previous + bias_update,
                                 -e.bias_maximum, e.bias_maximum);

  // CALIBRATE_IMU 捕捉（reset_bias で武装）: 連続した静止・有効サンプル
  // の生ジャイロ時間平均をバイアス確定値として採用する。単一サンプル
  // ではなく区間平均とし、失格・無効サンプルでウィンドウを取り直す。
  // 採用後も IIR/EKF による継続学習はそのまま進行する。
  if (bias_capture_active_) {
    if (qualification_candidate && imu.valid) {
      bias_capture_sum_ = bias_capture_sum_ + gyro_body_raw;
      ++bias_capture_count_;
    } else {
      bias_capture_sum_ = vec3{};
      bias_capture_count_ = 0;
    }
    if (bias_capture_count_ > 0 &&
        static_cast<float>(bias_capture_count_) * dt >=
            e.bias_qualification_time) {
      const vec3 mean =
          bias_capture_sum_ * (1.0F / static_cast<float>(bias_capture_count_));
      ekf_bias_xy_ = {mean[0], mean[1]};
      gyro_bias_z = etl::clamp(mean[2], -e.bias_maximum, e.bias_maximum);
      bias_capture_active_ = false;
      bias_capture_sum_ = vec3{};
      bias_capture_count_ = 0;
    }
  }

  // 出力
  quaternion_ = quaternion;
  velocity_world_ = velocity_world;
  ball_rate_world_ = ball_rate_world;
  relative_position_body_ = relative_position_body;
  gyro_bias_z_ = gyro_bias_z;
  qualification_time_ = qualification_time;

  out.quaternion = quaternion;
  out.velocity_world = velocity_world;
  out.ball_rate_world = ball_rate_world;
  out.relative_position_body = relative_position_body;
  out.gyro_bias_z = gyro_bias_z;
  out.qualification_time = qualification_time;
  out.euler = yaw_e;
  out.gyro_body = gyro_body;
  out.velocity_world2 = {velocity_world[0], velocity_world[1]};
  out.rel_pos_body2 = relative_position_body;
  out.accel_world = acceleration_world;
  out.still = still;
  out.updated_gyro_bias_z = gyro_bias_z;
  out.bias_learning_enabled = bias_learning_enabled;
  out.updated_qualification_time = qualification_time;
  out.valid = imu.valid;
  return out;
}

}  // namespace bb
