#include "feature/odometry/odometry.hpp"

#include <math.h>

namespace bb {

// 待機中慣性オドメトリ（検証用途）の ZUPT・バイアス学習定数。
// 表示用で精度要求が緩いためパラメータ化せず実装定数とする。
constexpr float K_ZUPT_HOLD_TIME = 0.3F;  // ZUPT 確定までの静止継続 [s]
constexpr float K_ACCEL_BIAS_TIME_CONSTANT =
    2.0F;  // 加速度バイアス学習時定数 [s]

// 動作開始直後のオドメトリ settle 保持・積分速度包絡の定数。
// ボールへの配置トランジェント中の幻影速度を捨てるための保持時間と、
// 物理的にあり得ない推定速度を飽和する包絡で、いずれも設計定数とする。
constexpr float K_SETTLE_HOLD_TIME = 1.0F;   // settle 保持の上限 [s]
constexpr float K_SETTLE_STILL_TIME = 0.2F;  // still 継続による早期解除 [s]
// 速度包絡 [m/s]: 理論上限 ~0.9（wheel_radius_effective×
// speed_command_limit/sinλ、param_store::derive 相当）に過渡余裕を
// 載せた値。これを超える推定速度は実運動ではなく幻影とみなす。
constexpr float K_ODOM_SPEED_LIMIT = 1.0F;

void odometry::init() {
  raw_x_ = 0.0F;
  raw_y_ = 0.0F;
  origin_x_ = 0.0F;
  origin_y_ = 0.0F;
  yaw_offset_ = 0.0F;
  decl_x_ = 0.0F;
  decl_y_ = 0.0F;
  decl_yaw_ = 0.0F;
  inertial_vx_ = 0.0F;
  inertial_vy_ = 0.0F;
  accel_bias_x_ = 0.0F;
  accel_bias_y_ = 0.0F;
  still_time_ = 0.0F;
  settle_left_ = 0.0F;
  settle_still_ = 0.0F;
}

void odometry::arm_settle_hold() {
  settle_left_ = K_SETTLE_HOLD_TIME;
  settle_still_ = 0.0F;
}

void odometry::update(float vx_w, float vy_w, bool still, float dt) {
  if (settle_left_ > 0.0F) {
    // settle 保持: 宣言位置へ張り付き、配置トランジェントの幻影速度を
    // 積分しない。still（水平加速度残差小＝姿勢収束済み）が継続すれば
    // 早期解除し、上限時間でも解除する。
    settle_left_ -= dt;
    settle_still_ = still ? settle_still_ + dt : 0.0F;
    if (settle_left_ <= 0.0F || settle_still_ >= K_SETTLE_STILL_TIME) {
      settle_left_ = 0.0F;
    }
  } else {
    const float vn = sqrtf(vx_w * vx_w + vy_w * vy_w);
    if (vn > K_ODOM_SPEED_LIMIT) {
      const float s = K_ODOM_SPEED_LIMIT / vn;
      vx_w *= s;
      vy_w *= s;
    }
    raw_x_ += vx_w * dt;
    raw_y_ += vy_w * dt;
  }
  // 動作中は待機用慣性速度を無効化する（次回待機突入時に張り残さない。
  // 加速度バイアスは学習済み値として跨いで保持してよい）。
  inertial_vx_ = 0.0F;
  inertial_vy_ = 0.0F;
  still_time_ = 0.0F;
}

void odometry::update_inertial(float ax_w, float ay_w, bool still, float dt) {
  if (still) {
    still_time_ += dt;
    if (still_time_ >= K_ZUPT_HOLD_TIME) {
      inertial_vx_ = 0.0F;
      inertial_vy_ = 0.0F;
      const float w = dt / (K_ACCEL_BIAS_TIME_CONSTANT + dt);
      accel_bias_x_ += w * (ax_w - accel_bias_x_);
      accel_bias_y_ += w * (ay_w - accel_bias_y_);
    }
  } else {
    still_time_ = 0.0F;
    inertial_vx_ += (ax_w - accel_bias_x_) * dt;
    inertial_vy_ += (ay_w - accel_bias_y_) * dt;
  }
  raw_x_ += inertial_vx_ * dt;
  raw_y_ += inertial_vy_ * dt;
}

odometry::pose odometry::inertial_velocity() {
  return {inertial_vx_, inertial_vy_};
}

odometry::pose odometry::raw() {
  return {raw_x_, raw_y_};
}

odometry::pose odometry::get() {
  return {raw_x_ - origin_x_, raw_y_ - origin_y_};
}

float odometry::yaw_of(float raw_yaw) {
  return raw_yaw - yaw_offset_;
}

void odometry::reset_origin(float raw_x, float raw_y, float raw_yaw) {
  origin_x_ = raw_x;
  origin_y_ = raw_y;
  yaw_offset_ = raw_yaw;
  // 原点宣言は (0,0,0) の宣言位置とみなす。
  decl_x_ = 0.0F;
  decl_y_ = 0.0F;
  decl_yaw_ = 0.0F;
}

void odometry::set_pose(float x, float y, float yaw, float raw_x, float raw_y,
                        float raw_yaw) {
  origin_x_ = raw_x - x;
  origin_y_ = raw_y - y;
  yaw_offset_ = raw_yaw - yaw;
  decl_x_ = x;
  decl_y_ = y;
  decl_yaw_ = yaw;
}

void odometry::reapply_declared(float raw_x, float raw_y, float raw_yaw) {
  set_pose(decl_x_, decl_y_, decl_yaw_, raw_x, raw_y, raw_yaw);
}

}  // namespace bb
