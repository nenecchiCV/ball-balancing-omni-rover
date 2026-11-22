#include "feature/position_controller/position_controller.hpp"

#include <math.h>
#include <zephyr/irq.h>

namespace bb {

namespace {

inline float clamp_f(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

}  // namespace

void position_controller::init(const config& cfg) {
  p_ = cfg.params;
  const unsigned int key = irq_lock();
  target_ = target{0.0F, 0.0F, 0.0F, false};
  irq_unlock(key);
  prev_command_ = vec2{};
}

void position_controller::set_target(float x, float y, float speed_limit) {
  const target t{x, y, clamp_f(speed_limit, 0.0F, p_->controller.max_speed),
                 true};
  const unsigned int key = irq_lock();
  target_ = t;
  irq_unlock(key);
  // 目標更新直後は slew をゼロから再開して跳ねを抑える。
  const unsigned int key2 = irq_lock();
  prev_command_ = vec2{};
  irq_unlock(key2);
}

void position_controller::clear_target() {
  const unsigned int key = irq_lock();
  target_ = target{0.0F, 0.0F, 0.0F, false};
  irq_unlock(key);
}

void position_controller::hold_at(float x, float y) {
  const unsigned int key = irq_lock();
  target t = target_;
  irq_unlock(key);
  t.x = x;
  t.y = y;
  t.active = true;
  const unsigned int key2 = irq_lock();
  target_ = t;
  irq_unlock(key2);
}

position_controller::output position_controller::update(float x, float y,
                                                        float dt) {
  const unsigned int key = irq_lock();
  const target t = target_;
  irq_unlock(key);

  output o{};
  o.active = t.active;
  if (!t.active) {
    prev_command_ = vec2{};
    return o;
  }

  const float ex = t.x - x;
  const float ey = t.y - y;
  const float dist = sqrtf(ex * ex + ey * ey);
  if (dist <= p_->position.arrive_radius) {
    o.arrived = true;
    prev_command_ = vec2{};
    return o;
  }

  // 誤差比例 → 速度上限でベクトル飽和。
  float vx = p_->position.kp * ex;
  float vy = p_->position.kp * ey;
  const float v_norm = sqrtf(vx * vx + vy * vy);
  const float v_max = (t.speed_limit < p_->controller.max_speed)
                          ? t.speed_limit
                          : p_->controller.max_speed;
  if (v_norm > v_max && v_norm > 0.0F) {
    vx *= v_max / v_norm;
    vy *= v_max / v_norm;
  }

  // 加速度（slew）制限をベクトルノルムで掛ける。
  float dvx = vx - prev_command_[0];
  float dvy = vy - prev_command_[1];
  const float dv_norm = sqrtf(dvx * dvx + dvy * dvy);
  const float dv_max = p_->position.accel_limit * dt;
  if (dv_norm > dv_max && dv_norm > 0.0F) {
    dvx *= dv_max / dv_norm;
    dvy *= dv_max / dv_norm;
  }
  prev_command_[0] += dvx;
  prev_command_[1] += dvy;

  o.command = {prev_command_[0], prev_command_[1], 0.0F};
  return o;
}

}  // namespace bb
