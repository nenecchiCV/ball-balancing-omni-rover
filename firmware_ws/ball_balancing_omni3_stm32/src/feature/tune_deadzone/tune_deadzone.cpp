#include "feature/tune_deadzone/tune_deadzone.hpp"

#include <math.h>
#include <zephyr/irq.h>

#include <etl/algorithm.h>

namespace bb {

void tune_deadzone::init(const config& cfg) {
  p_ = cfg.params;
  const unsigned int key = irq_lock();
  running_ = false;
  done_ = false;
  start_req_ = false;
  cancel_req_ = false;
  meas_ = measure{};
  irq_unlock(key);
}

bool tune_deadzone::request_start(float dir, float max_duty, float ramp_s) {
  const unsigned int key = irq_lock();
  if (running_ || start_req_) {
    irq_unlock(key);
    return false;
  }
  dir_ = (dir >= 0.0F) ? 1.0F : -1.0F;
  max_duty_ = etl::clamp(max_duty, 0.01F, CLAMP_DUTY);
  ramp_s_ = etl::clamp(ramp_s, 1.0F, CLAMP_RAMP_S);
  start_req_ = true;
  irq_unlock(key);
  return true;
}

void tune_deadzone::request_cancel() {
  const unsigned int key = irq_lock();
  cancel_req_ = true;
  irq_unlock(key);
}

tune_deadzone::output tune_deadzone::step(const vec3& wheel_rate, bool estop,
                                          uint32_t tick_ms) {
  ARG_UNUSED(tick_ms);
  output out{};

  const unsigned int key = irq_lock();
  if (cancel_req_ || estop) {
    if (running_ || start_req_) {
      // E-Stop/キャンセルによる中断。
      done_ = true;
      done_result_ = 1;
    }
    running_ = false;
    start_req_ = false;
    cancel_req_ = false;
  }
  if (start_req_) {
    start_req_ = false;
    running_ = true;
    done_ = false;
    tick_ = 0;
    meas_ = measure{};
  }
  irq_unlock(key);

  if (!running_) {
    out.active = false;
    return out;
  }
  out.active = true;

  // 0→dir*max（ramp_s）→0（ramp_s）の三角掃引。
  const float t = static_cast<float>(tick_) * 1e-3F;
  const float total = 2.0F * ramp_s_;
  if (t >= total) {
    const unsigned int key2 = irq_lock();
    running_ = false;
    done_ = true;
    done_result_ = 0;
    irq_unlock(key2);
    return out;
  }
  const float phase = (t < ramp_s_) ? (t / ramp_s_) : (2.0F - t / ramp_s_);
  const float duty = dir_ * max_duty_ * phase;
  const float aduty = fabsf(duty);
  const float wake = p_->tune.wake_rate;

  for (int i = 0; i < 3; ++i) {
    out.duty[i] = duty;
    const float arate = fabsf(wheel_rate[i]);
    if (phase > 0.0F && t < ramp_s_) {
      // 立上り: 回転開始時点の |duty|。
      if (meas_.onset[i] == 0.0F && arate >= wake) {
        meas_.onset[i] = aduty;
      }
    } else {
      // 維持: 下降側で回転継続する最小 |duty|。
      if (arate >= wake &&
          (meas_.sustain[i] == 0.0F || aduty < meas_.sustain[i])) {
        meas_.sustain[i] = aduty;
      }
    }
  }
  ++tick_;
  return out;
}

bool tune_deadzone::done() {
  const unsigned int key = irq_lock();
  const bool d = done_;
  irq_unlock(key);
  return d;
}

uint8_t tune_deadzone::done_result() {
  const unsigned int key = irq_lock();
  const uint8_t r = done_result_;
  irq_unlock(key);
  return r;
}

tune_deadzone::measure tune_deadzone::result() {
  const unsigned int key = irq_lock();
  measure m = meas_;
  for (int i = 0; i < 3; ++i) {
    m.suggest[i] = etl::max(m.onset[i], m.sustain[i]);
  }
  irq_unlock(key);
  return m;
}

void tune_deadzone::consume_done() {
  const unsigned int key = irq_lock();
  done_ = false;
  irq_unlock(key);
}

}  // namespace bb
