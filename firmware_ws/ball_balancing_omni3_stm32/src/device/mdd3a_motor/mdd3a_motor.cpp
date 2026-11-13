#include "device/mdd3a_motor/mdd3a_motor.hpp"

#include <etl/algorithm.h>

namespace bb {

void mdd3a_motor::init(const config& cfg) {
  config_ = cfg;
  stop();
}

void mdd3a_motor::set_duty(float duty) {
  const float clamped = etl::clamp(duty, -1.0F, 1.0F);
  // MDD3A: mx_a=PWM, mx_b=DIR。DIR=H で正転。
  config_.dir->write(clamped >= 0.0F);
  config_.pwm->set_duty(clamped);
}

}  // namespace bb
