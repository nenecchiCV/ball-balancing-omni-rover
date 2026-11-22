#include "device/mdd3a_motor/mdd3a_motor.hpp"

#include <etl/algorithm.h>

namespace bb {

void mdd3a_motor::init(const config& cfg) {
  config_ = cfg;
  stop();
}

void mdd3a_motor::set_duty(float duty) {
  const float d = etl::clamp(duty, -1.0F, 1.0F);
  // MDD3A: MxA/MxB は CW/CCW 2入力（片方 PWM・他方 L で駆動、
  // 同値は停止）。正転は MxA、逆転は MxB を PWM 駆動する。
  config_.mxa->set_duty(d >= 0.0F ? d : 0.0F);
  config_.mxb->set_duty(d < 0.0F ? -d : 0.0F);
}

}  // namespace bb
