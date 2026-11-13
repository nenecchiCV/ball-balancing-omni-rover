#include "peripheral/pwm_output/pwm_output.hpp"

#include <etl/algorithm.h>

namespace bb {

bool pwm_output::init(const config& cfg) {
  config_ = cfg;
  return pwm_is_ready_dt(&config_.spec);
}

void pwm_output::set_duty(float duty) const {
  const float d = etl::clamp(duty, -1.0F, 1.0F);
  const uint32_t pulse = static_cast<uint32_t>(
      static_cast<float>(config_.spec.period) * (d < 0.0F ? -d : d));
  (void)pwm_set_pulse_dt(&config_.spec, pulse);
}

}  // namespace bb
