#include "device/status_led/status_led.hpp"

namespace bb {

void status_led::init(const config& cfg) {
  led_ = cfg.led;
  pattern_ = pattern::OFF;
  phase_ = false;
  last_toggle_ms_ = 0;
  led_->write(false);
}

void status_led::update(uint32_t now_ms) {
  uint32_t period = 0;
  switch (pattern_) {
    case pattern::SLOW_BLINK:
      period = 500;
      break;
    case pattern::FAST_BLINK:
      period = 125;
      break;
    case pattern::ON:
      led_->write(true);
      return;
    case pattern::OFF:
    default:
      led_->write(false);
      return;
  }
  if (now_ms - last_toggle_ms_ >= period) {
    last_toggle_ms_ = now_ms;
    phase_ = !phase_;
    led_->write(phase_);
  }
}

}  // namespace bb
