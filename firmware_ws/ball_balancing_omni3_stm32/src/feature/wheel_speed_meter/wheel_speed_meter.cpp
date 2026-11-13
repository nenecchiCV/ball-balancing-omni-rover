#include "feature/wheel_speed_meter/wheel_speed_meter.hpp"

namespace bb {

void wheel_speed_meter::init(const config& cfg) {
  enc_[0] = cfg.enc[0];
  enc_[1] = cfg.enc[1];
  enc_[2] = cfg.enc[2];
  window_ = cfg.window;
  rate_ = vec3{};
}

vec3 wheel_speed_meter::update() {
  for (int i = 0; i < 3; ++i) {
    rate_[i] = enc_[i]->update(window_).rate;
  }
  return rate_;
}

}  // namespace bb
