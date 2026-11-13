#include "device/wheel_encoder/wheel_encoder.hpp"

#include "common/vector_math/vector_math.hpp"

namespace bb {

void wheel_encoder::init(const config& cfg) {
  config_ = cfg;
}

wheel_encoder::sample wheel_encoder::update(float dt) {
  const int32_t ticks = config_.counter->delta();
  const float model_counts =
      static_cast<float>(ticks) / config_.ticks_per_model_count;
  const float displacement =
      model_counts * (2.0F * PI / config_.counts_per_rev);
  sample s{};
  s.displacement = displacement;
  s.rate = (dt > 0.0F) ? displacement / dt : 0.0F;
  return s;
}

}  // namespace bb
