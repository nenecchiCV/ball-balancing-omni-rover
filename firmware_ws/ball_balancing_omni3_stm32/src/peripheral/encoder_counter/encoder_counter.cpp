#include "peripheral/encoder_counter/encoder_counter.hpp"

#include <stm32_ll_tim.h>

namespace bb {

void encoder_counter::init(const config& cfg) {
  config_ = cfg;
  prev_raw_ = LL_TIM_GetCounter(config_.timer);
  count_ = 0;
}

int32_t encoder_counter::delta() {
  const uint32_t raw = LL_TIM_GetCounter(config_.timer);
  // 16 bit カウンタの差分を符号付きで取得する。
  const auto d = static_cast<int32_t>(static_cast<int16_t>(raw - prev_raw_));
  prev_raw_ = raw;
  count_ += d;
  return d;
}

}  // namespace bb
