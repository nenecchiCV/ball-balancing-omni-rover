#include "peripheral/timebase/timebase.hpp"

#include <zephyr/kernel.h>

namespace bb {

uint32_t timebase::micros() {
  return k_cyc_to_us_floor32(k_cycle_get_32());
}

uint32_t timebase::cycles_to_us(uint32_t delta_cycles) {
  return k_cyc_to_us_floor32(delta_cycles);
}

}  // namespace bb
