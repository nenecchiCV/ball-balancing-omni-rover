#pragma once

#include <zephyr/kernel.h>

#include <cstdint>

namespace bb {

// 時刻・サイクル計測（Zephyr kernel API の薄いラッパ、static class）。
class timebase {
 public:
  timebase() = delete;

  static uint32_t millis() {
    return k_uptime_get_32();
  }
  static uint32_t micros();
  static uint32_t cycles() {
    return k_cycle_get_32();
  }
  static uint32_t cycles_to_us(uint32_t delta_cycles);
};

}  // namespace bb
