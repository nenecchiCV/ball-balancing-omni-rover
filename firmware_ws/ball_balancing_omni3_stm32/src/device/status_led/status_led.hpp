#pragma once

#include <cstdint>

#include "peripheral/gpio_pin/gpio_pin.hpp"

namespace bb {

// 状態表示 LED（static class）。update() は診断タスクから定期的に呼ぶ。
class status_led {
 public:
  status_led() = delete;

  enum class pattern : uint8_t { OFF, ON, SLOW_BLINK, FAST_BLINK };

  struct config {
    gpio_pin* led;
  };

  static void init(const config& cfg);
  static void set(pattern pattern) {
    pattern_ = pattern;
  }
  static void update(uint32_t now_ms);

 private:
  static inline gpio_pin* led_ = nullptr;
  static inline pattern pattern_ = pattern::OFF;
  static inline bool phase_ = false;
  static inline uint32_t last_toggle_ms_ = 0;
};

}  // namespace bb
