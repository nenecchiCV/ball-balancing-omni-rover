#pragma once

#include "peripheral/gpio_pin/gpio_pin.hpp"

namespace bb {

// 非常停止入力（外部 E-stop + 基板ユーザボタン、static class）。
// init() で各入力ピンの EXTI 割込を有効化する。
class estop_input {
 public:
  estop_input() = delete;

  struct config {
    gpio_pin* external;  // 外部 E-stop（アクティブロー）。nullptr 可。
    gpio_pin* button;    // 基板ユーザボタン。nullptr 可。
  };

  static bool init(const config& cfg);
  static bool requested();

 private:
  static inline gpio_pin* external_ = nullptr;
  static inline gpio_pin* button_ = nullptr;
};

}  // namespace bb
