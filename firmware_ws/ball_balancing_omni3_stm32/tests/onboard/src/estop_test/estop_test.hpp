#pragma once

#include <cstdint>

#include "peripheral/gpio_pin/gpio_pin.hpp"

namespace bb {

// E-stop 入力検証テスト（static class）。
// 外部 E-stop・基板ボタンの生レベルと estop_input::requested() を
// 変化時 + 1 秒ハートビートで表示する。
class estop_test {
 public:
  estop_test() = delete;

  struct config {
    gpio_pin* external;
    gpio_pin* button;
  };

  static void init(const config& cfg);

  // test_runner から呼ばれるコールバック。
  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

 private:
  static inline gpio_pin* external_ = nullptr;
  static inline gpio_pin* button_ = nullptr;
  static inline bool prev_req_ = false;
  static inline bool prev_ext_ = false;
  static inline bool prev_btn_ = false;
  static inline uint32_t tick_ = 0;
};

}  // namespace bb
