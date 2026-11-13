#pragma once

#include "peripheral/gpio_pin/gpio_pin.hpp"
#include "peripheral/pwm_output/pwm_output.hpp"

namespace bb {

// Cytron MDD3A モータドライバ（PWM + DIR）。
// 複数インスタンスが必要なためクラスのままとし、init() で設定を受け取る。
class mdd3a_motor {
 public:
  struct config {
    pwm_output* pwm;
    gpio_pin* dir;
  };

  void init(const config& cfg);
  void set_duty(float duty);
  void stop() {
    set_duty(0.0F);
  }

 private:
  config config_{};
};

}  // namespace bb
