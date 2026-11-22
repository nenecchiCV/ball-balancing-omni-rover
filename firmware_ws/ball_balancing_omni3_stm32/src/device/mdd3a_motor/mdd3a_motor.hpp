#pragma once

#include "peripheral/pwm_output/pwm_output.hpp"

namespace bb {

// Cytron MDD3A モータドライバ（MxA/MxB = CW/CCW 2入力、両方とも PWM）。
// 複数インスタンスが必要なためクラスのままとし、init() で設定を受け取る。
class mdd3a_motor {
 public:
  struct config {
    pwm_output* mxa;  // 正転脚（duty > 0 で駆動）
    pwm_output* mxb;  // 逆転脚（duty < 0 で駆動）
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
