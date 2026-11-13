#pragma once

#include <zephyr/drivers/pwm.h>

#include <cstdint>

namespace bb {

// PWM 出力（devicetree の pwm_dt_spec を保持する薄いアダプタ）。
// キャリア周期・チャネル・極性は devicetree で確定する。
// 複数インスタンスが必要なためクラスのままとし、init() で設定を受け取る。
class pwm_output {
 public:
  struct config {
    pwm_dt_spec spec;
  };

  bool init(const config& cfg);
  // duty [-1.0, +1.0]。負値の場合は DIR
  // ピン側が極性を担うため絶対値を出力する。
  void set_duty(float duty) const;

 private:
  config config_{};
};

}  // namespace bb
