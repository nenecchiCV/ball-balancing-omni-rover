#pragma once

#include "common/control_params/control_params.hpp"
#include "device/mdd3a_motor/mdd3a_motor.hpp"
#include "device/wheel_encoder/wheel_encoder.hpp"
#include "peripheral/encoder_counter/encoder_counter.hpp"
#include "peripheral/gpio_pin/gpio_pin.hpp"
#include "peripheral/pwm_output/pwm_output.hpp"

namespace bb {

// オンボードテストアプリのコンポジションルート（static class）。
// robot_application と同じハードウェア配線を行い、テストコンポーネントへ
// 必要なハンドルを注入する。倒立制御ループは持たず、実行ループは
// test_runner に委譲する。
class test_application {
 public:
  test_application() = delete;

  // 全ペリフェラル・デバイス・フィーチャ・テストの初期化とティック開始。
  // 戻り値は IMU 検出の成否。
  static bool init();
  [[noreturn]] static void run();

 private:
  static inline const param_set* p_ = nullptr;
  static inline gpio_pin dir_pins_[3];
  static inline gpio_pin led_pin_;
  static inline gpio_pin estop_ext_pin_;
  static inline gpio_pin estop_btn_pin_;
  static inline pwm_output pwm_ch_[3];
  static inline encoder_counter enc_count_[3];
  static inline wheel_encoder encoders_[3];
  static inline mdd3a_motor motors_[3];
};

}  // namespace bb
