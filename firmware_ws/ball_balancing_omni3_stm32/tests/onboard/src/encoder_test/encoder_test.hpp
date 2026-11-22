#pragma once

#include <cstdint>

#include "device/mdd3a_motor/mdd3a_motor.hpp"
#include "device/wheel_encoder/wheel_encoder.hpp"
#include "peripheral/encoder_counter/encoder_counter.hpp"

namespace bb {

// エンコーダ検証テスト（static class）。
//   `test enc`      : 手で車輪を回し、変位・輪速・累積カウントを表示。
//                     想定正方向への回転で正の値になるか目視確認する。
//   `test enc spin [duty]` : 全モータを ±duty（既定 0.40）で 3 秒ずつ駆動し、
//                     duty 符号とエンコーダ輪速符号の一致を確認する。
class encoder_test {
 public:
  encoder_test() = delete;

  struct config {
    encoder_counter* counters[3];
    wheel_encoder* encoders[3];
    mdd3a_motor* motors[3];
  };

  static void init(const config& cfg);

  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

 private:
  static inline encoder_counter* counters_[3] = {nullptr, nullptr, nullptr};
  static inline wheel_encoder* encoders_[3] = {nullptr, nullptr, nullptr};
  static inline mdd3a_motor* motors_[3] = {nullptr, nullptr, nullptr};
  static inline bool spin_ = false;
  static inline uint32_t spin_phase_ = 0;      // 更新回数（1 更新 = 100 ms）
  static inline float spin_duty_ = 0.40F;      // spin モードの駆動デューティ
};

}  // namespace bb
