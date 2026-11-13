#pragma once

#include <cstdint>

#include "device/mdd3a_motor/mdd3a_motor.hpp"

namespace bb {

// モータ不感帯システム同定テスト（static class）。
//   `test deadzone [dir] [max_duty] [ramp_s]`
// 全車輪を無負荷（車輪のみ・接地なし）で同一デューティに掃引する。
// dir（±1、既定 +1）、max_duty（既定 0.25）、ramp_s（片道時間、既定 15 s）。
// 0→±max→0 の三角掃引を行い、20 Hz で CSV 行を出力する。
// 人がログを回収し、車輪が回り始めるデューティを不感帯として同定する
// （上り/下りで履歴差も評価できる）。
class motor_deadzone_test {
 public:
  motor_deadzone_test() = delete;

  struct config {
    mdd3a_motor* motors[3];
  };

  static void init(const config& cfg);

  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

 private:
  static inline mdd3a_motor* motors_[3] = {nullptr, nullptr, nullptr};
  static inline float dir_ = 1.0F;
  static inline float max_duty_ = 0.25F;
  static inline float ramp_s_ = 15.0F;
  static inline uint32_t tick_ = 0;
  static inline uint32_t meter_phase_ = 0;
};

}  // namespace bb
