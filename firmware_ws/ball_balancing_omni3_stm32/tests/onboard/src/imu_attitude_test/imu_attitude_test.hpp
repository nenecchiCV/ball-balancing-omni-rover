#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// IMU+推定器 姿勢検証テスト（static class）。
// 200 Hz で lsm6dso::read() → state_estimator::step() を実行し、
// 10 Hz で生加速度・生ジャイロ・推定オイラー角を表示する。
// 車体を手で傾けて、各軸まわりの角度変化が想定方向と一致するかを
// 人が目視確認する（軸写像行列の妥当性検証用）。
class imu_attitude_test {
 public:
  imu_attitude_test() = delete;

  struct config {
    const param_set* params;
  };

  static void init(const config& cfg);

  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

 private:
  static inline const param_set* p_ = nullptr;
  static inline uint32_t print_phase_ = 0;
};

}  // namespace bb
