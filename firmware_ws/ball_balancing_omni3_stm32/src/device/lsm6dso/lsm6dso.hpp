#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "common/imu_sample/imu_sample.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// LSM6DSO IMU（Zephyr センサドライバ経由、static class）。
// デバイス設定（ODR 833Hz, ±4g, ±2000dps）は devicetree で確定済み。
// 本クラスは軸写像のみを担当する。
class lsm6dso {
 public:
  lsm6dso() = delete;

  struct config {
    const struct device* dev = nullptr;
    mat33 axis_map = {{{1.0F, 0.0F, 0.0F},
                       {0.0F, 1.0F, 0.0F},
                       {0.0F, 0.0F, 1.0F}}};  // 実装向きで確定
  };

  using sample = imu_sample;

  static bool init(const config& cfg);
  static sample read();

 private:
  static inline const struct device* dev_ = nullptr;
  static inline mat33 axis_map_{};
};

}  // namespace bb
