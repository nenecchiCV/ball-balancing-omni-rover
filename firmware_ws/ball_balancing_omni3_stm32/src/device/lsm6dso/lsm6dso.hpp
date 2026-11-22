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
    // 実装向きで確定する軸写像（hardware_config.md 参照）。
    // 加速度と角速度で符号規約が異なる場合があるため別マップとする。
    mat33 accel_axis_map = {{{1.0F, 0.0F, 0.0F},
                             {0.0F, 1.0F, 0.0F},
                             {0.0F, 0.0F, 1.0F}}};
    mat33 gyro_axis_map = {{{1.0F, 0.0F, 0.0F},
                            {0.0F, 1.0F, 0.0F},
                            {0.0F, 0.0F, 1.0F}}};
  };

  using sample = imu_sample;

  static bool init(const config& cfg);
  static sample read();

 private:
  static inline const struct device* dev_ = nullptr;
  static inline mat33 accel_axis_map_{};
  static inline mat33 gyro_axis_map_{};
};

}  // namespace bb
