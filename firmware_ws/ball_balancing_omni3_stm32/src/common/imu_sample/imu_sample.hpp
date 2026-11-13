#pragma once

#include "common/vector_math/vector_math.hpp"

namespace bb {

// IMU 1 サンプル分のデータ構造。
// device 層の IMU ドライバが生成し、feature 層が消費する。
// Zephyr に依存しないため、feature 層はホストテスト可能。
struct imu_sample {
  vec3 specific_force{};
  vec3 gyro{};
  bool valid = false;
};

}  // namespace bb
