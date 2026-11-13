#pragma once

#include "common/vector_math/vector_math.hpp"
#include "device/wheel_encoder/wheel_encoder.hpp"

namespace bb {

// 輪速計（static class）。外側タスク周期で 3 輪のエンコーダ変位を
// 速度へ換算し、内側タスク用に保持する。
class wheel_speed_meter {
 public:
  wheel_speed_meter() = delete;

  struct config {
    wheel_encoder* enc[3] = {nullptr, nullptr, nullptr};
    float window = 0.005F;
  };

  static void init(const config& cfg);
  // 外側タスク周期（5 ms）で呼ばれ、保持輪速を更新する。
  static vec3 update();
  // 1 kHz タスク用の保持値。
  static vec3 rate() {
    return rate_;
  }

 private:
  static inline wheel_encoder* enc_[3] = {nullptr, nullptr, nullptr};
  static inline float window_ = 0.005F;
  static inline vec3 rate_{};
};

}  // namespace bb
