#pragma once

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// cmd.* パラメータに基づく指令生成（サイクルプロファイル、static class）。
class command_profile {
 public:
  command_profile() = delete;

  struct config {
    const param_set* params;
  };

  struct output {
    vec3 command{};  // [vx_w; vy_w; yaw_rate]
    bool enable = true;
  };

  static void init(const config& cfg);
  static output update(float now_sec);

 private:
  static inline const param_set* p_ = nullptr;
  static inline vec2 prev_velocity_{};
  static inline float prev_time_ = 0.0F;
};

}  // namespace bb
