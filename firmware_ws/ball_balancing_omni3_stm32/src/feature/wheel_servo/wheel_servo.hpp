#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// ballbot_wheel_servo_duty_step の移植（1 kHz 内側タスク、static class）。
class wheel_servo {
 public:
  wheel_servo() = delete;

  struct config {
    const param_set* params;
  };

  struct input {
    vec3 attitude_torque{};
    vec3 wheel_rate_command{};
    bool settle_witness = false;
    bool enable = false;
    vec3 wheel_rate{};
    uint8_t mode = 0;
  };

  struct output {
    vec3 duty{};
    vec3 filtered_speed{};
  };

  static void init(const config& cfg);
  static output step(const input& in);

 private:
  static vec3 servo_startup_ramp(const vec3& value, bool enable);

  static inline const param_set* p_ = nullptr;
  static inline vec3 filtered_speed_{};
  static inline float ramp_clock_ = 0.0F;
  static inline bool ramp_started_ = false;
};

}  // namespace bb
