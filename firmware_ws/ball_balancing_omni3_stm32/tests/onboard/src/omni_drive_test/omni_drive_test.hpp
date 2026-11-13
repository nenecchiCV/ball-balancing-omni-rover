#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"
#include "device/mdd3a_motor/mdd3a_motor.hpp"

namespace bb {

// 3 輪オムニ速度・回転指令テスト（static class）。
//   `test omni <vx> <vy> <yaw>`
// vx, vy: 車体座標並進速度 [m/s]（±max_speed にクランプ）
// yaw: ヨー角速度 [rad/s]（±max_yaw_rate にクランプ）
//
// lqi.wheel_speed_from_planar_velocity で [vx,vy,yaw]→輪速指令へ写像し、
// テスト内蔵の FF+比例輪速サーボで追従させる。倒立制御は使わないため、
// 人が車体を手で支えて実施する。60 秒で自動停止する。
//
// 注: 製品の wheel_servo の推進トルク上限（0.045 Nm）は無負荷駆動に
// 不足するため、本テストはテスト固有のデューティ生成を使う。
class omni_drive_test {
 public:
  omni_drive_test() = delete;

  struct config {
    const param_set* params;
    mdd3a_motor* motors[3];
  };

  static void init(const config& cfg);

  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

 private:
  static inline const param_set* p_ = nullptr;
  static inline mdd3a_motor* motors_[3] = {nullptr, nullptr, nullptr};
  static inline vec3 cmd_{};            // {vx, vy, yaw}
  static inline vec3 wheel_cmd_{};      // 輪速指令 [rad/s]
  static inline float duty_ff_ = 0.0F;  // 逆起電力フィードフォワード係数
  static inline uint32_t tick_ = 0;
  static inline uint32_t meter_phase_ = 0;
};

}  // namespace bb
