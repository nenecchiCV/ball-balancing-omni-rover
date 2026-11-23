#pragma once

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"
#include "device/estop_input/estop_input.hpp"
#include "device/lsm6dso/lsm6dso.hpp"
#include "device/mdd3a_motor/mdd3a_motor.hpp"
#include "device/status_led/status_led.hpp"
#include "device/wheel_encoder/wheel_encoder.hpp"
#include "feature/balance_controller/balance_controller.hpp"
#include "feature/command_profile/command_profile.hpp"
#include "feature/link_session/link_session.hpp"
#include "feature/odometry/odometry.hpp"
#include "feature/operation_manager/operation_manager.hpp"
#include "feature/position_controller/position_controller.hpp"
#include "feature/state_estimator/state_estimator.hpp"
#include "feature/wheel_servo/wheel_servo.hpp"
#include "feature/wheel_speed_meter/wheel_speed_meter.hpp"
#include "manager/task_scheduler/task_scheduler.hpp"
#include "peripheral/control_tick/control_tick.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/encoder_counter/encoder_counter.hpp"
#include "peripheral/gpio_pin/gpio_pin.hpp"
#include "peripheral/pwm_output/pwm_output.hpp"
#include "peripheral/timebase/timebase.hpp"

namespace bb {

// shell から参照する診断スナップショット。
struct telemetry_snapshot {
  balance_controller::output ctrl{};
  state_estimator::output est{};
  vec3 wheel_rate{};
  bool enable = false;
  bool imu_ok = false;
  uint32_t outer_index = 0;
  uint32_t tick_count = 0;
  uint32_t overrun_count = 0;
  uint32_t max_inner_us = 0;
  uint32_t max_outer_us = 0;
};

// ハードウェア構成はすべて devicetree (bb,pins / qdec / imu ノード)
// から取得する。システムに 1 つのみのため static class とし、
// 生成・破棄ではなく init() / run() でライフサイクルを表す。
class robot_application {
 public:
  robot_application() = delete;

  struct config {
    const param_set* params;
    mat33 imu_accel_axis_map;
    mat33 imu_gyro_axis_map;
  };

  static bool init(const config& cfg);
  [[noreturn]] static void run();

  // shell インターフェース（別スレッドから呼ばれる）
  static telemetry_snapshot snapshot();
  static void set_telemetry(bool on) {
    telem_enabled_ = on;
  }
  static bool telemetry_enabled() {
    return telem_enabled_;
  }

  // `bb link` 診断用カウンタ。
  static link_session::stats link_stats();

 private:
  // task_scheduler の hooks へ登録するタスクコールバック。
  static void outer_task_cb();
  static void inner_task_cb();
  static void diag_task_cb();

  static inline const param_set* p_ = nullptr;
  static inline mat33 imu_accel_axis_map_{};
  static inline mat33 imu_gyro_axis_map_{};

  // peripheral（複数インスタンスを要するクラスのみ保持）
  static inline gpio_pin led_pin_;
  static inline gpio_pin estop_ext_pin_;
  static inline gpio_pin estop_btn_pin_;
  static inline pwm_output pwm_a_[3];
  static inline pwm_output pwm_b_[3];
  static inline encoder_counter enc_count_[3];

  // device（複数インスタンスを要するクラスのみ保持）
  static inline mdd3a_motor motors_[3];
  static inline wheel_encoder encoders_[3];

  // タスク間 ZOH コンテキスト（同一制御スレッド内で更新される）
  static inline balance_controller::output ctrl_out_{};
  static inline state_estimator::output est_out_{};
  static inline wheel_servo::output servo_out_{};
  static inline vec3 wheel_rate_{};
  static inline bool enable_ = false;
  static inline bool imu_ok_ = false;
  static inline bool prev_operating_ = false;
  static inline uint32_t outer_index_ = 0;
  // shell スレッドから書き換えられるため volatile。bool 単一ワードの
  // 読み書きは Cortex-M 上でアトミック。
  static inline volatile bool telem_enabled_ = false;
};

}  // namespace bb
