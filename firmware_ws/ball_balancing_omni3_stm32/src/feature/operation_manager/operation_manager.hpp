#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"

namespace bb {

// 動作状態（待機 ⇔ 動作中 ⇔ 終端）と IMU キャリブレーション状態を
// 管理する（FR-12, FR-13、static class）。状態遷移・受理判定は Zephyr
// 非依存（irq_lock のみ使用）の純粋ロジックで、ホストテスト対象。
// モータ駆動の実体は robot_application が drives_motors() を参照して
// ゲートする。
//
// スレッドモデル:
//   リンクスレッド: request_start / request_stop / request_calib を
//                   link_session::dispatch 経由で呼ぶ。
//   制御スレッド:   drives_motors / calib_update / take_bias_reset_req /
//                   notify_controller_terminal を外側タスクから呼ぶ。
class operation_manager {
 public:
  operation_manager() = delete;

  struct config {
    const param_set* params;
  };

  // STATUS / EVT_STATUS の op_state / calib_state と同じ値。
  struct op_state {
    static constexpr uint8_t STANDBY = 0;
    static constexpr uint8_t OPERATING = 1;
    static constexpr uint8_t FAULT = 2;
  };
  struct calib_state {
    static constexpr uint8_t NONE = 0;
    static constexpr uint8_t RUNNING = 1;
    static constexpr uint8_t DONE = 2;
    static constexpr uint8_t FAILED = 3;
  };

  // START_OPERATION 受理判定の入力（リンクスレッド側で組立てる）。
  struct start_cond {
    bool imu_ok;
    bool estop;
    bool qualified;
    float tilt;  // 推定姿勢の傾斜角 [rad]
  };

  static void init(const config& cfg);

  static uint8_t state();  // op_state
  static uint8_t calib();  // calib_state
  static bool drives_motors();

  // リンクスレッド（dispatch）・シェルから呼ぶ。
  static uint8_t request_start(const start_cond& c);  // ack_result を返す
  static void request_stop();
  static uint8_t request_calib(uint8_t seq);

  // 外側タスクから呼ぶ。
  // -1=実施中継続、0..2=CALIB_RESULT result（状態更新済み）。
  static int8_t calib_update(uint32_t now_ms, bool qualified, bool imu_ok);
  static uint8_t calib_seq();  // CALIB_RESULT エコー用の受理 seq
  static bool take_bias_reset_req();
  static void notify_controller_terminal();

 private:
  static inline const param_set* p_ = nullptr;

  // 全て irq_lock 保護。
  static inline uint8_t state_ = op_state::STANDBY;
  static inline uint8_t calib_ = calib_state::NONE;
  static inline uint8_t calib_seq_ = 0;
  static inline uint32_t calib_deadline_ms_ = 0;
  static inline bool bias_reset_req_ = false;
};

}  // namespace bb
