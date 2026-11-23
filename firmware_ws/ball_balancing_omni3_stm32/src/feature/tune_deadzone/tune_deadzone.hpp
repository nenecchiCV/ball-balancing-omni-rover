#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// 不感帯同定セッション（FR-17、static class）。
// TUNING 中のみ robot_application が 1 kHz 内側タスクから step() を呼び、
// 戻り値の duty をモータへ直接適用する（wheel_servo は回さない）。
// 掃引は motor_deadzone_test と同じ三角波 0→dir*max→0（2*ramp_s）。
//
// スレッドモデル:
//   リンクスレッド: start/cancel がセッション要求を積む。
//   制御スレッド:   step() が掃引を進め、結果スロットを積む。
//   robot_application が完了を検出して link_session へ TUNE_RESULT を渡す。
class tune_deadzone {
 public:
  tune_deadzone() = delete;

  struct config {
    const param_set* params;
  };

  // step() の戻り値。active 中は duty をモータへ適用し、
  // done()==true のとき result() が確定している（duty は 0）。
  struct output {
    vec3 duty{};
    bool active = false;
  };

  static void init(const config& cfg);

  // リンクスレッドから呼ぶ。掃引パラメータをセットして開始要求を積む。
  // 実行中なら false。dir は ±1.0 のみ使用（符号）。
  static bool request_start(float dir, float max_duty, float ramp_s);
  static void request_cancel();

  // 1 kHz 内側タスク（TUNING 中のみ呼ぶ）。E-Stop で即時中断する。
  static output step(const vec3& wheel_rate, bool estop, uint32_t tick_ms);

  // TUNE_RESULT 組立て用の計測結果（確定後のみ有効）。
  struct measure {
    float onset[3];    // 立上り duty 絶対値（回転開始時、0=未検出）
    float sustain[3];  // 維持 duty 絶対値（下降側で回転継続の最小）
    float suggest[3];  // 推奨 deadzone_comp = max(onset, sustain)
  };
  static measure result();
  static bool done();            // 掃引完了/中断で確定（result=0/1）
  static uint8_t done_result();
  static void consume_done();  // robot_application が通知後にクリア

 private:
  static constexpr float CLAMP_DUTY = 0.8F;
  static constexpr float CLAMP_RAMP_S = 120.0F;

  static inline const param_set* p_ = nullptr;
  static inline float dir_ = 1.0F;
  static inline float max_duty_ = 0.25F;
  static inline float ramp_s_ = 15.0F;
  static inline uint32_t tick_ = 0;
  static inline bool start_req_ = false;
  static inline bool cancel_req_ = false;
  static inline bool running_ = false;
  static inline bool done_ = false;
  static inline uint8_t done_result_ = 0;
  static inline measure meas_{};
};

}  // namespace bb
