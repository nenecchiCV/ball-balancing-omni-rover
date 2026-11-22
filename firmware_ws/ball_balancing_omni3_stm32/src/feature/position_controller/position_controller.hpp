#pragma once

#include "common/control_params/control_params.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// ワールド座標の目標位置への位置外ループ（FR-10、static class）。
// 位置誤差 → ワールド速度指令を生成し balance_controller へ渡す。
// 目標スロットはリンクスレッドから書・制御スレッドから読されるため、
// 読み書きは irq_lock で短く保護する。
class position_controller {
 public:
  position_controller() = delete;

  struct config {
    const param_set* params;
  };

  struct output {
    vec3 command{};  // [vx_w; vy_w; yaw_rate]
    bool active = false;
    bool arrived = false;
  };

  static void init(const config& cfg);

  // 目標設定。speed_limit は min(speed_limit, controller.max_speed)。
  static void set_target(float x, float y, float speed_limit);
  static void clear_target();
  // STOP 対応: 目標を指定位置へ置き換える（active は維持）。
  static void hold_at(float x, float y);

  // 現在位置に対する指令（外側タスク呼出し）。未設定・到達時は零指令。
  static output update(float x, float y, float dt);

 private:
  // enclosing class の complete-class 規則を避けるため
  // メンバ初期化子を持たない POD にする。
  struct target {
    float x;
    float y;
    float speed_limit;
    bool active;
  };

  static inline const param_set* p_ = nullptr;
  static inline target target_ = {0.0F, 0.0F, 0.0F, false};
  static inline vec2 prev_command_{};  // slew 用の前回指令
};

}  // namespace bb
