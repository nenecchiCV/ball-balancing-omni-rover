#pragma once

#include "peripheral/encoder_counter/encoder_counter.hpp"

namespace bb {

// ホイール用エンコーダ。encoder_counter の生カウント差分を
// モデルカウント基準の変位・速度へ換算する。
// 複数インスタンスが必要なためクラスのままとし、init() で設定を受け取る。
class wheel_encoder {
 public:
  struct config {
    encoder_counter* counter = nullptr;
    // モデルカウントあたりの生カウント数（x4 デコード等）。
    float ticks_per_model_count = 4.0F;
    float counts_per_rev = 341.2F;  // モデルカウント基準（実機確定）
  };

  struct sample {
    float displacement = 0.0F;  // rad
    float rate = 0.0F;          // rad/s（呼出窓の後退差分）
  };

  void init(const config& cfg);
  sample update(float dt);

 private:
  config config_{};
};

}  // namespace bb
