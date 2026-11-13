#pragma once

#include <soc.h>

#include <cstdint>

namespace bb {

// TIM エンコーダモードの生カウント差分を返す。
// devicetree の qdec ノードがタイマをエンコーダモードで初期化するため、
// 本クラスはカウンタ値の読取りと差分計算のみを行う。
// counts_per_revolution=2^n により ARR=65535 の自由周回となり、
// int16_t 差分でラップアラウンドを処理する。
// 複数インスタンスが必要なためクラスのままとし、init() で設定を受け取る。
class encoder_counter {
 public:
  struct config {
    TIM_TypeDef* timer;
  };

  // 現在の CNT を基準値として取り込む。
  void init(const config& cfg);
  // 前回呼び出しからのカウント差分（符号付き、16bitラップ吸収）。
  int32_t delta();
  [[nodiscard]] int64_t count() const {
    return count_;
  }

 private:
  config config_{};
  uint32_t prev_raw_ = 0;
  int64_t count_ = 0;
};

}  // namespace bb
