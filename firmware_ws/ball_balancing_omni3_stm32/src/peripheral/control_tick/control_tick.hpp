#pragma once

#include <zephyr/kernel.h>

namespace bb {

// TIM6 による 1 kHz 制御ティック（システムに 1 つのみのため static class）。
// devicetree で有効化した timers6 のプリスケーラ (1 MHz) に ARR=999 を設定し、
// 更新割込でセマフォを返して制御スレッドを起床させる。
// init() はタイマ設定と割込登録のみ行い、start() で計時を開始する。
class control_tick {
 public:
  control_tick() = delete;

  static bool init();
  // 更新割込とカウンタを有効化してティック供給を開始する。
  static void start();
  static void stop();
  // 次ティックまでブロック。ISR からセマフォが供給される。
  static void wait();
  // ISR 直接呼び出し用のセマフォ。
  static struct k_sem* sem();
  // ISR 発火回数（診断用）。
  static uint32_t isr_count();

 private:
  static void tick_isr_cb(const void* arg);
};

}  // namespace bb
