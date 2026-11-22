#pragma once

// ホストテスト用 irq_lock/irq_unlock スタブ（シングルスレッド前提で無操作）。

static inline unsigned int irq_lock(void) {
  return 0;
}

static inline void irq_unlock(unsigned int key) {
  (void)key;
}
