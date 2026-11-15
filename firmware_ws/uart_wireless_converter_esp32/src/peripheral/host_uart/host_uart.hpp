#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <cstdint>

namespace uwc {

// ホスト UART (uart0) の割込駆動ラッパ。static class。
// RX は割込で rx_ring へ積み、TX は内部リング経由で割込駆動する。
class host_uart {
 public:
  host_uart() = delete;

  struct config {
    const struct device* uart_dev;
    struct ring_buf* rx_ring;
    struct k_sem* rx_sem;
  };

  static bool init(const config& cfg);

  // TX リングへ積んで送出を開始する。積めたバイト数を返す。
  static int write(const uint8_t* data, uint32_t len);

  // RX リング溢れで破棄したバイト数（統計参照用）
  static uint32_t rx_dropped() {
    return rx_drop_count_;
  }

 private:
  static void uart_isr_cb(const struct device* dev, void* user_data);

  static inline const struct device* uart_dev_ = nullptr;
  static inline struct ring_buf* rx_ring_ = nullptr;
  static inline struct k_sem* rx_sem_ = nullptr;
  static inline uint32_t rx_drop_count_ = 0;

  static inline struct ring_buf tx_ring_ = {};
  static inline uint8_t tx_mem_[512] = {};
};

}  // namespace uwc
