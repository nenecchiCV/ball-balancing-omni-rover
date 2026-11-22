#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <cstdint>

namespace bb {

// USART1 リンク UART（115200 8N1、static class）。
// RX は割込 → リング → k_sem 通知、TX はリング → 割込駆動送出で、
// 呼出し側（リンクスレッド）をブロックしない。
class link_uart {
 public:
  link_uart() = delete;

  struct config {
    const struct device* dev;
  };

  static bool init(const config& cfg);

  // RX リングへの投入ごとに give されるセマフォ（k_poll 用にポインタで返す）。
  static struct k_sem* rx_sem() {
    return &rx_sem_;
  }

  // RX リングから取り出す（呼出しはリンクスレッドに限定）。
  static uint32_t rx_read(uint8_t* buf, uint32_t max_len);
  // TX リングへ投入して割込送出を開始。投入できたバイト数を返す。
  static uint32_t write(const uint8_t* buf, uint32_t len);

  static uint32_t rx_dropped() {
    return rx_dropped_;
  }
  static uint32_t tx_dropped() {
    return tx_dropped_;
  }

 private:
  static void uart_isr_cb(const struct device* dev, void* user_data);

  static constexpr uint32_t RX_SIZE = 256;
  static constexpr uint32_t TX_SIZE = 512;

  static inline const struct device* dev_ = nullptr;
  static inline struct k_sem rx_sem_ = {};

  static inline uint8_t rx_buf_[RX_SIZE] = {};
  static inline volatile uint32_t rx_head_ = 0;  // ISR 書・スレッド読
  static inline volatile uint32_t rx_tail_ = 0;  // スレッド書・ISR 読
  static inline volatile uint32_t rx_dropped_ = 0;

  static inline uint8_t tx_buf_[TX_SIZE] = {};
  static inline volatile uint32_t tx_head_ = 0;  // スレッド書・ISR 読
  static inline volatile uint32_t tx_tail_ = 0;  // ISR 書・スレッド読
  static inline volatile uint32_t tx_dropped_ = 0;
};

}  // namespace bb
