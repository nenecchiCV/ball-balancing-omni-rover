#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <cstdint>

namespace uwc {

// UART0 ↔ 無線リンク の双方向バイト中継。static class。
// ブリッジスレッドで稼働し、両方向のリングを排出する。
class uart_bridge {
 public:
  uart_bridge() = delete;

  struct config {
    struct ring_buf* host_to_link;
    struct ring_buf* link_to_host;
    uint32_t poll_ms;  // イベント待ちの最大間隔
  };

  static bool init(const config& cfg);
  [[noreturn]] static void run();

  // host_uart / link が rx_sem として共有するセマフォ。
  // init() 後に呼ぶこと。
  static struct k_sem* rx_sem() {
    return &rx_sem_;
  }

 private:
  static void pump_host_to_link();
  static void pump_link_to_host();

  static inline struct ring_buf* host_to_link_ = nullptr;
  static inline struct ring_buf* link_to_host_ = nullptr;
  static inline uint32_t poll_ms_ = 1;
  static inline struct k_sem rx_sem_ = {};
};

}  // namespace uwc
