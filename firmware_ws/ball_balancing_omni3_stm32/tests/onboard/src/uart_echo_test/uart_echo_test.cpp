#include "uart_echo_test/uart_echo_test.hpp"

#include <stdio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"

namespace bb {

namespace {

constexpr uint32_t RING_SIZE = 256U;

const struct device* uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));

// USART1 RX -> 排出スレッド への受信リングと転送統計。
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
uint8_t ring_storage_[RING_SIZE];
struct ring_buf rx_ring_;
K_SEM_DEFINE(rx_sem_, 0, 1);
uint32_t rx_count_ = 0;
uint32_t tx_count_ = 0;
uint32_t drop_count_ = 0;
uint32_t quiet_wakes_ = 0;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// USART1 受信割込: RX FIFO をリングへ移し排出スレッドを起こす。
// リング溢れ分は drop として計数する。
void uart_rx_cb(const struct device* dev, void* user_data) {
  ARG_UNUSED(user_data);
  while (uart_irq_update(dev) != 0 && uart_irq_is_pending(dev) != 0) {
    if (uart_irq_rx_ready(dev) == 0) {
      break;
    }
    uint8_t buf[64];
    const int n = uart_fifo_read(dev, buf, sizeof(buf));
    if (n <= 0) {
      break;
    }
    rx_count_ += static_cast<uint32_t>(n);
    const uint32_t put =
        ring_buf_put(&rx_ring_, buf, static_cast<uint32_t>(n));
    drop_count_ += static_cast<uint32_t>(n) - put;
    if (put > 0U) {
      k_sem_give(&rx_sem_);
    }
  }
}

// 排出スレッド: リングの内容を USART1 TX へ poll_out で折り返す。
// 2 秒以上エコーが無いアイドル時のみ "hb" マーカーを打ち、
// STM32 TX→ESP32 RX→BLE Notify の折返し経路を受信側だけでも
// 確認できるようにする（転送中のストリームは汚さない）。
void echo_entry(void*, void*, void*) {
  uint8_t buf[64];
  for (;;) {
    k_sem_take(&rx_sem_, K_MSEC(500));
    uint32_t drained = 0;
    for (;;) {
      const uint32_t n = ring_buf_get(&rx_ring_, buf, sizeof(buf));
      if (n == 0U) {
        break;
      }
      for (uint32_t i = 0; i < n; ++i) {
        uart_poll_out(uart_dev, buf[i]);
      }
      tx_count_ += n;
      drained += n;
    }
    if (drained == 0U) {
      ++quiet_wakes_;
      if ((quiet_wakes_ % 4U) == 0U) {
        uart_poll_out(uart_dev, 'h');
        uart_poll_out(uart_dev, 'b');
      }
    } else {
      quiet_wakes_ = 0;
    }
  }
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
K_THREAD_DEFINE(echo_thread, 1024, echo_entry, nullptr, nullptr, nullptr, 10, 0,
                0);

}  // namespace

bool uart_echo_test::init() {
  if (!device_is_ready(uart_dev)) {
    debug_uart::write_line("echo test: usart1 not ready");
    return false;
  }
  ring_buf_init(&rx_ring_, RING_SIZE, ring_storage_);
  uart_irq_callback_set(uart_dev, uart_rx_cb);
  // 無線変換疎通は shell 操作なしで成立するよう、ブート時から
  // エコーを有効にする。`test echo` は統計報告の開始として使う。
  uart_irq_rx_enable(uart_dev);
  return true;
}

bool uart_echo_test::start_cb(int argc, char** argv) {
  (void)argc;
  (void)argv;
  rx_count_ = 0;
  tx_count_ = 0;
  drop_count_ = 0;
  ring_buf_reset(&rx_ring_);
  uart_irq_rx_enable(uart_dev);
  debug_uart::write_line("echo test: USART1 byte echo stats started");
  return true;
}

void uart_echo_test::update_cb() {
  // 1 Hz（period=1000 tick）呼出。転送統計を報告する。
  char line[96];
  const int n = snprintf(
      line, sizeof(line), "echo t=%lu rx=%lu tx=%lu drop=%lu\r\n",
      static_cast<unsigned long>(timebase::millis()),
      static_cast<unsigned long>(rx_count_),
      static_cast<unsigned long>(tx_count_),
      static_cast<unsigned long>(drop_count_));
  debug_uart::write(line, static_cast<uint32_t>(n));
}

void uart_echo_test::stop_cb() {
  uart_irq_rx_disable(uart_dev);
  uart_irq_tx_disable(uart_dev);
  debug_uart::write_line("echo test: stopped");
}

}  // namespace bb
