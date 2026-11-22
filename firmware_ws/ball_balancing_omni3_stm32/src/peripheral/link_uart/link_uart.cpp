#include "peripheral/link_uart/link_uart.hpp"

#include <zephyr/drivers/uart.h>

namespace bb {

bool link_uart::init(const config& cfg) {
  dev_ = cfg.dev;
  if (!device_is_ready(dev_)) {
    return false;
  }
  k_sem_init(&rx_sem_, 0, 32);
  uart_irq_callback_user_data_set(dev_, &uart_isr_cb, nullptr);
  uart_irq_rx_enable(dev_);
  return true;
}

uint32_t link_uart::rx_read(uint8_t* buf, uint32_t max_len) {
  const uint32_t tail = rx_tail_;
  const uint32_t avail = (rx_head_ + RX_SIZE - tail) % RX_SIZE;
  const uint32_t n = (avail < max_len) ? avail : max_len;
  for (uint32_t i = 0; i < n; ++i) {
    buf[i] = rx_buf_[(tail + i) % RX_SIZE];
  }
  rx_tail_ = (tail + n) % RX_SIZE;
  return n;
}

uint32_t link_uart::write(const uint8_t* buf, uint32_t len) {
  uint32_t n = 0;
  const uint32_t head = tx_head_;
  const uint32_t tail = tx_tail_;
  while (n < len && (head + n + 1) % TX_SIZE != tail) {
    tx_buf_[(head + n) % TX_SIZE] = buf[n];
    ++n;
  }
  if (n < len) {
    tx_dropped_ += len - n;
  }
  if (n > 0) {
    tx_head_ = (head + n) % TX_SIZE;
    uart_irq_tx_enable(dev_);
  }
  return n;
}

void link_uart::uart_isr_cb(const struct device* dev, void* user_data) {
  ARG_UNUSED(user_data);
  if (uart_irq_update(dev) == 0) {
    return;
  }
  if (uart_irq_rx_ready(dev) != 0) {
    uint8_t tmp[32];
    int n = 0;
    while ((n = uart_fifo_read(dev, tmp, sizeof(tmp))) > 0) {
      bool gave = false;
      for (int i = 0; i < n; ++i) {
        const uint32_t next = (rx_head_ + 1) % RX_SIZE;
        if (next != rx_tail_) {
          rx_buf_[rx_head_] = tmp[i];
          rx_head_ = next;
          gave = true;
        } else {
          ++rx_dropped_;
        }
      }
      if (gave) {
        k_sem_give(&rx_sem_);
      }
    }
  }
  if (uart_irq_tx_ready(dev) != 0) {
    if (tx_head_ == tx_tail_) {
      uart_irq_tx_disable(dev);
      return;
    }
    // 連続領域分だけまとめて FIFO へ流す。
    const uint32_t n =
        (tx_head_ > tx_tail_) ? (tx_head_ - tx_tail_) : (TX_SIZE - tx_tail_);
    const int sent =
        uart_fifo_fill(dev, &tx_buf_[tx_tail_], static_cast<int>(n));
    if (sent > 0) {
      tx_tail_ = (tx_tail_ + static_cast<uint32_t>(sent)) % TX_SIZE;
    }
    if (tx_head_ == tx_tail_) {
      uart_irq_tx_disable(dev);
    }
  }
}

}  // namespace bb
