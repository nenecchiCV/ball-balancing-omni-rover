#include "peripheral/host_uart/host_uart.hpp"

#include <zephyr/drivers/uart.h>

namespace uwc {

bool host_uart::init(const config& cfg) {
  uart_dev_ = cfg.uart_dev;
  rx_ring_ = cfg.rx_ring;
  rx_sem_ = cfg.rx_sem;
  if (!device_is_ready(uart_dev_)) {
    return false;
  }
  ring_buf_init(&tx_ring_, sizeof(tx_mem_), tx_mem_);
  uart_irq_callback_user_data_set(uart_dev_, &uart_isr_cb, nullptr);
  uart_irq_rx_enable(uart_dev_);
  return true;
}

int host_uart::write(const uint8_t* data, uint32_t len) {
  const uint32_t n = ring_buf_put(&tx_ring_, data, len);
  if (n > 0) {
    uart_irq_tx_enable(uart_dev_);
  }
  return static_cast<int>(n);
}

void host_uart::uart_isr_cb(const struct device* dev, void* user_data) {
  ARG_UNUSED(user_data);
  while (uart_irq_update(dev) != 0 && uart_irq_is_pending(dev) != 0) {
    if (uart_irq_rx_ready(dev) != 0) {
      uint8_t buf[64];
      const int n = uart_fifo_read(dev, buf, sizeof(buf));
      if (n > 0) {
        const uint32_t put =
            ring_buf_put(rx_ring_, buf, static_cast<uint32_t>(n));
        rx_drop_count_ += static_cast<uint32_t>(n) - put;
        k_sem_give(rx_sem_);
      }
    }
    if (uart_irq_tx_ready(dev) != 0) {
      uint8_t buf[64];
      const uint32_t avail = ring_buf_peek(&tx_ring_, buf, sizeof(buf));
      if (avail == 0) {
        uart_irq_tx_disable(dev);
        continue;
      }
      const int sent = uart_fifo_fill(dev, buf, static_cast<int>(avail));
      if (sent > 0) {
        (void)ring_buf_get(&tx_ring_, nullptr, static_cast<uint32_t>(sent));
      }
    }
  }
}

}  // namespace uwc
