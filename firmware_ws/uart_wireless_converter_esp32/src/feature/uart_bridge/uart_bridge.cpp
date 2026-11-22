#include "feature/uart_bridge/uart_bridge.hpp"

#include "device/link/link.hpp"
#include "feature/link_monitor/link_monitor.hpp"
#include "peripheral/host_uart/host_uart.hpp"

namespace uwc {

namespace {
constexpr uint32_t TX_CHUNK_MAX = 256;
}

bool uart_bridge::init(const config& cfg) {
  host_to_link_ = cfg.host_to_link;
  link_to_host_ = cfg.link_to_host;
  poll_ms_ = (cfg.poll_ms == 0) ? 1 : cfg.poll_ms;
  k_sem_init(&rx_sem_, 0, 64);
  return host_to_link_ != nullptr && link_to_host_ != nullptr;
}

void uart_bridge::run() {
  for (;;) {
    (void)k_sem_take(&rx_sem_, K_MSEC(poll_ms_));
    pump_host_to_link();
    pump_link_to_host();
  }
}

void uart_bridge::pump_host_to_link() {
  if (!link::connected()) {
    // 未接続時の滞留データは再接続時に古いまま流れないよう破棄する（FR-01）
    const uint32_t stale =
        ring_buf_get(host_to_link_, nullptr, ring_buf_size_get(host_to_link_));
    if (stale > 0) {
      link_monitor::note_drop(stale);
    }
    return;
  }
  const uint16_t mtu = link::max_payload();
  if (mtu == 0) {
    return;
  }
  const uint32_t chunk_max =
      (mtu < TX_CHUNK_MAX) ? static_cast<uint32_t>(mtu) : TX_CHUNK_MAX;
  uint8_t buf[TX_CHUNK_MAX];
  for (;;) {
    const uint32_t n = ring_buf_get(host_to_link_, buf, chunk_max);
    if (n == 0) {
      return;
    }
    if (!link::send(buf, static_cast<uint16_t>(n))) {
      link_monitor::note_send_fail();
      link_monitor::note_drop(n);
      continue;
    }
    link_monitor::note_tx(n);
  }
}

void uart_bridge::pump_link_to_host() {
  uint8_t buf[128];
  for (;;) {
    const uint32_t n = ring_buf_peek(link_to_host_, buf, sizeof(buf));
    if (n == 0) {
      return;
    }
    const int written = host_uart::write(buf, n);
    if (written <= 0) {
      // UART TX リングが一杯: 破棄せず link_to_host_ に残し、
      // 次ポーリングで残りから再送する（バックプレッシャー）。
      return;
    }
    link_monitor::note_rx(static_cast<uint32_t>(written));
    (void)ring_buf_get(link_to_host_, nullptr, static_cast<uint32_t>(written));
  }
}

}  // namespace uwc
