#include "link_test/link_test.hpp"

#include <stdio.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include "common/link_protocol/link_protocol.hpp"
#include "device/estop_input/estop_input.hpp"
#include "device/lsm6dso/lsm6dso.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/link_uart/link_uart.hpp"
#include "peripheral/timebase/timebase.hpp"
#include "uart_echo_test/uart_echo_test.hpp"

namespace bb {

namespace {

// PONG 能力ビット: テストビルド（bit3）のみ。本番機能ビットは立てない。
constexpr uint8_t TEST_ROBOT_CAPS = 0x08;
constexpr uint32_t STAT_PERIOD_MS = 1000;

link_protocol::frame_decoder decoder_{};
uint8_t rx_scratch_[128] = {};
uint8_t frame_buf_[link_protocol::MAX_FRAME] = {};
uint32_t rx_frames_ = 0;
uint32_t tx_frames_ = 0;
uint32_t last_stat_ms_ = 0;
bool imu_ok_ = false;

void send_frame(uint8_t type, uint8_t seq, const uint8_t* payload,
                uint8_t len) {
  const uint32_t n =
      link_protocol::encode(type, seq, payload, len, frame_buf_);
  // link_uart::write は内部で排他を持たないため TX ISR との整合を
  // irq_lock で保護する（制御スレッド=テストスレッド上の呼出し）。
  const unsigned int key = irq_lock();
  const uint32_t wrote = link_uart::write(frame_buf_, n);
  irq_unlock(key);
  if (wrote == n) {
    ++tx_frames_;
  }
}

void send_ack(uint8_t req_type, uint8_t seq, uint8_t result) {
  const uint8_t pl[3] = {req_type, seq, result};
  send_frame(link_protocol::resp::ACK, 0, pl, sizeof(pl));
}

void respond(const link_protocol::request& req) {
  switch (req.type) {
    case link_protocol::req::PING: {
      const uint8_t r[3] = {req.seq, link_protocol::PROTO_VER,
                            TEST_ROBOT_CAPS};
      send_frame(link_protocol::resp::PONG, 0, r, sizeof(r));
      break;
    }
    case link_protocol::req::GET_STATUS: {
      // テストビルドは常時待機（op_state=0 / mode=0 / enable=0）。
      // 健全性フィールドは実測値を返す。
      uint8_t r[11];
      r[0] = req.seq;
      r[1] = 0;  // op_state: standby
      r[2] = 0;  // mode: standby
      r[3] = 0;  // enable
      r[4] = estop_input::requested() ? 1 : 0;
      r[5] = imu_ok_ ? 1 : 0;
      r[6] = 0;  // calib_state: none
      link_protocol::write_u32(&r[7], timebase::millis());
      send_frame(link_protocol::resp::STATUS, 0, r, sizeof(r));
      break;
    }
    case link_protocol::req::TEST_ECHO: {
      uint8_t r[1 + link_protocol::MAX_PAYLOAD];
      r[0] = req.seq;
      memcpy(&r[1], req.payload, req.payload_len);
      send_frame(link_protocol::resp::TEST_ECHO_RESP, 0, r,
                 static_cast<uint8_t>(req.payload_len + 1));
      break;
    }
    default:
      send_ack(req.type, req.seq, link_protocol::ack_result::UNSUPPORTED);
      break;
  }
}

}  // namespace

bool link_test::start_cb(int argc, char** argv) {
  (void)argc;
  (void)argv;
  // USART1 をバイトエコーから link_uart（プロトコル用 ISR）へ切替える。
  if (!link_uart::init({DEVICE_DT_GET(DT_ALIAS(link_uart))})) {
    debug_uart::write_line("link test: usart1 not ready");
    return false;
  }
  decoder_.reset();
  rx_frames_ = 0;
  tx_frames_ = 0;
  last_stat_ms_ = timebase::millis();
  debug_uart::write_line("link test: protocol responder started");
  return true;
}

void link_test::update_cb() {
  // 10 ms 周期呼出（period_ticks=10）。受信排水→応答→統計報告。
  imu_ok_ = lsm6dso::read().valid;

  for (;;) {
    const uint32_t n = link_uart::rx_read(rx_scratch_, sizeof(rx_scratch_));
    if (n == 0) {
      break;
    }
    for (uint32_t i = 0; i < n; ++i) {
      decoder_.push(rx_scratch_[i]);
    }
    link_protocol::request req{};
    while (decoder_.poll(&req)) {
      ++rx_frames_;
      respond(req);
    }
  }

  const uint32_t now = timebase::millis();
  if (now - last_stat_ms_ >= STAT_PERIOD_MS) {
    last_stat_ms_ = now;
    char line[96];
    const int len = snprintf(
        line, sizeof(line),
        "link t=%lu rx_frames=%lu crc_drops=%lu rx_overruns=%lu "
        "tx_frames=%lu\r\n",
        static_cast<unsigned long>(now), static_cast<unsigned long>(rx_frames_),
        static_cast<unsigned long>(decoder_.dropped()),
        static_cast<unsigned long>(link_uart::rx_dropped()),
        static_cast<unsigned long>(tx_frames_));
    debug_uart::write(line, static_cast<uint32_t>(len));
  }
}

void link_test::stop_cb() {
  const struct device* dev = DEVICE_DT_GET(DT_ALIAS(link_uart));
  uart_irq_rx_disable(dev);
  uart_irq_tx_disable(dev);
  // USART1 をバイトエコーへ返還する（`test echo` の疎通確認経路を復帰）。
  uart_echo_test::resume();
  debug_uart::write_line("link test: stopped");
}

}  // namespace bb
