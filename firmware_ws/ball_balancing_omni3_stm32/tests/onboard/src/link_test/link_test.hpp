#pragma once

#include <cstdint>

namespace bb {

// ロボットリンクプロトコル応答テスト（static class、spec/test/link_test.md）。
// `uart_echo_test` が USART1 を占有するため、`test link` 実行中は
// link_uart へ所有権を移し、停止時にバイトエコーへ返還する。
// 受理: PING→PONG（caps=テストビルド bit3）、GET_STATUS→STATUS、
// TEST_ECHO→TEST_ECHO_RESP、その他→ACK(UNSUPPORTED)。
class link_test {
 public:
  link_test() = delete;

  // test_runner から呼ばれるコールバック。
  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();
};

}  // namespace bb
