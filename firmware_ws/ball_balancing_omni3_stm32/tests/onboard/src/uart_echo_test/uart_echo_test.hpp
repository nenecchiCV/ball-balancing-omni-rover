#pragma once

#include <cstdint>

namespace bb {

// USART1 バイトエコーテスト（static class）。
// 無線変換（uart_wireless_converter_esp32）経由のシリアル通信を
// エンドツーエンドで検証するため、USART1 で受信したバイトを
// そのまま USART1 TX へ折り返す。受信は割込でリングへ積み、
// 排出スレッドが uart_poll_out で返送する。
// エコー自体は init() 時点から常時有効（shell 不要の疎通確認用）。
// `test echo`/`test stop` はエコーの再有効化と転送統計の報告を行う。
class uart_echo_test {
 public:
  uart_echo_test() = delete;

  static bool init();

  // test_runner から呼ばれるコールバック。
  static bool start_cb(int argc, char** argv);
  static void update_cb();
  static void stop_cb();

  // USART1 を他コンポーネント（link_test）から返還し、バイトエコーの
  // 割込コールバックと受信を再有効化する。
  static void resume();
};

}  // namespace bb
