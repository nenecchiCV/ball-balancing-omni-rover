#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <cstdint>

namespace uwc {

// 無線トランスポートの統一静的インターフェース。static class。
// 実装は Kconfig UWC_TRANSPORT_* で選択したソース（link_ble.cpp /
// link_wifi.cpp）が提供し、上位層はトランスポート種別を知らない。
class link {
 public:
  link() = delete;

  // 接続状態の抽象表現（BLE advertising / Wi-Fi AP待受 を含む）
  enum class state : uint8_t {
    IDLE,
    WAITING,  // advertising / 接続待受中
    CONNECTED,
  };

  // link から上位層への状態遷移通知（init の config で注入）
  enum class event : uint8_t {
    WAITING_STARTED,
    CONNECTED,
    DISCONNECTED,
    MTU_UPDATED,
  };

  struct config {
    struct ring_buf* rx_ring;  // アプリ→ホスト方向の受信先
    struct k_sem* rx_sem;      // 受信通知用セマフォ
    void (*event_cb)(event ev);
  };

  static bool init(const config& cfg);
  static bool connected();
  static state link_state();

  // 1 送出あたりのペイロード上限。未接続時は 0。
  static uint16_t max_payload();

  // 無線リンクへ len バイト送出（len <= max_payload()）。失敗時 false。
  static bool send(const uint8_t* data, uint16_t len);

  // link 層で破棄した受信バイト数（リング溢れ等、統計参照用）
  static uint32_t rx_dropped();

  // 診断用文字列（トランスポート種別と接続先等、"ble ..."/"wifi ..."）
  static const char* describe(char* buf, uint32_t len);
};

}  // namespace uwc
