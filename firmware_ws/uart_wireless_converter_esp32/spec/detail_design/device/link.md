# link

無線トランスポートの統一静的インターフェース。`link` static class（システムに 1 つ）。実装は Kconfig `UWC_TRANSPORT_*` に応じて `link_ble.cpp` または `link_wifi.cpp` が選択され、上位層はトランスポート種別を知らない。

## 公開型

```cpp
enum class state : uint8_t { IDLE, WAITING, CONNECTED };

enum class event : uint8_t {
    WAITING_STARTED, CONNECTED, DISCONNECTED, MTU_UPDATED,
};

struct config {
    struct ring_buf* rx_ring;   /* アプリ→ホスト方向の受信先 */
    struct k_sem* rx_sem;       /* 受信通知用セマフォ */
    void (*event_cb)(event ev); /* 状態遷移通知（上位層が注入） */
};
```

## 公開関数

- `static bool init(const config&)` — トランスポートのスタック起動・サービス登録を行い、受信バイトを `rx_ring` に積み `rx_sem` で通知するよう構成する。起動失敗時は false。
- `static bool connected()` — リンクが確立しているかを返す。
- `static state link_state()` — 内部状態（IDLE/WAITING/CONNECTED）を返す。診断・監視用。
- `static uint16_t max_payload()` — 1 送出あたりのペイロード上限を返す。BLE 時は接続中 ATT MTU-3、未接続時は 0。
- `static bool send(const uint8_t* data, uint16_t len)` — 無線リンクへ `len` バイトを送出する。`len` は `max_payload()` 以下。リンク未確立・送出不能時は false で返し、呼出側の統計に反映させる。
- `static uint32_t rx_dropped()` — link 層が破棄した受信バイト数（`rx_ring` 溢れ）を返す。
- `static const char* describe(char* buf, uint32_t len)` — 診断用文字列（トランスポート種別・接続先・MTU・購読状態）を `buf` に書き込んで返す。

## 内部状態（link_ble.cpp の場合）

- 内部状態は無名名前空間の POD `impl` に集約する: BT 接続 `bt_conn*` 参照、CCC 購読フラグ、現在状態、受信リング・セマフォ参照、注入された `event_cb`、ドロップカウンタ。
- Zephyr BT の接続・切断・MTU 更新・CCC 変化コールバックから `impl.event_cb` を呼び、上位（`link_monitor`）へ通知する。`link` は上位層を直接参照しない。

## 備考

- BLE 実装では NUS 互換 UUID のサービスを `BT_GATT_SERVICE_DEFINE` で静的登録し、RX 特性 Write コールバックが `rx_ring` に積む。TX 通知は `bt_gatt_notify` を `send()` から呼ぶ。
- Wi-Fi 実装は SoftAP + TCP/UDP で同一静的シグネチャを持つ `link` を提供する。
