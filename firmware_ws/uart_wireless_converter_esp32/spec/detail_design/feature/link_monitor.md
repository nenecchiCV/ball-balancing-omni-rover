# link_monitor

無線リンクの接続遷移・転送統計・活動時刻を記録し、診断スナップショットを提供する。`link_monitor` static class（システムに 1 つ）。

## 公開型

`link` が公開する `link::event`（`WAITING_STARTED`/`CONNECTED`/`DISCONNECTED`/`MTU_UPDATED`）をそのまま受け取る。

```cpp
struct stats {
    uint32_t bytes_host_to_link;
    uint32_t bytes_link_to_host;
    uint32_t send_fail_count;
    uint32_t drop_bytes;
    uint32_t connect_count;
    uint32_t disconnect_count;
    uint32_t last_activity_ms;
};
```

## 公開関数

- `static void init()` — カウンタ・状態を初期化する。
- `static void on_event(link::event ev)` — `link` の `event_cb` から呼ばれ、状態遷移を記録する。あわせて `status_led` の表示モードを更新する（`WAITING_STARTED`→`BLINK`、`CONNECTED`→`SOLID`、`DISCONNECTED`→`OFF`、FR-05）。LED 制御は `irq_lock` 区間の外で行う。
- `static void note_tx(uint32_t bytes)` / `static void note_rx(uint32_t bytes)` — 転送バイト数と活動時刻を更新する。
- `static void note_drop(uint32_t bytes)` / `static void note_send_fail()` — 破棄・送出失敗を記録する。
- `static stats snapshot()` — shell 参照用に `irq_lock()` で整合コピーを返す。
- `static void reset()` — 統計カウンタのみクリアする（接続状態は保持）。
- `static bool connected()` — 現在接続中かを返す。

## 内部状態

- `stats` メンバ、最終接続状態フラグ、（BLE 時）接続先アドレス・MTU・PHY のキャッシュ。

## 備考

- `link` 実装のコールバックから呼ばれる関数は `*_cb` 命名にしない（イベント通知関数として設計する）。Zephyr BT コールバック自体は `link` 内で `*_cb` として定義し、そこから `link_monitor::on_event` を呼ぶ。
