# uart_bridge

UART0 と無線リンクの間でバイトストリームを双方向中継するブリッジ。`uart_bridge` static class（システムに 1 つ）。ブリッジスレッドで稼働する。

## 公開型

```cpp
struct config {
    struct ring_buf* host_to_link;  /* UART→無線 の中継リング */
    struct ring_buf* link_to_host;  /* 無線→UART の中継リング */
    uint32_t poll_ms;               /* イベント待ちの最大間隔 */
};
```

## 公開関数

- `static bool init(const config&)` — リング参照を保持し、ブリッジ用 `k_sem` を初期化する。`init()` 後に呼ぶこと。
- `[[noreturn]] static void run()` — ブリッジスレッド本体。`rx_sem_` 待ち + タイムアウトで周期的に両方向を排出する。
- `static struct k_sem* rx_sem()` — 生産側（UART ISR / link コールバック）が `rx_sem` として共有するセマフォを返す。`init()` 済みであれば初期化済みの `k_sem` を指す。

## 内部状態

- `bridge_sem_`（受信イベント）・リング参照・統計カウンタ（`host_to_link` 排出数、`link_to_host` 排出数、ドロップ数）。

## 備考

- 単一スレッドで両リングを排出するため、リングのコンシューマ側排他は不要。プロデューサは UART ISR と BT コンテキストの 2 者だが、リングは方向ごとに独立している。
- 送出時は `link::max_payload()` で分割して `link::send()` を呼ぶ。送出失敗はリトライせず破棄として統計に記録し、ストリームを進める（FR-01 の破棄方針）。
