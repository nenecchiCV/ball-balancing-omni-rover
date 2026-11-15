# converter_application

全コンポーネントの設定注入・初期化・実行ループを司る composition root。`converter_application` static class（システムに 1 つ）。

## 公開型

```cpp
struct config {
    /* 現時点では空。将来の帯域・ボーレート等を注入する余地を残す */
};
```

## 公開関数

- `static bool init(const config&)` — `host_uart`・`link`・`uart_bridge`・`link_monitor` を `DT_ALIAS(host-uart)` の UART デバイスとリングバッファで配線して初期化し、BLE（または選択トランスポート）を起動する。失敗時は false。
- `[[noreturn]] static void run()` — `uart_bridge::run()` へ制御を渡す。
- `static link_monitor::stats stats()` — shell からの統計照会用。
- `static void reset_stats()` — `uwc stats reset` 用に `link_monitor` の統計をクリアする。

## shell コマンド

| コマンド | 内容 |
|---|---|
| `uwc status` | トランスポート種別・接続状態・（BLE 時）MTU/PHY/接続先・最終活動時刻を表示（`status_cmd_cb`） |
| `uwc stats [reset]` | 転送統計表示、`reset` でカウンタクリア（`stats_cmd_cb`） |

## 内部状態

- `host_uart`/`link`/`uart_bridge` へ渡す `host_to_link`・`link_to_host` の 2 つの静的 `ring_buf`、コンポーネント参照の `static inline` メンバ。

## 備考

- `main.cpp` 側は `init(cfg)` → `run()` を呼ぶだけに留める。
- `uart_bridge` の `bridge_sem` は `host_uart` と `link` が `rx_sem` として共有し、どちらからの受信でもブリッジを起こす。
