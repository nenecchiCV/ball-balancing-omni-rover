# host_uart

ホスト MCU（G474 USART1）と接続する UART0 の割込駆動ラッパ。`host_uart` static class（システムに 1 つ）。UART ドライバへの依存を peripheral 層に閉じ込める。

## 公開型

```cpp
struct config {
    const struct device* uart_dev;   /* DT_ALIAS(host-uart) */
    struct ring_buf* rx_ring;        /* 受信バイトの書き込み先 */
    struct k_sem* rx_sem;            /* 受信通知用セマフォ */
};
```

## 公開関数

- `static bool init(const config&)` — `uart_dev` の準備確認、割込駆動（`uart_irq_callback_user_data_set` + `uart_irq_rx_enable`）を有効化し、`rx_ring`/`rx_sem` を保持する。
- `static int write(const uint8_t* data, uint32_t len)` — 内部 TX リングへ積んで `uart_irq_tx_enable` で送出を開始する。積めたバイト数を返し、不足分は呼出側が破棄として扱う。
- `static uint32_t rx_dropped()` — `rx_ring` 溢れで破棄した受信バイト数を返す（統計参照用）。

## 内部状態

- `uart_dev_`、`rx_ring_`、`rx_sem_`、TX リング（`tx_ring_` + `tx_mem_`）、`rx_drop_count_` の static メンバ。
- ISR コールバック `uart_isr_cb` が `uart_irq_rx_ready` で `uart_fifo_read` → `ring_buf_put` → `rx_sem` give。TX 側は `uart_irq_tx_ready` で `tx_ring_` から `uart_fifo_fill` し、空になれば `uart_irq_tx_disable`。
- `rx_ring` のコンシューマはブリッジが直接 `ring_buf_get` するため `read()` は提供しない。

## 備考

- エラー・オーバーランは `uart_irq_err_pending` で検出し、統計へ載せる場合は `link_monitor` に通知するフックを設ける（現時点では記録のみ）。
