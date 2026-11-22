# link_uart

USART1（115200 8N1、PC4=TX / PC5=RX）の割込駆動 UART。`uart_wireless_converter_esp32` 経由 BLE リンクのバイトストリームを送受する。`k_sem` で受信をリンクスレッドへ通知し、TX は割込駆動リングで呼出し側をブロックしない。

## 公開型

```cpp
class link_uart {  // static class
 public:
  struct config { const struct device* dev; };
};
```

## 公開関数

- `static bool init(const config& cfg)` — デバイスレディ確認、`uart_irq_callback_user_data_set` で `uart_isr_cb` を登録し `uart_irq_rx_enable`。
- `static struct k_sem* rx_sem()` — RX リングへバイト投入ごとに give されるセマフォ。リンクスレッドの `k_poll` 対象。
- `static uint32_t rx_read(uint8_t* buf, uint32_t max_len)` — RX リングから排他なく取り出す（呼出しはリンクスレッドに限定）。取り出したバイト数を返す。
- `static uint32_t write(const uint8_t* buf, uint32_t len)` — TX リングへ投入し `uart_irq_tx_enable`。投入できたバイト数を返す（フル時は短い値。割込が FIFO を消費して送出する）。呼出しはリンクスレッドに限定。

## 内部状態

- `rx_buf_[256]` / `rx_head_`・`rx_tail_`（volatile、ISR 書込み・スレッド読出しの単一方向リング。オーバーラン時は捨てて `rx_dropped_` 計数）。
- `tx_buf_[512]` / `tx_head_`・`tx_tail_`（スレッド書込み・ISR 読出しの単一方向リング）。
- `rx_sem_`（`k_sem`、ISR → スレッド通知）。
- `uart_isr_cb`: RX は `uart_fifo_read` → リング投入 → `rx_sem_give`。TX は `uart_irq_tx_ready` 時に `uart_fifo_fill` でリングを吐き、空で `uart_irq_tx_disable`。

## 備考

- `uart_poll_out` は使わない（TX を割込駆動にして制御スレッド側のブロッキングを排除するため）。
- ボーレートは devicetree の `current-speed`（115200）が正本。`uart_configure` は呼ばない。
