# uart_echo_test — USART1 バイトエコー

## 目的

`uart_wireless_converter_esp32`（XIAO ESP32-C6、BLE↔UART 変換）の
シリアル透過をエンドツーエンドで検証するため、USART1（PC4 TX/PC5 RX、
115200 8N1）で受信したバイトをそのまま TX へ折り返す。
無線リンク越しの往復検証を PC 側だけで完結させる。

## 前提

- テストファームウェア書き込み済み。
- XIAO ESP32-C6 がシールド上に実装され、USART1 と UART0 が結線済み。
- エコー自体は `uart_echo_test::init()` 時点から常時有効であり、
  shell 操作なしで動作する（リモート側からの疎通確認用）。

## 手順

```text
uart:~$ test echo
echo test: USART1 byte echo stats started
echo t=1000 rx=0 tx=0 drop=0
echo t=2000 rx=20 tx=20 drop=0
```

1. `test echo` で転送統計の 1 Hz 報告を開始する（エコーは常時有効）。
2. 無線側（BLE クライアント）から USART1 へバイトを送信し、
   `rx`/`tx` が一致して増えることを確認する。
3. `test stop` で統計報告とエコーを停止する。

## ログ形式

```text
echo t=<ms> rx=<bytes> tx=<bytes> drop=<bytes>
```

- `rx`: USART1 受信割込で読み取った累計バイト数。
- `tx`: USART1 TX へ返送した累計バイト数。
- `drop`: リング溢れで破棄した累計バイト数。
- エコーが 2 秒以上無い間は USART1 TX へ `hb` マーカーを約 2 秒周期で送出する
  （STM32→ESP32→BLE Notify の折返し経路を送信データなしで確認するため）。

## 合否基準

- 送信バイトが同一内容で折り返されること（自動判定は
  `uart_wireless_converter_esp32/spec/test/scripts/ble_serial_test.py` 側）。
- `drop=0` であること。

## 対応コンポーネント

- `tests/onboard/src/uart_echo_test/`（period 1000 tick = 1 Hz 統計報告）
- 構成: `&usart1`（ボード dts で PC4/PC5・115200 有効済み）、
  `CONFIG_UART_INTERRUPT_DRIVEN=y`
