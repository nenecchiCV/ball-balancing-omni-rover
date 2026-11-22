# テスト計画

`uart_wireless_converter_esp32` の実機検証方法を定義する。
検証対象は BLE↔UART 透過ブリッジで、ホスト MCU（NUCLEO-G474RE USART1）
まで含めたエンドツーエンドのシリアル通信を確認する。

## 検証トポロジ

```text
[PC (bleak)] <--BLE--> [XIAO ESP32-C6 (本ファーム)] <--UART0 115200-->
  [NUCLEO-G474RE USART1 (onboard test fw `uart_echo_test`)]
```

- PC は BLE セントラルとして NUS 相当 GATT（service `6E400001-…`、
  RX Write `6E400002-…`、TX Notify `6E400003-…`）へ接続する。
- G474 側は onboard テストファームの `uart_echo_test` が USART1 の
  受信バイトをそのまま返送する（ブート時から常時有効。2 秒以上の
  アイドルでは `hb` マーカーも送出）。
- よって PC→STM32→PC のバイト折返し（ラウンドトリップ）で
  双方向の透過を同時に検証できる。

## 前提

- 本ファームウェアが XIAO ESP32-C6 に書き込まれ、給電・advertising 中
  （LED は 1 s 周期点滅）。
- NUCLEO-G474RE に `uart_echo_test` を含む onboard テストファームが
  書き込まれ、USART1（PC4/PC5）と XIAO UART0（GPIO16/17）が結線済み。
- PC に Bluetooth LE アダプタと `pip install bleak` 済みの Python 3。
- モータ・バッテリー不要（モータ非接続で実施可能）。

## 実行

```bash
python spec/test/scripts/ble_serial_test.py
# 結果は spec/test/results/<timestamp>/ に生ログと summary.md を出力。
```

オプション: `--name`（デバイス名）、`--stream-bytes`（ストリーム長）、
`--shell-port COMx`（ST-Link シェルから STM32 側 `test echo` 統計も採取）。

## テスト一覧

| テスト | 内容 | 詳細仕様 |
|---|---|---|
| scan | `uwc-esp32c6` / NUS UUID の advertising 検出 | [ble_serial.md](ble_serial.md) |
| gatt | 接続・NUS サービス/特性発見・MTU | 〃 |
| echo small | 14 B メッセージ往復 | 〃 |
| echo 256B | 全バイト値パターン往復 | 〃 |
| echo stream | 連続ストリーム往復・実効スループット | 〃 |
| reconnect | 切断→再 advertising→再接続→往復 | 〃 |

## 既知の事象

- セントラル切断後に再 advertising しない・LED 消灯で無応答となる
  事象を過去に観測（`disconnected_cb` からの直接 `bt_le_adv_start`
  失敗で IDLE 停滞）。再開を system workqueue へ遅延委譲し失敗時
  リトライする対策済み。再現時は `reconnect` テストが FAIL となる。
- ローカルの ATT MTU 上限は prj.conf で 247 B に設定済み。
  セントラル側が小さい MTU で応答する環境では実効スループットは
  その MTU に制限される（MTU 23 時は ~19 kbps 程度）。
