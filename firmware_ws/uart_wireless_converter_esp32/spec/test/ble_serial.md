# ble_serial_test — BLE シリアル透過検証

## 目的

PC 側 BLE セントラル（bleak）から NUS 相当 GATT へ接続し、
RX Write→UART0 TX→（G474 USART1 エコー）→UART0 RX→TX Notify の
バイト折返しで、無線↔UART ブリッジの双方向透過を検証する。

## 手順

```bash
python spec/test/scripts/ble_serial_test.py [--shell-port COM4]
```

スクリプトは以下を順に実行し、`spec/test/results/<timestamp>/` に
各段階のログと `summary.md` を出力する。

1. **scan**: 10 s スキャンで `uwc-esp32c6`（または NUS サービス UUID
   `6e400001-…`）を検出。RSSI・advertising 一覧を `00_scan.log` へ記録。
2. **gatt**: 接続しサービス/特性を列挙（`01_gatt.log`）。RX/TX 特性の
   存在と MTU を確認。接続失敗時は 1 回リトライする。
3. **echo small**: `uwc-echo-test\n`（14 B）を RX Write（response=False）
   し TX Notify で折返しを待つ。RTT を計測。
4. **echo 256B**: `0x00..0xFF` パターンを往復し、全バイト一致を確認。
5. **echo stream**: `--stream-bytes`（既定 1024 B）を実効 MTU-3 単位で
   連続 Write し、折返し全体の一致と実効スループットを計測。
6. **reconnect**: 切断→2 s 待機→再スキャン→再接続→往復で、
   FR-02 の自動再 advertising と再接続動作を確認。
7. **stm32 echo stats**（`--shell-port` 指定時のみ）: ST-Link シェルへ
   `test echo`/`test stop` を送り、G474 側の rx/tx/drop 統計を採取する。

## ログ形式

- `summary.md`: 各テストの PASS/FAIL/REVIEW と計測値の表。
- `NN_<name>.log`: 段階ごとの生データ（advertising 一覧、GATT dump、
  送受信バイト列、RTT、スループット）。

## 合否基準

- scan/gatt: 対象デバイスと NUS 特性が検出されること。
- echo 系: 送信バイト列と受信バイト列が完全一致すること。
  （`hb` マーカーはエコー 2 秒無通信時のみのため、往復中は混入しない）
- stream: データ欠落・化けなしで全量折り返すこと。
- reconnect: 切断後に再 advertising が観測され、再接続で往復可能なこと。

## 対応コンポーネント

- 被検: `link`（link_ble）、`uart_bridge`、`host_uart`、`link_monitor`
- 相手側: `ball_balancing_omni3_stm32` の `tests/onboard/src/uart_echo_test`
- スクリプト: `spec/test/scripts/ble_serial_test.py`（bleak 依存）
