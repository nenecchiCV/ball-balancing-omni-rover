# ハードウェア構成仕様

本ファイルは `uart_wireless_converter_esp32` ファームウェアが対象とするモジュールのハードウェア仕様を定義する。ピンアサインの確定は本ファイルを正本とし、`boards/xiao_esp32c6_esp32c6_hpcore.overlay`（devicetree）をその機械実行可能な写像として同期させる。

## 構成要素

| 要素 | 部品 | 概要 |
|---|---|---|
| 無線モジュール | Seeed XIAO ESP32-C6 | ESP32-C6（RISC-V 160 MHz）、4 MB Flash、512 KB SRAM。Wi-Fi 6 + BLE 5.3 + 802.15.4 内蔵、基板実装アンテナ（外付けアンテナ不要）。USB-C 経由で書き込み・コンソール |
| ホスト MCU | NUCLEO-G474RE（`ball_balancing_omni3_stm32`） | USART1（PC4/PC5）に接続。UART バイトストリームの相手側 |

## MCU リソース割当

| 機能 | ペリフェラル | ピン | XIAO 端子 | 備考 |
|---|---|---|---|---|
| ホスト UART TX | UART0 TX | GPIO16 | D6 | G474 USART1 RX（PC5）へ |
| ホスト UART RX | UART0 RX | GPIO17 | D7 | G474 USART1 TX（PC4）へ。DTS で bias-pull-up 済み |
| 診断/書き込み | USB-Serial-JTAG（`usb_serial`） | USB-C | — | Zephyr console/shell + esptool 書き込み |
| 無線 | ESP32-C6 内蔵 RF | — | — | `esp32_bt_hci`（BLE）/ `wifi` ノード |

## インターフェース仕様

### ホスト UART（UART0）

- 115200 8N1（devicetree `current-speed` が正本）。帯域が不足する場合は両端の devicetree を同時に変更して高速化する。
- データはフレーミングなしの生バイトストリームとし、無線リンクの向こう側へそのまま透過する。プロトコル（フレーミング・コマンド）はホスト側アプリ層の責務とし、本ファームウェアは解釈しない。
- UART0 の CTS/RTS は使用しない。フロー制御が必要になった場合は仕様を更新する。

### 無線リンク

- 初版は BLE ペリフェラル（`esp32_bt_hci`、Zephyr BT ホストスタック上の NUS 相当 GATT サービス）。
- Wi-Fi（`wifi` ノード、SoftAP + TCP/UDP）への切替はビルド時の Kconfig 選択とし、無線モジュール・配線は変更しない。

## 電源

- XIAO ESP32-C6 は 3.3 V 動作。G474 側の 3V3 から供給するか、USB 給電とするかは未確定（下記参照）。
- BLE 時の消費電流は数十 mA オーダ見込み。Wi-Fi 切替時は送受信ピーク ~300 mA @3.3 V を見込む。

## 未確定事項

- 電源供給経路（G474 3V3 / Nucleo 5V→XIAO 5V 端子 / USB 給電）と消費電流実測: 実機で確定し、G474 側 5 V 系 200 mA 予算との整合を確認する。
- UART0 の最終ボーレート: 現状 115200。テレメトリ帯域実測後に必要なら引き上げる。
- BLE の接続パラメータ（MTU・接続間隔・PHY）: 実機で調整して定数を確定する。
