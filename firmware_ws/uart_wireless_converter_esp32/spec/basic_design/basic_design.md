# 基本設計書

`uart_wireless_converter_esp32` のソフトウェア構成・実行モデル・実装方針を定義する。本ファームウェアは XIAO ESP32-C6 上で動作し、ホスト MCU（NUCLEO-G474RE）の UART と無線リンクの間でバイトストリームを透過する。初版の無線トランスポートは BLE（NUS 相当 GATT）とし、Wi-Fi への切替を要件に応じて後から選択できる構成とする。

## 実行基盤

- **RTOS**: Zephyr 4.3（Zephyr SDK 0.17.4、riscv64-zephyr-elf）。対象ボード `xiao_esp32c6/esp32c6/hpcore`。
- **ハードウェア記述**: UART・無線ノードはすべて devicetree（ボード標準 dts + `boards/xiao_esp32c6_esp32c6_hpcore.overlay` の alias）で宣言し、コードは `DT_*` マクロ経由でのみ参照する。`spec/requirements/hardware_config.md` は人間可読な対応表とする。
- **ビルド**: `west` 経由の Zephyr アプリケーション。Zephyr 本体・HAL モジュールはワークスペース（`C:\zephyr_ws`）に置き、アプリ側ソースへ混入させない。ESP32 無線には `hal_espressif` とバイナリ blob（`west blobs fetch hal_espressif`）が必要。
- **C++**: 制限付き C++（例外・RTTI・動的確保・STL 不使用）。STL 相当が必要な箇所は ETL（`third_party/etl`、git submodule、ヘッダオンリー）で代替する。

## ソフト構成

### 層構造

下から上への一方向依存のみ許可する（同一層内は循環しない範囲で許可）。

| 層 | 責務 | 例 |
|---|---|---|
| `manager` | 機能管理・コンポーネント配線 | `converter_application` |
| `feature` | 機能（中継・リンク監視） | `uart_bridge`, `link_monitor` |
| `device` | 無線トランスポート実装 | `link`（`link_ble.cpp`/`link_wifi.cpp`） |
| `peripheral` | MCU ペリフェラル制御 | `host_uart`, `debug_console`, `status_led` |
| `common` | 純粋ユーティリティ・定数 | （現時点では該当なし） |

### トランスポート抽象（FR-04）

- `device/link` の `link` static class が無線トランスポートの統一静的インターフェース（`init`/`connected`/`max_payload`/`send`）を定義する。`link_ble.cpp`（BLE）と `link_wifi.cpp`（将来）は同一クラス名の別実装とし、CMake が Kconfig `UWC_TRANSPORT_*` に応じて一方のみをコンパイルする。
- `feature` 層以降はトランスポート種別を知らない。継承は使わず、切替はコンパイル単位の選択で行う。

### クラス設計ルール

- 1 コンポーネント = 1 クラス。継承は使わない。
- システムに 1 つ常駐するコンポーネントは static class（全メンバ `static`、ctor `= delete`）とする。
- インターフェースは値型で受け渡す。`float` 前提の処理はない（バイトストリームのみ）。

## データフロー

```
[G474 USART1] <--UART0--> [host_uart] <--ring--> [uart_bridge] <--API--> [link (BLE)]
                                                                    ↑ NUS 相当 GATT
[usb_serial] <-console/shell-> [debug_console]              [link_monitor] = 状態・統計
[GPIO15 led0] <--GPIO--> [status_led] <-mode- [link_monitor]
```

- **ホスト→アプリ**: UART0 RX（割込）→ 静的 `ring_buf`（`host_to_link`）→ ブリッジ → `link::send()` で BLE Notification。ペイロードは接続中 MTU（実効 MTU-3）を超えない単位に分割する。
- **アプリ→ホスト**: RX 特性 Write（BT コンテキスト）→ 静的 `ring_buf`（`link_to_host`）→ ブリッジ → `host_uart::write()` で UART0 TX へ送出。
- リングバッファは `uart_bridge` が静的に保持し、両方向のプロデューサ（UART ISR / BT コンテキスト）が書き込む。いずれも単一プロデューサ/単一コンシューマのため排他は不要（`ring_buf` 自体はシングルプロデューサ/コンシューマ設計）。

## タスク・スレッド管理

| 実行主体 | 役割 | 備考 |
|---|---|---|
| UART0 割込ハンドラ | 受信バイトを `host_to_link` リングへ移し `bridge_sem` を give | ISR、処理を長引かせない |
| BT コンテキスト（Zephyr 内部） | RX 特性 Write → `link_to_host` リングへ移し `bridge_sem` を give / 接続イベントを `link_monitor` へ通知 | スタックスレッドでブロックしない |
| ブリッジスレッド（`uart_bridge`） | `bridge_sem` 待ち+短周期タイムアウトで両リングを排出 | ポーリング間隔は NFR-01 の目標内 |
| shell スレッド | `uwc` コマンド処理 | `link_monitor` の snapshot 参照のみ |
| LED 点滅ワーク（system workqueue） | BLINK モード中に 500 ms 周期で LED をトグル | 専用スレッドなし、`k_work_delayable` で駆動（FR-05） |
| adv 再開ワーク（system workqueue） | 切断・接続失敗後の connectable advertising 再開 | `k_work_delayable` で駆動、失敗時リトライ（FR-02） |
| main スレッド | `init` 後 `converter_application::run()` へ委譲 | — |

## 通信

- **ホスト**: UART0（115200 8N1、devicetree `current-speed`）を `host_uart` が割込駆動で運用する。フレーミングは行わず生バイトを透過する。
- **無線**: `link` が BLE ペリフェラルを構成する。Advertising・接続・MTU/PHY 更新は Zephyr BT API 経由で、CCC 購読状態に応じて Notification を送出する。未接続・未購読時の UART→BLE データは破棄する（FR-01）。
- **診断**: `usb_serial`（USB-Serial-JTAG）に console/shell。`debug_console` 経由で `printk` を用いる。UART0 と診断経路は分離し、透過ストリームを汚染しない。

## メモリ管理

- アプリケーション側はヒープを使わない（NFR-04）。リングバッファ・接続コンテキストは静的/スタック確保する。
- ESP32 無線スタック内部の確保のみカーネルヒープを許容（`CONFIG_HEAP_MEM_POOL_SIZE=32768`）。
- `etl` の固定容量コンテナのみ使用可能とし、可変長コンテナは使わない。

## 言語規約（制限付き C++）

- 標準は C++23（`CONFIG_STD_CPP23`）。**STL 禁止**・**例外禁止**・**RTTI 禁止**・**動的確保禁止**（アプリ側）。
- `CONFIG_STATIC_INIT_GNU=n` により実行時コンストラクタは走らない。static メンバ・グローバルはゼロ/定数初期化のみで成り立たせ、非ゼロの値は `init()` で設定する。Zephyr のサービス定義マクロ（`BT_GATT_SERVICE_DEFINE` 等）が生成する初期化子はこの方針と両立する範囲で利用する。
- **static class ルール**・**ライフサイクル**（`init(const config&)`/`start()`/`stop()`）・**命名規則**（`snake_case`、定数のみ `UPPER_SNAKE_CASE`、コールバック `*_cb`）は `ball_balancing_omni3_stm32` と同一とする。

## 実装方針

- **GATT 定義**: NUS 相当サービスは `BT_GATT_SERVICE_DEFINE` で静的に登録し、RX Write コールバックで `link_to_host` リングへ積む。TX Notification は接続中 `att` の実効 MTU に合わせて分割する。
- **初期化**: `converter_application` が `init()` で `host_uart`・`link`・`uart_bridge`・`link_monitor` を配線し、BLE を起動して advertising を開始する。`run()` はブリッジループに制御を渡す。
- **エラーポリシー**: ペリフェラル・スタックの起動失敗は `init` の戻り値 false で表し、`usb_serial` シェル上で診断可能なログを残す。実行中の一時的な送出失敗は統計として記録し、機能は継続する。
- **状態表示**: `link` の遷移イベントを `link_monitor` が `status_led` の表示モードへ写像し、ユーザー LED（`led0`、GPIO15）を advertising 中は 1 s 周期点滅、接続中は常時点灯、初期化前は消灯とする（FR-05）。点滅の周期生成は専用スレッドを持たず `k_work_delayable`（system workqueue）で行う。

## 安全性

- 無線リンクの断絶自体は本モジュールの責務とせず、UART の無応答としてホスト側（G474）の安全機構が検出する前提（`ball_balancing_omni3_stm32` 側でタイムアウト→停止）。本モジュールは `link_monitor` で活動時刻を記録し、診断から参照できる状態に留める。
