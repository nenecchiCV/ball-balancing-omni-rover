# 要件仕様書

`robot_controller_tauri` アプリケーションの機能要求・非機能要求を ID 付きで管理する。本アプリは Next.js（フロントエンド）× Tauri 2（Rust バックエンド）製のデスクトップアプリであり、`uart_wireless_converter_esp32` が提供する BLE NUS 透過リンク経由で `ball_balancing_omni3_stm32` の玉乗りオムニローバーを操作・監視する。

```
[robot_controller_tauri] <--BLE NUS--> [XIAO ESP32-C6] <--UART 115200--> [G474 USART1]
```

アプリとロボットの間のアプリケーションプロトコルは `spec/basic_design/robot_link_protocol.md` を正本とする。ロボット側がこのプロトコルを処理するために必要なファームウェア側要求は `firmware_ws/ball_balancing_omni3_stm32/spec/requirements/requirements.md` の FR-09〜FR-11 にトレースする。

## 機能要求

| ID | 要求 | 個別仕様 |
|---|---|---|
| FR-01 | BLE でロボットを検出・接続・切断する | [functional/FR-01.md](functional/FR-01.md) |
| FR-02 | 画面上の 2 本のジョイスティックで速度指令を送る | [functional/FR-02.md](functional/FR-02.md) |
| FR-03 | 目標位置と速度を入力して位置制御させる | [functional/FR-03.md](functional/FR-03.md) |
| FR-04 | 自己位置推定結果をワールド座標で表示する | [functional/FR-04.md](functional/FR-04.md) |
| FR-05 | ロボットのログメッセージを表示する | [functional/FR-05.md](functional/FR-05.md) |
| FR-06 | 通信異常と緊急停止を操作者へ通知・提供する | [functional/FR-06.md](functional/FR-06.md) |

## 非機能要求

| ID | 要求 | 個別仕様 |
|---|---|---|
| NFR-01 | 操作の応答性とテレメトリの実時間性を確保する | [nonfunctional/NFR-01.md](nonfunctional/NFR-01.md) |
| NFR-02 | 誤操作・誤指令を防ぐ安全設計を守る | [nonfunctional/NFR-02.md](nonfunctional/NFR-02.md) |
| NFR-03 | 言語・アーキテクチャ規約を守る | [nonfunctional/NFR-03.md](nonfunctional/NFR-03.md) |
| NFR-04 | ビルド・検証をターミナルから自動実行できる | [nonfunctional/NFR-04.md](nonfunctional/NFR-04.md) |
