# 実行環境仕様

本ファイルは `robot_controller_tauri` が対象とする実行環境と対向機器の仕様を定義する。firmware プロジェクトの `hardware_config.md` に相当し、環境の確定は本ファイルを正本とする。

## ホスト環境

| 要素 | 仕様 | 備考 |
|---|---|---|
| OS | Windows 10/11（64 bit）を第一対象とし、macOS/Linux でもビルド可能な構成とする | Tauri 2 の対応プラットフォームに従う |
| BLE | Bluetooth 4.0 以上の BLE セントラル機能を持つアダプタ | `btleplug` が OS 標準 BLE API（Windows: WinRT）を利用する |
| Node.js | v20 以降 | フロントエンドビルド用 |
| Rust | stable（edition 2021） | バックエンドビルド用 |
| 主要 npm 依存 | vite 8.x, react 19, react-router 7, `@tauri-apps/api` 2.x, zustand 5, tailwindcss 4 | `package.json` が正本 |
| 主要 cargo 依存 | tauri 2, btleplug 0.11, tokio, serde, thiserror, tauri-specta / specta / specta-typescript | `src-tauri/Cargo.toml` が正本。specta 系は `src/lib/bindings.ts` 生成に使用 |
| 開発コンテナ | リポジトリ `.devcontainer` | Node.js / Rust / Tauri Linux 依存を同梱。コンテナ内ビルドは Linux 向け成果物となり、BLE 実機通信は Linux ホストの BlueZ/D-Bus 経由に限る（Windows + Docker Desktop では Bluetooth を渡せない） |

## モバイル実行環境（iOS）

| 要素 | 仕様 | 備考 |
|---|---|---|
| OS | iOS 15 以降 | `gen/apple/project.yml` の deploymentTarget が正本 |
| ビルド | Xcode で `src-tauri/gen/apple/*.xcodeproj` をビルド、または `npm run tauri ios build` | 生成手順は `tools/build_ios.sh` |
| BLE | CoreBluetooth（`btleplug`） | `NSBluetoothAlwaysUsageDescription` を `src-tauri/Info.plist` で付与。iOS シミュレータでは Bluetooth が動作しないため BLE 確認は実機のみ |
| 署名 | 実機ビルドには Apple Development Team ID が必要 | `APPLE_DEVELOPMENT_TEAM` 環境変数または `bundle > iOS > developmentTeam` |

## 対向機器

| 要素 | 機器 | 概要 |
|---|---|---|
| 無線変換器 | XIAO ESP32-C6（`uart_wireless_converter_esp32`） | BLE ペリフェラル。デバイス名 `uwc-esp32c6` で advertising する |
| ロボット制御 MCU | NUCLEO-G474RE（`ball_balancing_omni3_stm32`） | USART1（PC4 TX / PC5 RX、115200 8N1）で ESP32 と接続。プロトコルの最終処理者 |

## BLE インターフェース仕様

- GATT サービスは Nordic UART Service 相当（透過 UART）。Service UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`。
- RX 特性（アプリ→ロボット方向、Write / Write Without Response）: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`。
- TX 特性（ロボット→アプリ方向、Notify）: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`。
- リンクはバイトストリームであり、メッセージ境界は保持されない。フレーミングは `robot_link_protocol.md` のプロトコル層の責務とする。
- 接続パラメータはペリフェラル側が接続間隔 7.5–30 ms・2M PHY・MTU 拡大を要求する（`uart_wireless_converter_esp32` 側 NFR-01）。アプリ側は OS の既定動作に従う。
