# reference_slides（参照用スライド）

本フォルダは、プロジェクトの設計内容を横断的にまとめた**参照用スライド**を置く。仕様の正本ではない。数値・仕様の正本は以下を参照すること。

| 領域 | 正本 |
|---|---|
| ハード（機構・電装・制御理論） | `specs/hardware_design/` |
| シミュレーションモデル（プラント・制御・推定） | `matlab_ws/ball_balancing_omni3/` |
| 実機ソフト | `firmware_ws/ball_balancing_omni3_stm32/spec/` |
| 無線変換器 | `firmware_ws/uart_wireless_converter_esp32/spec/` |
| 操作用アプリ・リンクプロトコル | `app_ws/robot_controller_tauri/spec/` |

## 構成

- `slides.md` — Slidev スライド本体。章構成は「ハード概要 → シミュレーションモデル仕様（プラント・制御・推定設計）→ 実機ソフト仕様 → 操作用アプリ」

## 実行

```bash
npm install
npm run dev     # http://localhost:3030 でプレビュー
npm run build   # dist/ へ静的ビルド
npm run export  # PDF エクスポート（playwright-chromium が必要）
```
