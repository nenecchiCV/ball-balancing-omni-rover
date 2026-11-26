# robot_controller_tauri

玉乗りオムニローバー（`ball_balancing_omni3_stm32`）を BLE 経由で操作・監視するデスクトップアプリ。Vite + React（フロントエンド）× Tauri 2（Rust バックエンド）で構成し、`uart_wireless_converter_esp32` の NUS 相当 GATT 透過リンクを介してロボットと通信する。

```
[robot_controller_tauri] <--BLE NUS--> [XIAO ESP32-C6] <--UART 115200--> [NUCLEO-G474RE USART1]
```

## 機能

- **Connect**: BLE スキャン・接続・切断・プロトコルバージョン照合（FR-01）
- **RC**: 2 本のバーチャルジョイスティックで速度指令（左=並進 vx/vy、右=回転 yaw_rate、20 Hz 送出）（FR-02）
- **Position**: 目標位置 (x, y) と速度上限を入力して位置制御（FR-03）
- **Odometry**: 起動点原点のワールド座標で位置・軌跡を表示、原点リセット（FR-04）
- **Logs**: ロボットのログメッセージとアプリイベントを時系列表示（FR-05）
- **全画面共通**: 緊急停止ボタン（`STOP` 即時送信）と接続バッジ（FR-06）

## 構成

- `spec/` — 仕様書（要件・基本設計・詳細設計・テスト計画）。`spec/basic_design/robot_link_protocol.md` がアプリ↔ロボット間 API の正本
- `src/` — React SPA フロントエンド（Vite ビルド、react-router、feature slice 構成）
  - `app/` — ルート定義のみ（`app.tsx` の `<Routes>` テーブル）
  - `features/<domain>/` — 画面実体（connection / rc / position / odometry / logs / home）
  - `components/` — 共通部品（`app-shell`）
  - `stores/` — zustand 状態管理（connection / telemetry / logs）
  - `lib/` — IPC 境界。`bindings.ts` は tauri-specta 生成物、`ipc.ts` が `@tauri-apps/api` の唯一の入口
- `src-tauri/` — Rust バックエンド（IPC シェル + Tauri 非依存ドメイン）
  - `protocol/` `ble/` `robot/` — Tauri 非依存モジュール（`cargo test` 対象）
  - `commands.rs` `error.rs` `state.rs` `lib.rs` — IPC シェル（tauri API を使うのはこの範囲のみ）

## 必要なツール

| ツール | 用途 |
|---|---|
| Node.js v20+ | フロントエンドビルド |
| Rust stable + Tauri 前提（VS Build Tools / WebView2） | バックエンドビルド |
| Bluetooth 4.0+ アダプタ | BLE セントラル |

リポジトリ `.devcontainer` にも同等のビルド環境（Node.js / Rust / Tauri Linux 依存）を同梱しており、コンテナ内でビルド・検証まで実行できる。コンテナ内ビルドは Linux 向け成果物となり、BLE 実機通信は Linux ホストの BlueZ/D-Bus 経由に限る。制約の詳細は `.devcontainer/README.md` を参照。

## ビルド・実行

```bash
npm install
npm run tauri dev      # 開発実行（Vite devUrl + Tauri。起動時に src/lib/bindings.ts を再生成）
npm run build          # 型検査 + Vite ビルド (dist/)
npm run lint           # ESLint
npm run tauri build    # 本番ビルド + バンドル
npx tsc --noEmit       # 型検査（フロントエンド）
cargo check            # 型検査（src-tauri/ で実行）
cargo test             # プロトコル/ドメイン単体テスト（src-tauri/ で実行）
```

## プロトコル

フレーム形式・メッセージ一覧（`SET_VELOCITY` / `SET_POSE_TARGET` / `SUBSCRIBE` / `EVT_POSE` / `EVT_LOG` 等）は `spec/basic_design/robot_link_protocol.md` を参照。ロボット側の対応実装は `ball_balancing_omni3_stm32` の FR-09〜FR-11 にトレースする。

## 仕様駆動開発

`spec/requirements`（要求・環境）→ `spec/basic_design`（基本設計・プロトコル・コンポーネント一覧）→ `spec/detail_design`（各コンポーネント詳細）→ `src` / `src-tauri`（実装）の順で同期して変更する。方針はリポジトリの `.agents/instructions/` に準じる。
