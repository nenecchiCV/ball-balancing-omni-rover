# コンポーネント一覧

バックエンドは 1 コンポーネント = 1 モジュール（Rust）、フロントエンドは 1 画面要素 = 1 ファイル（TS/TSX）とする。構成モデルは `spec/basic_design/basic_design.md` の「IPC シェル + Tauri 非依存ドメイン（Rust）／feature slice（フロント）」に従う。

## 第 1 部: バックエンド（`src-tauri/src/`）

依存制約: `tauri::` を import できるのは `lib.rs`・`commands.rs`・`state.rs` のみ。それ以外は Tauri 非依存とする。

### ドメインモジュール（Tauri 非依存）

| コンポーネント | ファイル | 責務 |
|---|---|---|
| frame | `protocol/frame.rs` | フレーム符号化・ストリーム復号（SYNC 再同期・CRC16 検査） |
| crc16 | `protocol/crc16.rs` | CRC16-CCITT-FALSE 計算 |
| message | `protocol/message.rs` | メッセージ TYPE 定数・ペイロード型・エンコード/デコード |
| nus | `ble/nus.rs` | `btleplug` による NUS クライアント。スキャン・接続・RX Write・TX 通知購読・切断検出 |
| session | `robot/session.rs` | 接続ライフサイクル、フレーム送出、ディスパッチャ（seq 対応付け・イベント振分け）、保留リクエストのタイムアウト、broadcast イベントバス |
| telemetry | `robot/telemetry.rs` | `PoseTelemetry`/`StatusTelemetry` 型と broadcast バスの定義 |
| log | `robot/log.rs` | `LogMessage`/`LogSource` 型定義 |

### IPC シェル（Tauri 依存）

| コンポーネント | ファイル | 責務 |
|---|---|---|
| commands | `commands.rs` | `#[tauri::command]` + `#[specta::specta]` 群。UI 引数の検証・`Session` 呼出し、`watch_*` の broadcast→Channel 橋渡し |
| error | `error.rs` | `AppError`（thiserror + Serialize + specta::Type）。IPC 境界の統一エラー型 |
| state | `state.rs` | `AppState`（`Arc<Session>` 保持）定義。`tauri::Builder::manage` 登録用 |
| app_entry | `lib.rs`, `main.rs` | specta Builder・bindings.ts エクスポート・command 登録・`manage`・`run` |

## 第 2 部: フロントエンド（`src/`）

依存制約: `@tauri-apps/api` を直接 import できるのは `lib/` のみ。feature 間の直接 import は禁止。

### lib（IPC 境界・共有）

| コンポーネント | ファイル | 責務 |
|---|---|---|
| bindings | `src/lib/bindings.ts` | ★生成物（tauri-specta）。command 呼出し `commands.*` と IPC 型の正本。手編集禁止 |
| ipc | `src/lib/ipc.ts` | `invoke`/`Channel` の唯一の入口。`commands` の既定値ラッパ + `watch_*` 購読ヘルパ |
| types | `src/lib/types.ts` | bindings 型の re-export とプロトコル定数（`CONTROL_MODE`） |
| limits | `src/lib/limits.ts` | 速度・ヨーレート上限など UI クランプ用定数（`control_params` と同期） |

### stores（zustand 状態管理）

| コンポーネント | ファイル | 責務 |
|---|---|---|
| connection | `src/stores/connection.ts` | `linkState`・スキャン結果デバイス一覧 |
| telemetry | `src/stores/telemetry.ts` | `pose`・`status`・軌跡（20000 点リング） |
| logs | `src/stores/logs.ts` | ログエントリ（5000 件リング、robot/app ソース混在） |

### components（共通部品）

| コンポーネント | ファイル | 責務 |
|---|---|---|
| app-shell | `src/components/app-shell.tsx` | 全画面共通シェル（ナビ、接続バッジ、動作状態バッジ、緊急停止、`watch_*` 一元起動） |

### features（画面 = ルート）

| コンポーネント | ファイル | 責務 |
|---|---|---|
| home-page | `features/home/home-page.tsx` + `app/page.tsx` | 接続状態・動作状態サマリ・機能ナビ・開始前準備パネル（FR-07） |
| connection-page | `features/connection/connection-page.tsx` + `app/connect/page.tsx` | スキャン・接続・切断・PING 照合（FR-01） |
| rc-page | `features/rc/rc-page.tsx` + `app/rc/page.tsx` | 2 本ジョイスティックの速度指令（FR-02） |
| joystick | `features/rc/joystick.tsx` | ポインタ操作型バーチャルジョイスティック（-1.0〜1.0 正規化出力） |
| position-page | `features/position/position-page.tsx` + `app/position/page.tsx` | 目標位置・速度入力とマップ表示（FR-03） |
| odometry-page | `features/odometry/odometry-page.tsx` + `app/odometry/page.tsx` | 軌跡・数値表示と原点リセット（FR-04） |
| odometry-map | `features/odometry/odometry-map.tsx` | 俯瞰マップ canvas（位置・向き・軌跡・目標点描画） |
| logs-page | `features/logs/logs-page.tsx` + `app/logs/page.tsx` | ログリスト・フィルタ・一時停止・クリア（FR-05） |
| test-page | `features/test/test-page.tsx` + `app/test/page.tsx` | プロトコル疎通・テストコマンド送受信（FR-08） |

## エントリ

- `src-tauri/src/main.rs`: `app_lib::run()` の呼出のみ（Tauri 2 構成）。
- `src/app/layout.tsx`: 全ページ共通レイアウト（`app-shell` 適用）。
