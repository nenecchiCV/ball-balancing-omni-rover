# 基本設計書

`robot_controller_tauri` のソフトウェア構成・実行モデル・実装方針を定義する。本アプリはデスクトップ上で BLE セントラルとして動作し、`uart_wireless_converter_esp32` 経由で玉乗りオムニローバー（`ball_balancing_omni3_stm32`）を操作・監視する。

## 実行基盤

- **フレームワーク**: Tauri 2。フロントエンドは Next.js 15（App Router、`output: 'export'` の静的エクスポート）を `src-tauri` 内蔵 WebView で配信する。バックエンドは Rust（edition 2021）。
- **BLE**: `btleplug` クレートが OS 標準 BLE API（Windows では WinRT）を利用する。WebView 内 Web Bluetooth には依存しない。
- **非同期**: バックエンドは `tokio` ランタイム（Tauri 付属の `tauri::async_runtime`）で動作する。BLE 受信は通知ストリームを非同期タスクで消費する。
- **プロセス境界**: フロントエンド ↔ バックエンドは Tauri の IPC。リクエスト/レスポンスは `invoke`/`#[tauri::command]`、購読型は `tauri::ipc::Channel`（`watch_*` command）とする。
- **型共有**: `tauri-specta` + `specta-typescript` が command シグネチャから `src/lib/bindings.ts` を自動生成する。IPC 往復型は Rust 側定義が正本であり、serde `camelCase` で JS 慣習に揃える。

## ソフト構成

Tauri は WebView（Next.js）と Rust コアの 2 プロセス構成であり、firmware のような単一の層スタックは適用しない。依存の一方向性は所有権ベースのルールとして両側で個別に定義する。

### バックエンド: IPC シェル + Tauri 非依存ドメイン

```
lib.rs ─→ commands.rs ─→ robot/ ─→ protocol/
               │        └→ ble/       (leaf)
               └→ state.rs / error.rs (leaf)
```

- **規則（依存制約）**: `tauri::` API を import できるのは `lib.rs`・`commands.rs`・`state.rs` のみ。`protocol/`・`ble/`・`robot/` は Tauri 非依存の純 Rust モジュールとし、`cargo test` で GUI ランタイムなしに検証できる。
- `commands.rs` は UI 引数の検証と `robot::Session` 呼出しのみを担う薄い IPC シェルとし、ドメインロジックを持たない。
- エラーは `error::AppError`（`thiserror` + `Serialize` + `specta::Type`）で返し、`Result<T, String>` は用いない。
- `tauri-specta` の `collect_commands!` で command 一覧を集約し、`lib.rs` の `specta.invoke_handler()` に渡す。debug ビルド起動時に `bindings.ts` を再生成する。

### フロントエンド: feature slice

- `src/app/` はルート定義のみ（`page.tsx` は feature コンポーネントを返す薄い層）。
- 画面実体は `src/features/<domain>/` に垂直分割する（connection / rc / position / odometry / logs / home）。
- **規則（依存制約）**: `@tauri-apps/api` を直接 import できるのは `src/lib/` のみ。feature 同士の直接 import は禁止し、共有は `src/components/`・`src/stores/`・`src/lib/` 経由とする。
- 状態管理は zustand（`src/stores/`）。高頻度テレメトリは store 側で分離し、Context による全画面再レンダを避ける。
- スタイルは Tailwind CSS v4（`globals.css` の `@theme` トークン + `@layer components` 共通クラス）。

## データフロー

```
[features] --invoke--> [commands.rs] --呼出--> [robot::Session] --write--> [ble::nus]
[features] <--Channel-- [watch_*] <--broadcast-- [Session bus] <--notify-- [nus]
                                              |
                                              +-> pending request (seq 対応付け, ACK/PONG/STATUS)
                                              +-> event fanout (EVT_POSE/EVT_LOG/EVT_STATUS)
```

- **指令系**: UI が `commands.robotSetVelocity(...)` → command が `Session` 経由でフレーム送出。ACK を待たず完了とする高頻度経路（`SET_VELOCITY`）と、ACK を待つ通常経路（残りリクエスト、タイムアウト 1 s）を使い分ける。`CALIBRATE_IMU` は受理 ACK の後に `CALIB_RESULT` が非同期で来る 2 段応答として扱う。
- **購読系**: ロボットからの `EVT_*` は通知ストリーム→フレームパーサ→ディスパッチャでデコードされ、`tokio::sync::broadcast` のイベントバスへ発行される。`watch_*` command が broadcast → `Channel` の橋渡しを行い、`app-shell` が起動時に全チャネルを一度だけ購読して zustand store へ配る。
- **ロボット向け購読**（`SUBSCRIBE` フレーム送出）は UI ストリームとは別系統とし、`robot_subscribe`/`robot_unsubscribe` command で制御する。

## スレッド・タスク管理

| 実行主体 | 役割 | 備考 |
|---|---|---|
| tokio: ディスパッチャタスク | NUS 通知 → フレームパーサ → 保留リクエスト完了 / broadcast 発行 | 接続確立時に spawn、切断で終了 |
| tokio: watch 転送タスク | broadcast → `Channel::send` | `watch_*` 呼出しごとに spawn、Channel 破棄で終了 |
| tokio: 送信パス | command → `Session::send` → NUS RX Write | Write Without Response。直列化は `Mutex` で担保 |
| WebView UI スレッド | ページ描画・操作・`invoke` | ジョイスティックの 20 Hz 送信は `setInterval` 駆動 |

## メモリ・リソース管理

- ログ表示・軌跡点は zustand store 側で上限付きリング（既定 5000 件 / 20000 点）として保持し、超過分を古い順に破棄する。
- BLE リソース（`Peripheral` ハンドル、通知ストリーム）は接続スコープに閉じ込め、切断時に解放する。
- broadcast 受信が遅れた場合（`Lagged`）は遅延分を捨てて最新へ追従する（テレメトリの鮮度優先）。

## 実装方針

- **プロトコル**: `spec/basic_design/robot_link_protocol.md` が正本。フレーム符号化・メッセージ定義は `src-tauri/src/protocol/` に集約し、上位からは型付き API で隠蔽する。
- **ページ分割**: 機能要求ごとにページを分ける（FR-01→`/connect`、FR-02→`/rc`、FR-03→`/position`、FR-04→`/odometry`、FR-05→`/logs`、FR-08→`/test`）。ホーム `/` は接続状態・動作状態サマリとナビゲーションに加え、FR-07 の開始前準備パネル（キャリブレーション・位置設定・動作開始/停止）を持つ。動作状態は app-shell のバッジでも常時表示する。
- **エラーポリシー**: command は `Result<T, AppError>` で返し、UI はエラーをステータス表示とログビュー（`app` ソース）へ記録する。パニックは起こさない。
- **クランプ**: 指令値は UI 入力時と `Session` 送信直前で二重にクランプする（NFR-02）。
- **命名**: Rust は snake_case、TS は camelCase（IPC 型は serde で変換）。フロントのファイル名は kebab-case。

## 安全性

- 緊急停止は `STOP` リクエストを全画面共通ボタンから即時送信する。ロボット側の最終安全機構（E-Stop、指令ウォッチドッグ 500 ms、リンク断絶フェイルセーフ）は `ball_balancing_omni3_stm32` が保持し、本アプリは通知と指令停止を担当する。
- 切断検出は `Session` が `link` broadcast に `Disconnected` を発行し、`watch_link_state` 経由で UI の接続バッジを即時更新する。
- `watch_*` を含む command 全般は Tauri GUI ランタイム（wry）への依存を引き込むため、Windows の `cargo test` ではアプリマニフェスト（comctl32 v6）のないテストバイナリがロード失敗する。ドメインの単体テストは Tauri 非依存モジュールに閉じて書き、IPC シェルを直接テストしない。
