# アプリ開発の指示

`app_ws/` 配下のデスクトップアプリ（Vite + React × Tauri）を作成・編集する作業に適用する。firmware プロジェクトとは別の構成モデルをとる（`firmware.md` の層ルールは適用しない）。

## 開発方針

- 仕様駆動開発を行う。要求 → 基本設計 → 詳細設計 → 実装 → 検証の順で整合を保ち、実装だけを先に変更しない。
- 変更の起点は要求とする。要求 ID を採番し、基本設計・詳細設計・実装へトレースできる状態を維持する。
- 設計書と実装は常に同期させる。実装を変える場合は対応する詳細設計を同じ変更内で更新し、構造を変える場合はコンポーネント一覧まで遡って更新する。

## 構成モデル

Tauri の 2 プロセス構成（WebView フロント + Rust コア）に従い、firmware の層モデルは写像しない。

- **バックエンド（`src-tauri/src/`）**: IPC シェル + Tauri 非依存ドメイン。`tauri::` を import できるのは `lib.rs`・`commands.rs`・`state.rs` のみ。`protocol/`・`ble/`・`robot/` は Tauri 非依存とし、`cargo test` で検証可能に保つ。
- **フロントエンド（`src/`）**: feature slice。`app/` はルート定義のみ、画面実体は `features/<domain>/`、共通部品は `components/`、状態は `stores/`（zustand）。`@tauri-apps/api` を直接 import できるのは `lib/` のみ。feature 間の直接 import は禁止。
- **IPC 型**: `tauri-specta` 生成の `src/lib/bindings.ts` が正本。IPC 往復型は Rust 側に `serde(rename_all = "camelCase")` + `specta::Type` を付け、u64 等の BigInt 相当型は使わない。`bindings.ts` は手編集しない（`npm run tauri dev` 起動時に再生成）。
- **イベント**: UI への配信は `tauri::ipc::Channel`（`watch_*` command）＋バックエンド `tokio::sync::broadcast`。`emit`/`listen` のグローバルイベントは使わない。
- **エラー**: command は `Result<T, AppError>`（`error.rs`、Serialize 済 tagged union）で返す。`Result<T, String>` は使わない。

## 仕様書の構成

プロジェクト直下の `spec/` に Markdown で仕様書を置く（firmware と同形式）。

- `spec/requirements/requirements.md` + `functional/FR-xx.md` + `nonfunctional/NFR-xx.md` + `environment.md`
- `spec/basic_design/basic_design.md` + `component_catalog.md`（Rust 側・フロント側の 2 部構成）+ `robot_link_protocol.md`（アプリ↔ロボット API の正本）
- `spec/detail_design/main.md`（索引）+ `spec/detail_design/<領域>/<component>.md`。領域はソース構造に対応（`protocol/`・`ble/`・`robot/`・`ipc/`・`frontend/`・`ui/`）。
- `spec/test/test_plan.md`

## コード規約

- Rust: edition 2021、snake_case。`async` は `tokio` プリミティブで同期する。
- TypeScript/React: camelCase（IPC 型は serde で変換済）。ファイル名は kebab-case、コンポーネント export は PascalCase。
- スタイル: Tailwind CSS v4。`globals.css` の `@theme` トークンと `@layer components` 共通クラスを使い、ページ固有の素 CSS は増やさない。
- 状態管理: zustand。React Context は使わない。

## 検証

- フロント: `npx tsc --noEmit`、`npm run lint`、`npm run build`（`dist/` への静的ビルドまで）。
- バックエンド: `cargo check`、`cargo test`（`src-tauri/`）。単体テストは Tauri 非依存モジュールに限定する（IPC シェルを参照するテストは Windows でマニフェスト非埋込のテストバイナリがロード失敗する）。
- 実行確認: `npm run tauri dev`（GUI 起動）。パッケージングは `npm run tauri build`、Windows 成果物は Windows ホストでのみ生成する。
