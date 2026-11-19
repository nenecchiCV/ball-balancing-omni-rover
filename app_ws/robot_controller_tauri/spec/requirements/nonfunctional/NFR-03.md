# NFR-03 言語・アーキテクチャ規約

## 内容

プロジェクトの言語・構造規約を守り、保守可能な構成を維持する。

## 仕様

- バックエンド（`src-tauri/`）は Rust edition 2021、フロントエンド（`src/`）は TypeScript + Next.js（App Router、静的エクスポート `output: 'export'`）とする。
- 層構造は `spec/detail_design/main.md` の対応表に従い、依存は上位→下位のみとする。
- 命名は Rust/TS エコシステムの標準に従う（Rust: `snake_case` 関数・`CamelCase` 型、TS: `camelCase` 関数・`PascalCase` コンポーネント）。プロトコル上のメッセージ名・フィールド名は `robot_link_protocol.md` の記述に一致させる。
- BLE 操作・プロトコル処理は Rust バックエンドに閉じ込め、フロントエンドは Tauri command（リクエスト/レスポンス）と event（購読）のみを介してアクセスする。
- 状態はバックエンドの `Mutex` 保護された共有状態に集約し、UI 表示用データはイベントでフロントエンドへ流す。

## 根拠

`firmware_ws` プロジェクトの層構造・責務分離の方針に倣い、無線変換器と同様に「トランスポートを下位層へ閉じ込める」構造を採る。
