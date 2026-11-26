# NFR-04 ビルド・検証の自動実行

## 内容

ビルド・型検査・静的解析をターミナルから再現可能にし、AI エージェントが検証を自動実行できる状態に保つ。

## 仕様

- `npm run dev`（開発用 devUrl + `tauri dev`）、`npm run build`（型検査 + Vite 静的ビルド `dist/`）、`npm run tauri build`（アプリバンドル）を定義する。
- `npx tsc --noEmit` と `cargo check`（src-tauri）が通ることを変更後の最低検証とする。
- VSCode タスク（`.vscode/tasks.json`）に同等コマンドを定義し、GUI なしでも同じコマンドを実行できる形にする。

## 測定方法

上記コマンドの終了コードで判定する。
