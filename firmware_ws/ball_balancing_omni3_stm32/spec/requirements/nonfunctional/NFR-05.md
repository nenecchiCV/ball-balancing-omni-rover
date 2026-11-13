# NFR-05 ビルド・デバッグの自動実行性

## 内容

ビルド・書き込み・デバッグ・静的解析を VSCode タスクとして定義し、同等コマンドをターミナル（AI エージェント）から実行可能にする。

## 仕様

- `.vscode/tasks.json` に `configure`、`build`、`lint`、`format`、`host-test`、`flash`、`debug-attach` を定義する。
- `.vscode/launch.json` に OpenOCD + cortex-debug ベースのデバッグ構成を定義する。
- 各タスクは GUI 非依存のコマンドであり、ターミナルからの再実行で同一結果が得られる。
- ファームウェアは Zephyr ワークスペース（`C:\zephyr_ws`、Zephyr 4.3 + hal_stm32/hal_st/cmsis_6）と Zephyr SDK 0.17.4（`C:\tools\zephyr-sdk-0.17.4`）でビルドする。導入手順は README を正本とする。
- feature 層のホスト単体テストは `tests/host` でホスト clang++ にてビルド・実行する（Zephyr 非依存）。

## 測定方法

タスクをターミナルから手動実行して成功すること。
