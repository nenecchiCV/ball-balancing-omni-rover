# NFR-03 静的解析

## 内容

clang-tidy と clang-format を常時クリーンに保つ。

## 仕様

- `.clang-tidy`・`.clang-format` は `ball_balancing_omni3_stm32` と同一ポリシーを共用する。
- `cmake/run_clang_tidy.cmake`・`cmake/run_clang_format.cmake` を同等手順で実行し、報告が残らない状態を維持する。Zephyr フレームワーク由来・マクロ展開由来の警告は `NOLINT` で局所抑制する。

## 測定方法

lint タスクの出力が 0 警告であること。
