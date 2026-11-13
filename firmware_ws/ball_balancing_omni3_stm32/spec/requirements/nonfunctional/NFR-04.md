# NFR-04 静的解析の常時クリーン

## 内容

`src/` 配下は clang-tidy の指摘を常時 0 に保つ。

## 仕様

- `.clang-tidy` に適用チェックを列挙する。対象: bugprone、cert、hicpp、performance、portability、readability、misc 系。
- ビルド時に compile_commands.json を生成し、`lint` ターゲットで `clang-tidy -p` を実行可能にする。
- clang-format（`.clang-format`、LLVM ベース）で整形を維持する。

## 測定方法

`lint` ターゲット実行結果が 0 件であること。警告の無効化は行単位 `// NOLINT` に理由付きで限定する。
