# NFR-02 言語・アーキテクチャ規約

## 内容

`ball_balancing_omni3_stm32` と同一の制限付き C++・層構造・命名規約を採用する。

## 仕様

- 言語標準は C++23（Zephyr SDK 0.17.4 付属 riscv64-zephyr-elf-gcc、`CONFIG_STD_CPP23`）。`-fno-exceptions -fno-rtti -fno-threadsafe-statics`。
- **STL 禁止**（`-nostdinc++`）。STL 相当が必要な箇所は ETL（`third_party/etl`、git submodule、`ETL_NO_STL`）を使う。
- **アプリ側動的確保禁止**: `new`/`delete`/`malloc` は使わない。無線スタック内部の確保は NFR-04 に従う。
- **静的初期化制限**: `CONFIG_STATIC_INIT_GNU=n` により実行時コンストラクタは走らない。static メンバ・グローバルはゼロ/定数初期化のみで成り立たせ、非ゼロの値は `init()` で設定する。ただし Zephyr の BT GATT サービス登録等、フレームワーク側の初期化子はこの限りでない。
- **命名規則**: 識別子は原則すべて `snake_case`。定数のみ `UPPER_SNAKE_CASE` とし `constexpr` で定義する。Zephyr API・マクロ識別子（`BT_*`/`DT_*`/`CONFIG_*` 等）はそのまま使用してよい。
- 層構造: `common < peripheral < device < feature < manager`。依存は上位→下位のみ。同一層内参照は循環しない範囲で許可。
- クラス継承は原則禁止。差し替えはコンポジション・コンセプト、または FR-04 のビルド時ソース選択で実現する。
- 1 コンポーネント = 1 クラス = 1 ディレクトリ（同名 hpp/cpp）。`src/main.cpp` は `src/` 直下。
- システムに 1 つ常駐するコンポーネントは static class（全メンバ `static`、ctor `= delete`、`init(config)` + `start()`/`stop()`）。
- コールバック（ISR・BT/shell ハンドラ・タスクフック）は `*_cb` サフィックスとする。
- ハードウェア構成は devicetree で宣言し、コードは `DT_*` マクロ経由でのみ参照する。ペリフェラル制御は Zephyr ドライバ API を `peripheral` 層に閉じ込める。

## 測定方法

コードレビュー + clang-tidy（`.clang-tidy` の `readability-identifier-naming` 等）で確認する。
