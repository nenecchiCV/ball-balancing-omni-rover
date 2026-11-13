# NFR-03 言語・アーキテクチャ規約

## 内容

制限付き C++23 を採用し、層構造・継承禁止・命名規約を守る。

## 仕様

- 言語標準は C++23（Zephyr SDK 0.17.4 付属 arm-zephyr-eabi-gcc 12.2、`CONFIG_STD_CPP23`）。`-fno-exceptions -fno-rtti -fno-threadsafe-statics`。
- **STL 禁止**（`-nostdinc++`）。STL 相当が必要な箇所は ETL（`third_party/etl`、git submodule、`ETL_NO_STL`）を使う。
- **動的確保禁止**: `new`/`delete`/`malloc` は使わない。`CONFIG_HEAP_MEM_POOL_SIZE=0` でカーネルヒープも無効化する。
- **静的初期化制限**: 実行時コンストラクタを持つグローバル/静的オブジェクトを作らない。定数は `constexpr`、コンポーネントは `main` 内ローカル static とし、コンストラクタは配線のみ行う。
- **命名規則**: 識別子は原則すべて `snake_case`（クラス・関数・変数・ファイル名）。定数のみ `UPPER_SNAKE_CASE` とし `constexpr` で定義する（定数目的のマクロは使わない）。Zephyr API・マクロ識別子（`GPIO_*`/`LL_TIM_*`/`DT_*` 等）はそのまま使用してよい。
- 浮動小数点は `float` 統一。`double` 定数・演算を混在させない。
- 層構造: `common < peripheral < device < feature < manager`。依存は上位→下位のみ。同一層内参照は循環しない範囲で許可。
- クラス継承は原則禁止。コンポジション・コンセプトで代替する。
- 1 コンポーネント = 1 クラス = 1 ディレクトリ（同名 hpp/cpp）。`src/main.cpp` は `src/` 直下。
- ハードウェア構成（ピン・タイマ・I2C・PWM・QDEC・IMU）は devicetree overlay で宣言し、コードは `DT_*` マクロ経由でのみ参照する。
- RTOS は Zephyr。ペリフェラル制御は Zephyr ドライバ API または `st,stm32-qdec` 等の DT ドライバ＋ `LL_TIM` レジスタ読取を `peripheral` 層に閉じ込める。

## 測定方法

コードレビュー + clang-tidy（hicpp/cppcoreguidelines 系、命名規則は `.clang-tidy` の `readability-identifier-naming`）で確認する。
