# ball_balancing_omni3_stm32

`matlab_ws/ball_balancing_omni3` の rev2 制御器・推定器を NUCLEO-G474RE へ移植した STM32 ファームウェア。Zephyr 4.3 上で制限付き C++（例外・RTTI・動的確保・STL 禁止）で実装する。

## 構成

- `spec/` — 仕様書（要件・基本設計・詳細設計）
- `src/` — 実装。`spec/detail_design/` と同じ層構造（`common → peripheral → device → feature → manager`）
- `boards/nucleo_g474re.overlay` + `dts/bindings/` — ピン・タイマ・I2C・PWM・QDEC・IMU の devicetree 宣言
- `prj.conf` — Kconfig（shell/logging/CPP/動的割込/ヒープ無効）
- `third_party/etl` — Embedded Template Library（git submodule、タグ 20.49.0、`ETL_NO_STL`）
- `tests/host/` — feature 層のホスト単体テスト（Zephyr 非依存、ホスト clang++）
- `tests/onboard/` — 実機検証用テストファームウェア（別ビルド、shell `test` コマンド駆動）
- `cmake/` — lint/format スクリプト
- `.vscode/` — タスク・デバッグ・clangd 設定

## 必要なツール

| ツール | 用途 | 導入 |
|---|---|---|
| Zephyr ワークスペース | Zephyr 4.3 + hal_stm32 + hal_st + cmsis_6 | `west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.3.0 C:\zephyr_ws` → `cd C:\zephyr_ws` → `west update` |
| Zephyr SDK 0.17.4 | クロスコンパイル（arm-zephyr-eabi-gcc 12.2） | `C:\tools\zephyr-sdk-0.17.4` へ展開 |
| Python 3.13 + west | Zephyr ビルドシステム | `winget install Python.Python.3.13` → `pip install west` |
| CMake + Ninja | ビルド | `winget install Kitware.CMake` / `Ninja-build.Ninja` |
| LLVM（clang-tidy/clang-format/clang++） | 静的解析・整形・ホストテスト | `winget install LLVM.LLVM` |
| OpenOCD | 書き込み・デバッグ | `winget install xpack-dev-tools.openocd-xpack` |

`ZEPHYR_TOOLCHAIN_VARIANT=zephyr` と `ZEPHYR_SDK_INSTALL_DIR=<SDK パス>` は `west`/`cmake` 実行時に必要（VSCode タスクには同梱済み）。

## ビルド・検証（ターミナル / AI エージェント共通）

```bash
# 環境変数（bash の場合）
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=C:/tools/zephyr-sdk-0.17.4

# ファームウェア（build-zephyr/zephyr/zephyr.elf/.hex/.bin）
cmake -S . -B build-zephyr -G Ninja -DBOARD=nucleo_g474re -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-zephyr

# 静的解析・整形
cmake -DCLANG_TIDY=clang-tidy -DCOMPILE_COMMANDS=build-zephyr/compile_commands.json -DSRC_ROOT=ball_balancing_omni3_stm32/src -P cmake/run_clang_tidy.cmake
cmake -DCLANG_FORMAT=clang-format -DSRC_ROOT=$PWD/src -DTEST_ROOT=$PWD/tests/host/src -P cmake/run_clang_format.cmake

# ホスト単体テスト（feature 層・ハード不要）
cmake -S tests/host -B build-tests -G Ninja -DCMAKE_CXX_COMPILER=clang++
cmake --build build-tests && ./build-tests/bb_feature_tests.exe

# オンボードテスト（実機検証用の別ファームウェア）
cmake -S tests/onboard -B tests/onboard/build -G Ninja -DBOARD=nucleo_g474re -DCMAKE_BUILD_TYPE=Debug
cmake --build tests/onboard/build

# 書き込み（OpenOCD + ST-LINK）
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build-zephyr/zephyr/zephyr.elf verify reset exit"
```

VSCode では `.vscode/tasks.json` の同名タスクが同等コマンドを実行する。デバッグは `Debug STM32 (OpenOCD)` 構成（cortex-debug + OpenOCD + ST-LINK）。

## 言語規約（制限付き C++）

- C++23。`-fno-exceptions -fno-rtti -fno-threadsafe-statics -nostdinc++`（STL 不使用・例外・RTTI・動的確保禁止）。
- STL 相当は ETL を使う（`etl::array`/`etl::clamp` 等）。
- 識別子は `snake_case`、定数のみ `UPPER_SNAKE` を `constexpr` で定義する。
- ヒープは使わない（`CONFIG_HEAP_MEM_POOL_SIZE=0`）。

## UART 操作

USART2（115200 8N1）に Zephyr shell が出る。

- `bb status` — モード・オイラー角・輪速・トルク・オーバーラン・最大実行時間を表示
- `bb telem on|off` — 200 ms 周期の CSV テレメトリを切替える（既定 off）

オンボードテストファームウェア（`tests/onboard`）では `test` コマンドで各検証
テストを実行する。テスト一覧・手順・合否基準は `spec/test/test_plan.md` を参照。

## 主要パラメータ

`src/common/control_params/control_params.hpp` に `ballbotRev1Parameters()` のスナップショットを集約している。未確定の値（エンコーダ生カウント/回転、IMU 軸マッピング）は `spec/requirements/hardware_config.md` の未確定事項を参照。

## 層構造

`common → peripheral → device → feature → manager`（上位→下位のみ依存、継承禁止）。詳細は `spec/basic_design/basic_design.md`。
