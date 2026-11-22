# uart_wireless_converter_esp32

NUCLEO-G474RE（`ball_balancing_omni3_stm32`）の UART とリモートコントローラ/ログ取得アプリを無線接続する UART-無線変換ファームウェア。Seeed XIAO ESP32-C6 上で Zephyr 4.3 により動作し、制限付き C++（例外・RTTI・動的確保・STL 禁止）で実装する。

```
[コントローラアプリ] <--BLE--> [XIAO ESP32-C6 (本ファーム)] <--UART--> [NUCLEO-G474RE USART1]
```

初版は BLE ペリフェラル（NUS 相当 GATT サービス）による透過 UART を実装する。無線トランスポートは `link` 静的インターフェースで抽象化してあり、Kconfig `UWC_TRANSPORT_*` で Wi-Fi 実装へ切替可能（FR-04 / NFR-03 参照）。

## 構成

- `spec/` — 仕様書（要件・基本設計・詳細設計）
- `src/` — 実装。`spec/detail_design/` と同じ層構造（`common → peripheral → device → feature → manager`）
- `boards/xiao_esp32c6_esp32c6_hpcore.overlay` — `host_uart` エイリアス等の devicetree 宣言
- `prj.conf` / `Kconfig` — Kconfig（shell/logging/CPP/BLE/トランスポート選択）
- `third_party/etl` — Embedded Template Library（git submodule、タグ 20.49.0、`ETL_NO_STL`）
- `cmake/` — lint/format スクリプト
- `.vscode/` — タスク設定

## 必要なツール

| ツール | 用途 | 導入 |
|---|---|---|
| Zephyr ワークスペース | Zephyr 4.3 + **hal_espressif**（無線 blob 含む） | `west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.3.0 C:\zephyr_ws` → `west update` → `west blobs fetch hal_espressif` |
| Zephyr SDK 0.17.4 | クロスコンパイル（riscv32-esp 用ツールチェーンを含む） | `C:\tools\zephyr-sdk-0.17.4` へ展開 |
| Python 3.13 + west | Zephyr ビルドシステム | `winget install Python.Python.3.13` → `pip install west` |
| CMake + Ninja | ビルド | `winget install Kitware.CMake` / `Ninja-build.Ninja` |
| LLVM（clang-tidy/clang-format） | 静的解析・整形 | `winget install LLVM.LLVM` |
| esptool | 書き込み（ESP32-C6 USB） | `pip install esptool` |

`ZEPHYR_TOOLCHAIN_VARIANT=zephyr` と `ZEPHYR_SDK_INSTALL_DIR=<SDK パス>` は `cmake` 実行時に必要（VSCode タスクには同梱済み）。ワークスペースが `C:\zephyr_ws` 以外の場合は環境変数 `ZEPHYR_WS_DIR` で `hal_espressif` の所在を指定する。

## ビルド・検証（ターミナル / AI エージェント共通）

```bash
# 環境変数（bash の場合）
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=C:/tools/zephyr-sdk-0.17.4

# ファームウェア（build-zephyr/zephyr/zephyr.elf/.bin）
cmake -S . -B build-zephyr -G Ninja -DBOARD=xiao_esp32c6/esp32c6/hpcore -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-zephyr

# 静的解析・整形
cmake -DCLANG_TIDY=clang-tidy -DCOMPILE_COMMANDS=build-zephyr/compile_commands.json -DSRC_ROOT=uart_wireless_converter_esp32/src -P cmake/run_clang_tidy.cmake
cmake -DCLANG_FORMAT=clang-format -DSRC_ROOT=$PWD/src -P cmake/run_clang_format.cmake

# 書き込み（Simple Boot、単一バイナリ。esptool は XIAO の USB ポートを自動検出）
esptool --chip esp32c6 write-flash 0x0 build-zephyr/zephyr/zephyr.bin
```

VSCode では `.vscode/tasks.json` の同名タスクが同等コマンドを実行する。

## UART / BLE 操作

- **ホスト UART**: `uart0`（XIAO D6=GPIO16 RX / D7=GPIO17 TX、115200 8N1）を NUCLEO の USART1（D0=PC5 RX / D1=PC4 TX）へ接続。UART に流れたバイトは BLE 接続中の相手へそのまま通知され、逆方向も同様に転送される。
- **診断**: USB-Serial-JTAG（`usb_serial`）に Zephyr shell が出る。`uwc status` でリンク状態・MTU・接続先、`uwc stats [reset]` で転送統計を表示する。
- **BLE**: デバイス名 `uwc-esp32c6` で advertising する。NUS 相当 UUID（Service `6E400001-…`、RX Write `6E400002-…`、TX Notify `6E400003-…`）なので nRF Connect / nRF Toolbox でも動作確認できる。切断時は自動で再 advertising する。

## 言語規約（制限付き C++）

- C++23。`-fno-exceptions -fno-rtti -fno-threadsafe-statics`（STL 不使用・例外・RTTI・動的確保禁止）。
- STL 相当は ETL を使う（`etl::array` 等）。
- 識別子は `snake_case`、定数のみ `UPPER_SNAKE` を `constexpr` で定義する。
- アプリケーション側コードはヒープを使わない。`CONFIG_HEAP_MEM_POOL_SIZE=32768` は ESP 無線スタック（hal_espressif blob）内部確保のためのもの（NFR-04）。

## 層構造

```
main.cpp
  └─ manager:   converter_application（配線・初期化・shell）
       ├─ feature:  uart_bridge（リング排出・双方向中継）
       │            link_monitor（状態・統計・活動時刻）
       ├─ device:   link ← link_ble.cpp / link_wifi.cpp（Kconfig 切替）
       └─ peripheral: host_uart（uart0 割込駆動）
                      debug_console（usb_serial printk）
```

依存は上位→下位のみ。`link` はイベントを注入コールバックで上位へ通知し、上位層を参照しない。

## 仕様駆動開発

`spec/requirements`（要求・ハード構成）→ `spec/basic_design`（基本設計・コンポーネント一覧）→ `spec/detail_design`（各コンポーネント詳細）→ `src`（実装）の順で同期して変更する。詳細は `spec/requirements/requirements.md` と本リポジトリの `.agents/instructions/firmware.md` を参照。
