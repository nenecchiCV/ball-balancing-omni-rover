# ファームウェア + アプリ開発コンテナ（devcontainer）

`firmware_ws/` 配下の両プロジェクト（nucleo_g474re / xiao_esp32c6、Zephyr 4.3）と
`app_ws/robot_controller_tauri`（Next.js × Tauri 2）の
開発・ビルド・静的解析・ホストテスト・デバッグ環境をまとめたコンテナです。

## コンテンツ

| ツール | バージョン | 用途 |
|---|---|---|
| Zephyr ワークスペース | v4.3.0（`/opt/zephyr_ws`） | hal_stm32 / hal_st / cmsis_6 / hal_espressif+blob |
| Zephyr SDK | 0.17.4（`/opt/zephyr-sdk-0.17.4`） | arm-zephyr-eabi + riscv64-zephyr-elf |
| LLVM | 21（clang-format のみ 23） | clang++ ホストテスト / clang-tidy / clang-format / clangd |
| OpenOCD | Debian 同梱版 | STM32 書き込み・デバッグ（ST-LINK） |
| openocd-esp32 | 0.12.0-esp32-20260831（`/opt/openocd-esp32`） | ESP32-C6 内蔵 USB-JTAG デバッグ |
| esptool / west / cmake / ninja / ccache | 最新 | ビルド・ESP32 書き込み |
| Node.js | 24.19.0（`/opt/node`） | アプリ フロントエンドビルド |
| Rust | stable + clippy/rustfmt（`~/.cargo`） | Tauri バックエンドビルド |
| WebKitGTK / BlueZ 依存 | Ubuntu 同梱版 | `tauri build` / `btleplug` の Linux ビルド |

必要な環境変数（`ZEPHYR_BASE` / `ZEPHYR_WS_DIR` / `ZEPHYR_SDK_INSTALL_DIR` /
`ZEPHYR_TOOLCHAIN_VARIANT` / venv・Node・cargo の `PATH`）はイメージに設定済みです。

## 起動方法

### VS Code Dev Containers

1. VS Code に **Dev Containers** 拡張機能を入れる
2. リポジトリルートを開き、`Ctrl+Shift+P` → `Dev Containers: Reopen in Container`
3. コンテナ内で `firmware_ws/ball_balancing_omni3_stm32` または
   `firmware_ws/uart_wireless_converter_esp32` をフォルダとして開く

`.vscode/` のタスク・launch 構成はコンテナ内でもそのまま使えます。

### docker CLI / エージェントからの実行

```powershell
# イメージ作成（リポジトリルートで）
docker build -f .devcontainer/Dockerfile -t bbor-devcontainer .

# コンテナ起動（リポジトリを bind mount、USB パススルー用に privileged）
docker run -d --name bbor --privileged `
  -v "${PWD}:/workspaces/ball-balancing-omni-rover" `
  -w /workspaces/ball-balancing-omni-rover `
  bbor-devcontainer sleep infinity

# コマンド実行（エージェントはこの形でビルド・テストを実行する）
docker exec bbor bash -lc "cd firmware_ws/ball_balancing_omni3_stm32 && cmake -S . -B build-zephyr -G Ninja -DBOARD=nucleo_g474re && cmake --build build-zephyr"
```

初回の動作確認はコンテナ内で `bash .devcontainer/check-env.sh` を実行してください。

## コンテナ内でできること

```bash
# STM32 アプリ（build-zephyr/zephyr/zephyr.elf/.hex/.bin）
cd firmware_ws/ball_balancing_omni3_stm32
cmake -S . -B build-zephyr -G Ninja -DBOARD=nucleo_g474re -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-zephyr

# ESP32 アプリ
cd firmware_ws/uart_wireless_converter_esp32
cmake -S . -B build-zephyr -G Ninja -DBOARD=xiao_esp32c6/esp32c6/hpcore -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-zephyr

# ホスト単体テスト・静的解析・整形（各プロジェクト README のコマンドと同じ）

# Tauri アプリ（app_ws/robot_controller_tauri）
cd app_ws/robot_controller_tauri
npm ci
npx tsc --noEmit              # フロント型検査
npm run build                 # Next.js 静的エクスポート (out/)
cd src-tauri
cargo check                   # バックエンド型検査
cargo test                    # プロトコル単体テスト
cargo build                   # Linux 実行バイナリ生成
cd .. && npm run tauri build  # Linux バンドル（deb / rpm / AppImage）
```

### アプリ（robot_controller_tauri）の制約

- コンテナ内 `tauri build` の成果物は Linux 向け（deb / rpm / AppImage）。
  Windows 向けインストーラ（.msi/.exe）はホスト側で `npm run tauri build` を実行する。
- BLE 実機通信はホストの Bluetooth コントローラに依存する。Linux ホストでは
  `docker run` に `--net=host -v /run/dbus:/run/dbus` を付けるとコンテナ内アプリから
  BlueZ 経由で接続できる。Windows + Docker Desktop では Bluetooth を渡せないため、
  実機通信を含む実行はホスト側 `npm run tauri dev` を使う。
- `tauri dev` の GUI 表示は X サーバ（`DISPLAY`）を必要とする。既定構成は
  `cargo check` / `cargo test` / `npm run build` / `tauri build` のヘッドレス検証を
  想定する。
- `node_modules` にはプラットフォーム依存バイナリが含まれる。bind mount で
  ホストと共有されるため、コンテナ内で `npm ci` / `npm install` すると
  Linux 用に入れ替わりホスト側の実行が壊れる。コンテナ内で依存解決する場合は
  別ディレクトリにコピーして実行するか、ホスト側で再 `npm install` して戻す。

## 書き込み・デバッグ（USB デバイスの扱い）

Windows + Docker Desktop は USB を直接パススルーできません。以下の 2 経路を用意しています。

### 経路 A: 書き込み・デバッグサーバはホスト側（最も確実）

USB デバイスに触るプロセスだけ Windows 側で動かします。

| 対象 | ホスト側で実行 | コンテナ側 |
|---|---|---|
| STM32 書き込み | `openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program ... verify reset exit"` | ビルドのみ |
| STM32 デバッグ | `openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "bindto 0.0.0.0"` | `Debug STM32 (host OpenOCD)`（gdb が `host.docker.internal:3333` に接続） |
| ESP32 書き込み | `esptool --chip esp32c6 write_flash 0x1000 build-zephyr/zephyr/zephyr.bin` | ビルドのみ |
| ESP32 デバッグ | openocd-esp32（Windows 版を要導入）を `-f board/esp32c6-builtin.cfg -c "bindto 0.0.0.0"` で起動 | `Debug ESP32-C6 (host OpenOCD)` |
| UART モニタ | ターミナルアプリ（Tera Term 等） | — |

ESP32 用 Windows 版 openocd-esp32 は
[espressif/openocd-esp32 releases](https://github.com/espressif/openocd-esp32/releases)
の `openocd-esp32-win64-*.zip` を展開して使います。

### 経路 B: usbipd で USB をコンテナへパススルー

`usbipd-win` で ST-LINK / ESP32-C6 を WSL2 経由でコンテナに見せます。

```powershell
# ホスト（管理者 PowerShell）で
winget install usbipd                      # 初回のみ
.devcontainer\usb-attach.ps1               # ST-LINK/ESP32-C6 を検出して attach
.devcontainer\usb-attach.ps1 -Detach       # 外すとき
.devcontainer\usb-attach.ps1 -BusId 2-3    # バス ID 指定も可
```

コンテナ側は `--privileged`（devcontainer.json / docker run 例に設定済み）で起動し、
`bash .devcontainer/check-env.sh` の USB 欄でデバイスが見えることを確認します。
見えた後はコンテナ内でそのまま実行できます:

```bash
# STM32 書き込み・デバッグ（VS Code の "Debug STM32 (OpenOCD)" も使用可）
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build-zephyr/zephyr/zephyr.elf verify reset exit"

# ESP32 書き込み（USB-Serial-JTAG が /dev/ttyACM* 等に見える）
esptool --chip esp32c6 write_flash 0x1000 build-zephyr/zephyr/zephyr.bin

# ESP32 デバッグ（openocd-esp32 + riscv64 gdb。"Debug ESP32-C6 (OpenOCD, usbipd)" 構成）
/opt/openocd-esp32/bin/openocd -f board/esp32c6-builtin.cfg   # :3333 で gdb 待受
```

### 補足・既知の制約

- usbipd の attach は WSL2 VM 単位です。Docker Desktop 配下で見えない場合は、
  リポジトリを WSL ディストロ内に置き、その WSL 内の docker でコンテナを動かす形が確実です。
- usbipd 中はホスト側から同じ COM ポートを開けません。モニタは detach するか別ポートで。
- Windows での usbipd パススルーは実機未検証です。まず経路 A で運用し、必要時に試してください。

## エージェントと MCP サーバ

MCP サーバはエージェントのプロセスが起動・接続します。

- **Devin Desktop などホスト側のエージェント**: コマンドだけ `docker exec` でコンテナに流す限り、
  `.devin/mcp_config.json`（matlab: stdio / fusion: `http://127.0.0.1:8765`）は変更不要です。
- **エージェント本体をコンテナ内で動かす場合**: ホストのローカル専用サービスに向き替える設定が必要です。

| MCP | 実態 | コンテナからの接続方法 |
|---|---|---|
| fusion | Fusion360.exe が `127.0.0.1:8765` で LISTEN | ホストで `netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=127.0.0.1`（要管理者）→ コンテナ側は `http://host.docker.internal:8765/mcp` |
| matlab | `matlab-mcp-server.exe`（stdio 専用、既存 MATLAB セッションに接続） | ホストで `uvx mcp-proxy --port 8777 --host 0.0.0.0 -- "C:\Users\<user>\.matlab\agentic-toolkits\bin\matlab-mcp-server.exe"` などで stdio→HTTP ブリッジ → コンテナ側は `http://host.docker.internal:8777` |

コンテナ内エージェント用の上書き設定例（`.devin/mcp_config.local.json`、gitignore 済み）:

```json
{
  "mcpServers": {
    "autodesk-fusion-mcp": {
      "url": "http://host.docker.internal:8765/mcp",
      "transport": "http"
    },
    "matlab": {
      "url": "http://host.docker.internal:8777/mcp",
      "transport": "http"
    }
  }
}
```

## 手動で必要な作業まとめ（自動化できない部分）

1. `winget install usbipd` と `.devcontainer/usb-attach.ps1`（管理者、経路 B のみ）
2. `netsh interface portproxy`（管理者、コンテナ内エージェントから Fusion を使う場合のみ）
3. `mcp-proxy` ブリッジ起動（matlab をコンテナ内エージェントに見せる場合のみ）
4. MATLAB 側の `satk_initialize`（matlab MCP を使うたびに必要）
5. ESP32 経路 A のデバッグ時のみ、Windows 版 openocd-esp32 の導入
6. usbipd 利用時の実機での疎通確認
