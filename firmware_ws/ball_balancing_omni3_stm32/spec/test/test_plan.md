# オンボードテスト計画

実機検証用のテストファームウェア（`tests/onboard/`）の構成・実行方法を定義する。
実機ソフトとは別の Zephyr アプリケーションとしてビルドし、検証時のみ書き込んで使う。

## 目的

- `src/` の common・peripheral・device・feature 層を実機構成のまま再利用し、単体・結合レベルの動作を人がログを見ながら検証する。
- 倒立制御ループ（balance_controller / command_profile / wheel_servo / task_scheduler）は検証対象外として組み込まず、ハードウェア系統の確認を先に行う。

## 構成

```text
tests/onboard/
  CMakeLists.txt        # 実機アプリと同じ overlay・dts bindings・HAL モジュールを参照
  prj.conf              # 実機と同等の Kconfig（shell・sensor・qdec 有効）
  src/
    main.cpp
    test_application/   # コンポジションルート（robot_application と同配線）
    test_runner/        # shell `test` コマンド + 1 kHz ティック駆動ディスパッチ
    estop_test/
    tick_timing_test/
    imu_attitude_test/
    encoder_test/
    omni_drive_test/
    motor_deadzone_test/
```

### 実行モデル

- `test_runner` が TIM6 制御ティック（1 kHz）を待ち、アクティブテストの `update_cb` を
  テスト固有の `period_ticks` 間隔で呼び出す。`start_cb`/`stop_cb` は shell コンテキストから
  mutex 保持下で呼ばれ、テスト切替・停止は競合しない。
- テストは 1 つずつ実行する。別テストの開始は現在テストの `stop_cb` 後に行われる。
- ティックが滞留した場合は追いかけず、オーバーランとして `test` コマンドに報告する。
- モータを駆動するテスト（enc spin / omni / deadzone）は `estop_input::requested()` を
  毎ティック監視し、検出時は即座にデューティをゼロにして自動停止する。

### ログ

- すべてのテストログは ST-Link USB の仮想 COM（USART2, 115200）へ
  `printk`（`debug_uart` 経由）で出力する。shell プロンプトとは別系統の出力が
  混在するため、行先頭のテスト接頭辞（`estop`/`tick`/`imu`/`enc`/`omni`/`dz`）で区別する。

### 起動確認（ハートビート）

- **要件**: ファームウェア実行中、USER LED（LD2, PA5）は常に 1 秒周期で点滅する。
- `status_led::pattern::SLOW_BLINK`（500 ms トグル）を `test_application::init()` で
  無条件に設定し、`test_runner` の 1 kHz ループから `status_led::update()` を呼ぶ。
- 書き込み・リセット直後から点滅すれば MCU がファームウェアを実行している。
  点滅しない場合は書き込み失敗・起動前クラッシュ・電源系統を疑う。
- IMU 健全性などの状態区別は LED では行わず、`test imu` 等のログで確認する。

## ビルドと書き込み

```bash
# 構成
cmake -S tests/onboard -B tests/onboard/build -G Ninja \
  -DBOARD=nucleo_g474re -DCMAKE_BUILD_TYPE=Debug
# ビルド
cmake --build tests/onboard/build
# 書き込み（ST-Link 接続時）
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program tests/onboard/build/zephyr/zephyr.elf verify reset exit"
```

VSCode タスク: `configure-test` → `build-test` → `flash-test`。

## shell コマンド

```text
uart:~$ test            # テスト一覧と実行状態
uart:~$ test <name>...  # テスト開始（引数はテスト固有）
uart:~$ test stop       # 現在のテストを停止
```

| コマンド | テスト | 周期 | 詳細仕様 |
|---|---|---|---|
| `test estop` | E-stop 入力 | 10 Hz | [estop.md](estop.md) |
| `test tick` | ティック計時 | 1 kHz 計測 / 1 s 報告 | [tick_timing.md](tick_timing.md) |
| `test imu` | IMU+姿勢推定 | 200 Hz 推定 / 10 Hz 表示 | [imu_attitude.md](imu_attitude.md) |
| `test enc` / `test enc spin` | エンコーダ方向 | 10 Hz | [encoder_direction.md](encoder_direction.md) |
| `test omni <vx> <vy> <yaw>` | オムニ速度・回転指令 | 1 kHz 駆動 / 10 Hz 表示 | [omni_drive.md](omni_drive.md) |
| `test deadzone [dir] [max] [sec]` | モータ不感帯同定 | 1 kHz 掃引 / 20 Hz CSV | [motor_deadzone.md](motor_deadzone.md) |

## 安全上の注意

- モータ駆動テストは車体を手で保持し、車輪が接地しない状態で実施する。
- E-stop（外部 PC0 または基板ボタン PC13）はテスト中も有効であり、
  モータ駆動テストでは検出時に即時停止する。
- `test omni` は起動から 60 秒で自動停止する。

## 推奨実施順序

1. `test tick` — 制御タイムベースが 1 kHz で正しく動くこと。
2. `test estop` — 停止入力の極性と検出経路。
3. `test imu` — IMU 配線・軸写像・推定器の健全性。
4. `test enc` → `test enc spin` — エンコーダ方向とモータ DIR 極性の一致。
5. `test deadzone` — モータ不感帯の同定（`omni_drive_test` の `DEADZONE_COMP` や
   制御パラメータへ反映する材料）。
6. `test omni` — オムニ運動学と輪速制御系の結合確認。

## テストコンポーネントの追加規約

- 各テストは `<name>_test` の static class とし、`init(config)` でハンドルを受け取り、
  `start_cb`/`update_cb`/`stop_cb` を `test_runner` の登録テーブルに追加する。
- ログ行はテスト名の接頭辞で始め、数値は `snprintf` + `debug_uart::write` で出す。
- モータを動かすテストは必ず `estop_input::requested()` を監視し、`stop_cb` で
  デューティをゼロにする。
