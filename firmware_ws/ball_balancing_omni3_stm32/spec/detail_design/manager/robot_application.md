# robot_application

全コンポーネントの設定注入・初期化・実行ループを司る composition root。`robot_application` static class（システムに 1 つ）。複数インスタンスを要するコンポーネント（GPIO/PWM/エンコーダ/モータ）は `static inline` メンバとして保持し、その他は static class 呼出しとする。

## 公開型

```cpp
struct config {
    const param_set* params;
    mat33 imu_accel_axis_map;
    mat33 imu_gyro_axis_map;
};
struct telemetry_snapshot {
    balance_controller::output ctrl;
    state_estimator::output est;
    vec3 wheel_rate; bool enable; bool imu_ok;
    uint32_t outer_index; uint32_t tick_count; uint32_t overrun_count;
    uint32_t max_inner_us; uint32_t max_outer_us;
};
```

## 公開関数

- `static bool init(const config&)` — 全コンポーネントへ設定を注入し初期化する。ピン・PWM・QDEC・IMU はすべて devicetree（`bb,pins`/`timersN`/`imu0` エイリアス）から `DT_*` マクロで取得する。`odometry`/`position_controller`/`operation_manager`/`link_session` も初期化し、リンクスレッド開始用 `link_init_sem` を give する。最後に `control_tick::init()` + `start()` で 1 kHz ティックを開始する。IMU 検出失敗時は false。
- `[[noreturn]] static void run()` — `task_scheduler` に `inner_task_cb`/`outer_task_cb`/`diag_task_cb` を渡して実行する。
- `static link_session::stats link_stats()` — `bb link` 診断用カウンタを返す。
- `static telemetry_snapshot snapshot()` — shell スレッドからの状態参照用に `irq_lock()` で整合コピーを返す。
- `set_telemetry(bool)` / `telemetry_enabled()` — CSV テレメトリ送出の切替（`volatile` フラグ）。

## shell コマンド

| コマンド | 内容 |
|---|---|
| `bb status` | 動作状態・キャリブ状態・モード・IMU 健全性・オイラー角・輪速・トルク・オーバーラン・最大実行時間を表示（`status_cmd_cb`） |
| `bb telem [on\|off]` | 200 ms 周期の CSV テレメトリ送出を切替える（`telem_cmd_cb`） |
| `bb link` | ロボットリンクの受信フレーム数・CRC ドロップ・UART オーバーラン・TX ドロップ・最終受信時刻を表示（`link_cmd_cb`） |
| `bb start` | `operation_manager::request_start_shell` を呼び、受理判定結果を表示する（FR-12 の机上検証経路） |
| `bb stop` | `operation_manager::request_stop` を呼び待機へ遷移する |
| `bb calib` | `operation_manager::request_calib` を呼び、受理判定と完了結果を表示する（FR-13） |

## リンクスレッド

`K_THREAD_DEFINE` で生成される低優先度スレッド（`K_PRIO_PREEMPT(7)`）。`link_init_sem` で `init()` 完了を待ってから `link_session::run_once()` を無限ループで呼ぶ。制御スレッド（main）より低優先度のため、UART 受信処理が 1 kHz/200 Hz タスクをプリエンプトしない。

## 外側タスクの指令選択（FR-09/FR-10/FR-12/FR-13）

`outer_task_cb` は冒頭で `STANDBY`→`OPERATING` の遷移エッジを処理してから `state_estimator::step`・`odometry::update` を実行し、動作状態の処理（FR-12/FR-13）へ進む:

- **開始エッジ（FR-12）**: `drives_motors()` の立上りで、推定器・制御器・オドメトリの滞留状態を最初の `OPERATING` ティックへ持ち込まないため `step()` より前に `balance_controller::reset()` + `state_estimator::reset_estimate(true)` + `odometry::reapply_declared(raw_x, raw_y, euler_z)` を実行する。待機中も推定・積分は走り続けるため（下記）、手で車体を動かす等で蓄積した速度・姿勢・位置ドリフトをここで捨てる。受理判定で確認済みのバイアス資格と学習済みバイアス・yaw は保持し、`SET_POSE`/`RESET_ODOMETRY` の宣言位置へ再アンカーする。
- `operation_manager::take_bias_reset_req()` が真なら `state_estimator::reset_bias()` を呼ぶ（FR-13 キャリブレーション開始）。
- `operation_manager::calib_update(now_ms, qualified, imu_ok)` の返値が `>=0` なら `link_session::notify_calib_result(result, calib_seq, gyro_bias_z)` で `CALIB_RESULT` を送出する。
- `drives_motors()==false`（`STANDBY`/`FAULT`）→ `balance_controller::step` を**実行せず**、制御出力はゼロ化した `output`（`mode` は直前値維持、`attitude_torque=0`）として扱う。
- `drives_motors()==true`（`OPERATING`）→ `link_session::command()` の結果で指令源を切替える:
  1. `link_active=false`（一度もフレームを受けていない）→ 従来どおり `command_profile` の出力。
  2. `link_mode::VELOCITY` → `velocity_cmd`（500 ms WDG 失効時はゼロ化済み）。
  3. `link_mode::POSITION` → `position_controller::update(pose)` の出力。
  4. `link_mode::STANDBY` → 零指令（リンク側モード。`OPERATING` 中の「その場保持」に相当）。

  その後 `balance_controller::step` へ渡す。`OPERATING` 中に `ctrl.mode==0`（mode0 終端ラッチ）に到達したら `operation_manager::notify_controller_terminal()` を呼び `FAULT` へ遷移させる（FR-12）。

最後に `link_session::publish_state`（メールボックス更新。`tilt` は推定姿勢の傾斜角。`imu_accel`/`imu_gyro` は `lsm6dso::read()` の生値、`enc_count` は各 `encoder_counter::count()` の i32 切詰めを載せる。FR-18）と `generate_events`（EVT 生成→バッファ投入）を呼ぶ。qualified は `updated_qualification_time >= bias_qualification_time` で評価する。`EST`（推定器）と `odometry` は動作状態に関わらず常に更新する（待機中のキャリブレーション・位置合わせのため）。

## 内部状態

- 複数インスタンスコンポーネントの static メンバ（`led_pin_`/`estop_*_pin_`/`pwm_a_`/`pwm_b_`/`enc_count_`/`motors_`/`encoders_`）
- 直近の推定・制御・サーボ出力・`enable` フラグ等の ZOH コンテキスト（同一制御スレッド内で更新・消費されるため排他不要）

## MATLAB との対応・適応事項

- `main.cpp` 側は `robot_application::init(cfg)` → `run()` を呼ぶだけに留める。
- E-Stop 検出は `estop_input` で集約し、検出時は外側タスクへ `enable=false` を渡す。mode 0 成立後は内側デューティを 0 に固定する。ボタンの割込フラグは `requested()` 呼出しで消費されるワンショット型のため、外側タスクは `estop_latch_` に保持し、非 STANDBY 中は状態報告・掃引中断などの他消費者へも同じ要求を供給する（STANDBY では毎ティック解除）。
- テレメトリは `diag_task_cb` で CSV 1 行を整形して `debug_uart` に送信する（既定 off、`bb telem on` で有効化）。
