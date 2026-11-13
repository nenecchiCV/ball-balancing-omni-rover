# robot_application

全コンポーネントの設定注入・初期化・実行ループを司る composition root。`robot_application` static class（システムに 1 つ）。複数インスタンスを要するコンポーネント（GPIO/PWM/エンコーダ/モータ）は `static inline` メンバとして保持し、その他は static class 呼出しとする。

## 公開型

```cpp
struct config {
    const param_set* params;
    mat33 imu_axis_map;
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

- `static bool init(const config&)` — 全コンポーネントへ設定を注入し初期化する。ピン・PWM・QDEC・IMU はすべて devicetree（`bb,pins`/`timersN`/`imu0` エイリアス）から `DT_*` マクロで取得する。最後に `control_tick::init()` + `start()` で 1 kHz ティックを開始する。IMU 検出失敗時は false。
- `[[noreturn]] static void run()` — `task_scheduler` に `inner_task_cb`/`outer_task_cb`/`diag_task_cb` を渡して実行する。
- `static telemetry_snapshot snapshot()` — shell スレッドからの状態参照用に `irq_lock()` で整合コピーを返す。
- `set_telemetry(bool)` / `telemetry_enabled()` — CSV テレメトリ送出の切替（`volatile` フラグ）。

## shell コマンド

| コマンド | 内容 |
|---|---|
| `bb status` | モード・IMU 健全性・オイラー角・輪速・トルク・オーバーラン・最大実行時間を表示（`status_cmd_cb`） |
| `bb telem [on\|off]` | 200 ms 周期の CSV テレメトリ送出を切替える（`telem_cmd_cb`） |

## 内部状態

- 複数インスタンスコンポーネントの static メンバ（`dir_pins_`/`led_pin_`/`estop_*_pin_`/`pwm_ch_`/`enc_count_`/`motors_`/`encoders_`）
- 直近の推定・制御・サーボ出力・`enable` フラグ等の ZOH コンテキスト（同一制御スレッド内で更新・消費されるため排他不要）

## MATLAB との対応・適応事項

- `main.cpp` 側は `robot_application::init(cfg)` → `run()` を呼ぶだけに留める。
- E-Stop 検出は `estop_input` で集約し、検出時は外側タスクへ `enable=false` を渡す。mode 0 成立後は内側デューティを 0 に固定する。
- テレメトリは `diag_task_cb` で CSV 1 行を整形して `debug_uart` に送信する（既定 off、`bb telem on` で有効化）。
