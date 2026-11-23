# balance_controller

`ballbotLqiControllerUpdateRev2` をそのまま移植した外側タスク。`balance_controller` static class。

## 公開型

```cpp
struct config { const param_set* params; };
struct output {
    vec3 attitude_torque; vec2 velocity_integral; uint8_t mode;
    bool yaw_bias_ready; vec3 wheel_rate_command; bool settle_witness; bool enable;
};
```

## 公開関数

- `static void init(const config&)` — パラメータ参照を保持し、全内部状態をリセットする。
- `static void reset()` — 全内部状態（モードラッチ・積分・フィルタ・ランプ時計・ヨーバイアスラッチ等）を初期状態へ戻す。`START_OPERATION` 受理時（`STANDBY`→`OPERATING` 遷移）に外側タスクから呼び、待機前の残留状態を持ち込まない（FR-12）。パラメータ参照は保持する。
- `static output step(const state_estimator::output& est, const vec3& command, bool enable, const vec3& wheel_rate)` — 外側 1 ステップを実行する。速度積分・ヨーバイアスラッチ等の入出力引数は内部状態として保持する。`STANDBY` 中は呼出し側が `step` 自体を実行しない（呼べば `!enable` で mode 0 ラッチが立つため）。

## 内部構成

`step()` は MATLAB 側の処理順序を保ったまま、責務ブロックごとのプライベートヘルパへ分割する。各ヘルパは 1 ステップ分の作業領域 `step_context`（ローカル状態の集約構造体）を参照渡しで受け取る。

| ヘルパ | 担当ブロック |
|---|---|
| `update_yaw_bias_latch` | `ballbotYawBiasStartupGuard` 相当のラッチ |
| `update_settle_tracking` | エレベーテッド権限・settle 輪速フィルタ |
| `prepare_commands` | 体軸回転・連成指令リミット・最大速度クランプ |
| `update_ball_velocity` | ボール速度・settle 切替・追従基準速度 |
| `apply_follow_brake` | フォロースルーブレーキ |
| `integrate_velocity` | 速度積分（デッドゾーン・指令ゲート・アンワインド下流側） |
| `update_filters` | ジャイロ・輪速ローパス |
| `apply_yaw_gate` | ヨーゲート・速度状態フィードバック・積分リーク |
| `compute_acceleration_demand` | 速度外ループ（lean 要求・各種ガード）・静的リーン適応注入 |
| `synthesize_torque` | 傾斜ブースト・LQI・傾斜積分・ホールド FF・コモンモード・スリップ/ヨー/ボールダンパー |
| `resolve_output` | トルク上限・mode 2 リカバリ・mode 0 出力 |
| `update_static_lean_adaptation` | 静的リーン適応の更新 |
| `apply_anti_windup` | 残余→速度積分リーク |
| `startup_ramp` | 起動ランプ |

## 内部状態（MATLAB persistent の写像）

- `prev_mode_`, `mode0_latch_`, `elevated_authority_`, `settle_wheel_rate_`, `settle_kin_prev_`, `settle_kin_accel_`
- `body_rate_filter_`, `wheel_rate_filter_`（オブザーバ無効時のローパス）, `tilt_integral_`, `prev_command_velocity_`, `acceleration_demand_slew_`
- `static_lean_bias_`, `drift_filter_`, `drift_rate_`, `prev_drift_mag_`, `seen_still_`, `prev_adapt_rest_`
- `yaw_bias_ready_`（`ballbotYawBiasStartupGuard` のラッチ）、`velocity_integral_`、`ramp_clock_`/`ramp_started_`

## MATLAB との対応・適応事項

- モード判定・エレベーテッド権限・フォロースルー・連成指令リミット・速度積分（デッドゾーン/リーク/逆ワインド）・リーンポイント外ループ（P + accelFF + dragFF + スルーレート + フェード + エキセントリック + コースト）・静的リーン適応・ヨーゲート・傾斜ブースト・LQI 適用・傾斜積分・コモンモード除去・スリップダンパー・ボールスピンブレーキ・ヨーダンパー・mode2 回復トルク・settle 処理を全て含む。
- `wheelRateObserverEnabled` は既定無効のため実装対象外（有効化時は `wheel_rate_observer_gain`/`DriveGain`/`Compliance` を定数に追加）。
- `openLoopTorque` は実装対象外（診断用フック）。
- `pinv(W_map)` は `inverse3` で実現。
