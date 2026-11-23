# state_estimator

`ballbotEstimatorStep` をそのまま移植した 14 状態推定器。`state_estimator` static class。

## 公開型

```cpp
struct config { const param_set* params; };
struct output {
    vec4 quaternion; vec3 velocity_world; vec3 ball_rate_world;
    vec2 relative_position_body; float gyro_bias_z; float qualification_time;
    vec3 euler; vec3 gyro_body; vec2 velocity_world2; vec2 rel_pos_body2;
    vec3 accel_world; float updated_gyro_bias_z; bool bias_learning_enabled;
    float updated_qualification_time; bool valid;
};
```

## 公開関数

- `static void init(const config&)` — パラメータ参照を保持し、全内部状態を初期値（quaternion 単位、他 0）でリセットする。
- `static output step(const imu_sample& imu, const vec3& wheel_rate, float dt)` — `ballbotEstimatorStep` の 1 ステップを実行し、`estimate` と `diagnostics` を返す。`imu_sample` は `common/imu_sample`（device 非依存のデータ構造）のため、本クラスはハードウェア無しでホストテスト可能。
- `static void reset_bias()` — ジャイロバイアス学習状態（`gyro_bias_z_`・`ekf_bias_xy_`・`qualification_time_`）をリセットする（`CALIBRATE_IMU` 対応）。姿勢・速度など他の推定状態は保持する。外側タスクから呼ぶ（`operation_manager::take_bias_reset_req` 経由）。

## 内部状態（MATLAB persistent の写像）

- `state`（14 要素相当）: `quaternion_`, `velocity_world_`, `ball_rate_world_`, `relative_position_body_`, `gyro_bias_z_`, `qualification_time_`
- EKF: `ekf_covariance_`（mat55）, `ekf_bias_xy_`（vec2）, `ekf_accel_world_`, `prev_ball_velocity_world_`
- 静止・補助フィルタ: `filtered_wheel_rate_for_still_`, `stillness_hold_`, `ekf_rel_vel_body_`, `ekf_rel_acc_body_`, `ekf_gyro_dot_body_`, `ekf_prev_gyro_body_`, `ekf_wheel_slew_`, `ekf_prev_wheel_rate_`, `inertial_hold_`

## MATLAB との対応・適応事項

`ballbotEstimatorStep.m` の行をほぼ 1:1 に写像する。`isfield` による機能スイッチは `constexpr` フラグで固定（`ekf.enabled = true`, `ekf.useLegacyCorrection = false`）。`pinv` は `inverse3` で実現する。
