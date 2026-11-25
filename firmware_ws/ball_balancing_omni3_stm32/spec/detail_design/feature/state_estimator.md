# state_estimator

`ballbotEstimatorStep` をそのまま移植した 14 状態推定器。`state_estimator` static class。

## 公開型

```cpp
struct config { const param_set* params; };
struct output {
    vec4 quaternion; vec3 velocity_world; vec3 ball_rate_world;
    vec2 relative_position_body; float gyro_bias_z; float qualification_time;
    vec3 euler; vec3 gyro_body; vec2 velocity_world2; vec2 rel_pos_body2;
    vec3 accel_world; bool still; float updated_gyro_bias_z;
    bool bias_learning_enabled; float updated_qualification_time; bool valid;
};
```

## 公開関数

- `static void init(const config&)` — パラメータ参照を保持し、全内部状態を初期値（quaternion 単位、他 0）でリセットする。
- `static output step(const imu_sample& imu, const vec3& wheel_rate, float dt)` — `ballbotEstimatorStep` の 1 ステップを実行し、`estimate` と `diagnostics` を返す。`imu_sample` は `common/imu_sample`（device 非依存のデータ構造）のため、本クラスはハードウェア無しでホストテスト可能。`out.still` は待機中慣性オドメトリ（FR-11）の ZUPT 用静止候補で、`assembly_still`（フィルタ済み輪速・ジャイロ閾値）かつ水平加速度残差 `|accel_world.xy| ≤ 0.3 m/s²`（実装定数）のとき立つ。等速運動は検知できない。
- `static void reset_bias()` — ジャイロバイアス学習状態（`gyro_bias_z_`・`ekf_bias_xy_`・`qualification_time_`）をリセットし、バイアス捕捉を武装する（`CALIBRATE_IMU` 対応）。姿勢・速度など他の推定状態は保持する。外側タスクから呼ぶ（`operation_manager::take_bias_reset_req` 経由）。捕捉中は `step()` が連続する静止・有効サンプルの生ジャイロを累積し、累積時間が `bias_qualification_time` に達した時点で時間平均を `gyro_bias_z_`・`ekf_bias_xy_` へ採用する（単一サンプルではなく区間平均。失格・無効サンプルでウィンドウを取り直す）。採用後も IIR/EKF の継続学習は進行する。
- `static void request_reset()` — 推定リセットを要求する。リンクスレッドから呼び（`SET_POSE` 受理時）、制御スレッドの `step()` 先頭で `reset_req_` を消費して `reset_estimate()` を実行する。
- `static void reset_estimate(bool keep_qualification = false)` — 動的推定状態を初期値へ戻す。quaternion は yaw 成分のみ残した回転へ置き換え `ekf_init_=false` とすることで、次の静止サンプルでロール/ピッチが加速度から再シードされ yaw はリセット前の値を保持する（yaw は加速度から観測できず、跳ばせるとオドメトリの yaw オフセットと不整合になるため）。速度・輪速・相対位置・補助フィルタも全てリセット。学習済みバイアス（`gyro_bias_z_`・`ekf_bias_xy_`）は保持する。`keep_qualification=true` のとき `qualification_time_` も保持する（`START_OPERATION` 受理時の動作開始リセット用。受理判定で資格成立を確認済みのため再証明待ちを挟まない）。`SET_POSE` 経由（`request_reset`）では既定値で呼ばれ、資格は再証明される。外側タスクから呼ぶ。

## 内部状態（MATLAB persistent の写像）

- `state`（14 要素相当）: `quaternion_`, `velocity_world_`, `ball_rate_world_`, `relative_position_body_`, `gyro_bias_z_`, `qualification_time_`
- EKF: `ekf_covariance_`（mat55）, `ekf_bias_xy_`（vec2）, `ekf_accel_world_`, `prev_ball_velocity_world_`
- 静止・補助フィルタ: `filtered_wheel_rate_for_still_`, `stillness_hold_`, `ekf_rel_vel_body_`, `ekf_rel_acc_body_`, `ekf_gyro_dot_body_`, `ekf_prev_gyro_body_`, `ekf_wheel_slew_`, `ekf_prev_wheel_rate_`, `inertial_hold_`
- リセット要求: `reset_req_`（リンクスレッド→制御スレッドの受信箱、`irq_lock` 保護）
- キャリブレーション捕捉: `bias_capture_sum_`・`bias_capture_count_`・`bias_capture_active_`（`reset_bias` で武装し、静止ウィンドウ平均採用で解除）

## MATLAB との対応・適応事項

`ballbotEstimatorStep.m` の行をほぼ 1:1 に写像する。`isfield` による機能スイッチは `constexpr` フラグで固定（`ekf.enabled = true`, `ekf.useLegacyCorrection = false`）。`pinv` は `inverse3` で実現する。

MATLAB にはない実機向け追加（姿勢シード/再シード）:

- **初期化シード**: `ekf_init_` は「比力が重力と整合（`|a|−g| ≤ ek.residual_gate`、≈静止）」する最初のサンプルまで遅延され、その比力方向からロール/ピッチを決めて `quaternion` をシードする（yaw は従前値を維持）。MATLAB 側は初期姿勢既知（`ballbotEstimatorInitialState`）のため残差ゲートは収束圏でしか機能しないが、実機は床置き等大傾斜起動で初期残差がゲート超過となり、これがないと加速度更新が永久拒否されてジャイロバイアスで傾斜が積算される（固着）。
- **固着時の再シード**: `update_allowed`（inertial 静止証明済み）成立にもかかわらず `|residual| > ek.residual_gate` が `kReseedHoldTime`（2 s）連続した場合は姿勢種ズレの固着と判定し、同じシード処理で EKF 内部状態（共分散・bias・加速度フィルタ）をリセットして復帰する。運搬中のドリフト蓄積等による固着を自己回復する。
