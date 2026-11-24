# imu_attitude_test — IMU+姿勢推定検証

## 目的

- LSM6DSO が I2C 経由で有効データを返すこと（結線確認）。
- `state_estimator` が IMU 単体入力で姿勢（オイラー角）を推定できること。
- 加速度・ジャイロ・推定角の各軸方向が車体の想定方向と一致すること
  （`axis_map` 行列の妥当性確認）。

## 前提

- 車体を手で保持し、傾けられる状態にする。
- 本テストは輪速入力をゼロとして推定器を回す（IMU 単体検証）。

## 手順

```text
uart:~$ test imu
imu test: tilt the chassis; check euler_deg matches the physical roll/pitch/yaw
imu t=5000 acc=[0.01 0.02 9.80] gyr=[0.001 -0.002 0.000] euler_deg=[0.6 -0.4 12.3] valid=1
```

1. 水平に静置 → `euler_deg` の roll/pitch が 0 付近で安定することを確認。
2. 車体を前後に傾ける → pitch が傾け方向に応じて ± に変化することを確認。
3. 左右に傾ける → roll が同様に変化することを確認。
4. 水平のまま回転させる → yaw が変化し、gyr[2] が回転方向と同符号になることを確認。
5. `acc` が静置でほぼ [0 0 9.8] になること（軸写像・極性の確認）。

## ログ形式

```text
imu t=<ms> acc=[x y z] gyr=[x y z] euler_deg=[roll pitch yaw] valid=<0|1>
```

200 Hz で `lsm6dso::read()` → `state_estimator::step()` を実行し、
10 Hz で表示する。`acc` は比力 [m/s^2]、`gyr` は角速度 [rad/s]（機体軸）。

## 合否基準

- `valid=1` が継続する。
- 水平静置で |roll|, |pitch| が数度以内で安定。
- 傾けた方向と euler の符号変化が物理的に一致する。不一致の場合は
  `lsm6dso::config::accel_axis_map` / `gyro_axis_map`（またはハーネス配線）を
  修正する。角速度は機体軸まわりの右ねじ正転で正になることを
  各軸について確認する（`test imu` の `gyr` 出力）。
- 期待符号（機体軸 +X=P0方向、+Y=左側、右ねじ正転が正）:

  | 操作 | acc（比力） | euler | gyr |
  |---|---|---|---|
  | 水平静置 | ≈ [0, 0, +9.8] | roll/pitch ≈ 0 | ≈ 0 |
  | +X 側（前方）を下げる | acc_x < 0 | pitch > 0 | — |
  | +Y 側（左側）を下げる | acc_y < 0 | roll < 0 | — |
  | 俯瞰で左回転（CCW） | — | yaw 増加 | gyr_z > 0 |

  acc は比力なので下げた側の軸成分は負が正しい値（重力方向とは逆符号）。

## 対応コンポーネント

- `tests/onboard/src/imu_attitude_test/`（period 5 tick = 200 Hz）
- 下位層: `lsm6dso`, `state_estimator`
