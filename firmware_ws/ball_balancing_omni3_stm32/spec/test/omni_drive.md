# omni_drive_test — オムニ速度・回転指令検証

## 目的

`lqi.wheel_speed_from_planar_velocity` による [vx, vy, yaw]→3 輪速写像と、
モータ・エンコーダの実装経路を結合し、指定方向の速度指令・回転指令で
各車輪が期待どおり回ることを確認する（倒立制御は使用しない）。

## 前提

- 車体を人が手で保持し、車輪がボール・地面に接触しない無負荷状態で実施する。
- 事前に `test enc spin` でモータ・エンコーダ極性を確認済みであること。

## 手順

```text
uart:~$ test omni 0.2 0 0      # +x 方向 0.2 m/s
uart:~$ test omni 0 0.2 0      # +y 方向 0.2 m/s
uart:~$ test omni 0 0 0.5      # +yaw 0.5 rad/s（原地回転）
uart:~$ test omni 0 0 0        # 指令ゼロ（停止）
uart:~$ test stop              # テスト終了
```

- 指令は `max_speed`（0.4 m/s）・`max_yaw_rate`（0.8 rad/s）にクランプされる。
- 起動から 60 秒で自動停止する。

## ログ形式

```text
omni: cmd=[vx vy yaw] wheel_cmd=[w0 w1 w2] rad/s; HOLD the chassis, ...
omni t=<ms> wcmd=[w0 w1 w2] w=[w0 w1 w2] duty=[d0 d1 d2]
```

- `wcmd` : `mul(wheel_speed_from_planar_velocity, [vx vy yaw])` の輪速指令 [rad/s]。
- `w` : `wheel_speed_meter` の計測輪速（5 ms 窓）。
- `duty` : テスト内蔵サーボの出力デューティ。

## 輪速サーボ（テスト固有）

製品の `wheel_servo` は推進トルク上限 0.045 Nm であり、無負荷車輪の
デッドゾーンを抜けられない可能性がある。本テストでは動作確認を優先し、
テスト固有のデューティ生成を使う。

```text
duty = (back_emf_constant/supply_voltage) * wcmd
     + 0.03 * (wcmd - w_meas)
     + sign(wcmd) * 0.08   // 不感帯補償
```

不感帯補償値は `test deadzone` の同定結果に応じて調整する。

## 合否基準

- 各コマンドで `w` が `wcmd` の方向・概算大きさに追従する（定常誤差は
  不感帯の影響として評価する）。
- `vx` 指令で主に wheel1/wheel2 が逆符号、`vy`/yaw 指令で 3 輪が同符号系に
  回ること（行列の構造どおり）。
- 期待と異なる回転方向の場合は運動学行列・DIR 配線・エンコーダ極性を再確認する。

## 対応コンポーネント

- `tests/onboard/src/omni_drive_test/`（period 1 tick = 1 kHz、表示 10 Hz）
- 下位層: `wheel_speed_meter`, `wheel_encoder`, `mdd3a_motor`, `estop_input`
