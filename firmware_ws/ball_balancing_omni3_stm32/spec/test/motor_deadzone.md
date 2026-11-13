# motor_deadzone_test — モータ不感帯システム同定

## 目的

無負荷（車輪のみ）状態で駆動デューティを三角掃引し、車輪が回り始める
デューティ（不感帯・静止摩擦）をエンコーダ輪速から同定する。
上り・下り掃引で履歴差（ヒステリシス）も評価できる。

## 前提

- 車体を人が保持し、3 輪すべてが接地しない無負荷状態。

## 手順

```text
uart:~$ test deadzone            # dir=+1, max=0.25, ramp=15 s
uart:~$ test deadzone -1         # 逆方向掃引
uart:~$ test deadzone 1 0.30 20  # max=0.30, ramp=20 s
```

1. 起動すると 0→±max→0 の三角掃引を行い、終了で自動停止する。
2. 出力 CSV を保存し、duty に対して各 `w` が立ち上がる点を読み取る。
3. 上りと下りで立ち上がり・停止デューティが異なれば履歴差として記録する。

## ログ形式

```text
dz,t_s,duty,w0,w1,w2          # ヘッダ（開始時に出力）
dz,<s>,<duty>,<w0>,<w1>,<w2>  # 20 Hz CSV
dz,done                       # 掃引完了
```

- `duty` : 全モータ共通の印加デューティ（`dir*max*三角波`）。
- `w*` : `wheel_speed_meter` の計測輪速 [rad/s]（5 ms 窓）。

## 同定方法

- 上り掃引で |w| が明確に立ち上がる最小 |duty| を静止不感帯上限とする。
- 下り掃引で |w| がゼロに戻る |duty| を維持限界とする。
- 両者の中間値を線形部の実効不感帯とし、差を履歴幅とする。
- `omni_drive_test::DEADZONE_COMP`（既定 0.08）や制御パラメータへ反映する。

## 合否基準

- CSV が欠損なく記録され、全車輪で輪速が立ち上がる。
- 立ち上がりデューティが異常に大きい車輪は配線・駆動系を確認する。

## 対応コンポーネント

- `tests/onboard/src/motor_deadzone_test/`（period 1 tick = 1 kHz、CSV 20 Hz）
- 下位層: `wheel_speed_meter`, `mdd3a_motor`, `estop_input`
