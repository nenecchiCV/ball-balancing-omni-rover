# encoder_test — エンコーダ方向・カウント検証

## 目的

- 3 輪のエンコーダが正しくカウントを取れること（結線・QDEC モード確認）。
- 正方向の定義（+duty → +輪速）とエンコーダの符号が一致すること。
- `test enc spin` ではモータ駆動により、DIR 配線とエンコーダ極性の
  一致を人がログから確認する。

## 前提

- 車体を手で保持し、車輪が自由に回る状態（車輪だけの無負荷でも可）。

## 手順

### 手回しモード

```text
uart:~$ test enc
enc test: rotate each wheel by hand in the expected positive direction;
          displacement/rate should be positive
enc t=1234 duty=0.00 cnt=[120 -45 300] disp=[0.012 -0.004 0.030] rate=[0.12 -0.04 0.30]
```

1. 各車輪を「前進方向」（+duty で駆動される方向）へ手で回す。
2. 対応するチャネルの `disp`/`rate`/`cnt` が正に変化することを確認する。

### モータ駆動モード

```text
uart:~$ test enc spin
enc spin: duty +0.15 (3 s) then -0.15 (3 s); expect rate sign == duty sign
enc t=1234 duty=0.15 cnt=[...] disp=[...] rate=[...]
```

1. 全モータが +0.15 duty で 3 秒、-0.15 duty で 3 秒駆動される。
2. 各 `rate` の符号が `duty` の符号と一致することを確認する。
3. 6 秒で自動停止する。

## ログ形式

```text
enc t=<ms> duty=<現在duty> cnt=[c0 c1 c2] disp=[d0 d1 d2] rate=[r0 r1 r2]
```

- `cnt` : `encoder_counter::count()` の累積生カウント（int64）。
- `disp` : 直近 100 ms のモデル変位 [rad]。
- `rate` : 直近 100 ms の輪速 [rad/s]。

## 合否基準

- 手回し: 前進方向の回転で `disp`/`rate` が正になる。
- spin: `rate` の符号が `duty` の符号と全車輪で一致する。
- 不一致の車輪は、エンコーダ A/B 相の接続入替または devicetree の
  チャネル割当を修正する。モータが指令と逆に回る場合は DIR 配線を修正する。

## 対応コンポーネント

- `tests/onboard/src/encoder_test/`（period 100 tick = 10 Hz）
- 下位層: `wheel_encoder`, `encoder_counter`, `mdd3a_motor`, `estop_input`
