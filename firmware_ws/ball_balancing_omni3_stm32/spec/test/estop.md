# estop_test — E-stop 入力検証

## 目的

外部 E-stop 入力（PC0）と基板ユーザボタン（PC13）の配線・極性・`estop_input` の
要求検出が想定どおり動くことを確認する。`spec/requirements/hardware_config.md`
の E-stop 極性未定事項の実機確認に対応する。

## 前提

- テストファームウェア書き込み済み。
- E-stop ボタン／ラインは操作可能な状態。

## 手順

```text
uart:~$ test estop
estop test: press external E-stop (PC0) or user button (PC13)
estop t=1234 ext_level=0 btn_level=0 requested=0
```

1. 何も押さずに 1 秒ごとのハートビート行で `requested=0` を確認する。
2. 外部 E-stop を押し、`ext_level` と `requested` が変化することを確認する。
3. 基板ボタンを押し、`btn_level` と `requested` が変化することを確認する。

## ログ形式

```text
estop t=<ms> ext_level=<0|1> btn_level=<0|1> requested=<0|1>
```

入力変化時は即時出力し、変化がなくても 1 秒ごとに現状態を出力する。
`ext_level`/`btn_level` は `gpio_pin::read()` の生レベル（HIGH=1）。

## 合否基準

- 非操作時: `requested=0` が継続する。
- 外部 E-stop 操作中: `requested=1` になる。
- 基板ボタン操作中: `requested=1` になる。
- 上記で `requested` が反転しない、または生レベルが期待極性と逆の場合は、
  devicetree の `GPIO_ACTIVE_LOW` 指定または配線を修正する。

## 対応コンポーネント

- `tests/onboard/src/estop_test/`（period 100 tick = 10 Hz）
- 下位層: `estop_input`, `gpio_pin`
