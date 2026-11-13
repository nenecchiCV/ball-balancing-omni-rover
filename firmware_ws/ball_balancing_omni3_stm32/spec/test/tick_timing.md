# tick_timing_test — 制御ティック計時検証

## 目的

TIM6 の 1 kHz 制御ティックが仕様どおりの周期で供給されることを確認する。
プリスケーラ設定ミスや割込登録不良を検出する。

## 手順

```text
uart:~$ test tick
tick test: period stats every 1 s (expected ~1000 us)
tick n=1000 mean_us=1000 min_us=999 max_us=1001 missed=0
```

数秒間観察し、`test stop` で終了する。

## ログ形式

```text
tick n=<サンプル数> mean_us=<平均> min_us=<最小> max_us=<最大> missed=<欠落>
```

- `update_cb` の呼出間隔を `timebase::micros()` で計測する。
- 間隔が 1500 us を超えたサンプルは欠落として `missed` に加算する。
- 報告出力（printk）はブロッキングのため、報告直後の 1 サンプルは統計から捨てる。

## 合否基準

- `mean_us` が 1000 ± 数 us、`max_us` がおおむね 1100 us 未満。
- `missed=0`（報告直後の摂動を除く）。
- 平均が顕著にずれる場合は TIM6 のクロック・プリスケーラ・ARR を確認する。

## 対応コンポーネント

- `tests/onboard/src/tick_timing_test/`（period 1 tick = 1 kHz）
- 下位層: `control_tick`, `timebase`
