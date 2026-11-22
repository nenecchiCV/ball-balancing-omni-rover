# オンボードテスト結果 20261123_022544

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=999 min_us=833 max_us=1000 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 77 行, valid 率 100% |
| enc | REVIEW | 56 行, 最終 cnt=('0', '0', '0')（方向は目視確認） |
| enc spin | PASS | +duty rate=[9.12, 9.44, 8.66] / -duty rate=[-9.02, -8.75, -8.75] |
| deadzone | REVIEW | 不感帯推定 |duty| w0=0.254 w1=0.180 w2=0.180（最大値超過時は不動） |
| omni | PASS | 93 行, 最終 w=[0.0, 2.76, -1.84] |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
