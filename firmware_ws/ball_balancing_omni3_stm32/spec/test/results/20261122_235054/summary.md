# オンボードテスト結果 20261122_235054

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=998 min_us=194 max_us=1001 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 76 行, valid 率 100% |
| enc | REVIEW | 56 行, 最終 cnt=('0', '0', '0')（方向は目視確認） |
| enc spin | REVIEW | +duty rate=[0.0, 0.0, 0.0] / -duty rate=[-1.93, 0.0, -1.98] |
| deadzone | REVIEW | 不感帯推定 |duty| w0=0.280 w1=0.088 w2=0.272（最大値超過時は不動） |
| omni | PASS | 93 行, 最終 w=[0.0, 1.84, -1.84] |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
