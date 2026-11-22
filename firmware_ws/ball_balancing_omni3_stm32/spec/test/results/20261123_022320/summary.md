# オンボードテスト結果 20261123_022320

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=998 min_us=84 max_us=1011 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 77 行, valid 率 100% |
| enc | REVIEW | 56 行, 最終 cnt=('0', '0', '0')（方向は目視確認） |
| enc spin | REVIEW | duty 正負双方の rate ログが不足 |
| deadzone | REVIEW | 不感帯推定 |duty| w0=0.272 w1=0.212 w2=0.206（最大値超過時は不動） |
| omni | PASS | 93 行, 最終 w=[0.0, 1.84, -1.84] |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
