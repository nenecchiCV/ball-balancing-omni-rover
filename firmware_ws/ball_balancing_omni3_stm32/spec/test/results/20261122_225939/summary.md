# オンボードテスト結果 20261122_225939

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=998 min_us=234 max_us=1013 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 77 行, valid 率 100% |
| enc | REVIEW | 56 行, 最終 cnt=('0', '0', '0')（方向は目視確認） |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
