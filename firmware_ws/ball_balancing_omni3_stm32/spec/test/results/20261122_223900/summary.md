# オンボードテスト結果 20261122_223900

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=999 min_us=915 max_us=1000 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 76 行, valid 率 100% |
| enc | REVIEW | 55 行, 最終 cnt=('39440', '40134', '29892')（方向は目視確認） |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
