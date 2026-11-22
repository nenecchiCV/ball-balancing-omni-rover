# オンボードテスト結果 20261122_230036

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=999 min_us=809 max_us=1005 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | FAIL | imu ログ行なし |
| enc | FAIL | enc ログ行なし |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
