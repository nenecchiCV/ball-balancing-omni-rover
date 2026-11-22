# オンボードテスト結果 20261122_224712

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | PASS | tick n=1000 mean_us=998 min_us=270 max_us=1000 missed=0 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | FAIL | imu ログ行なし |
| enc | FAIL | enc ログ行なし |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
