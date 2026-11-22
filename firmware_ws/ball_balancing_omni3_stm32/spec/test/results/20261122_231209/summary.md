# オンボードテスト結果 20261122_231209

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | FAIL | tick n=1000 mean_us=4270701 min_us=91 max_us=4269703782 missed=4269702 |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 77 行, valid 率 100% |
| enc | REVIEW | 56 行, 最終 cnt=('0', '0', '0')（方向は目視確認） |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
