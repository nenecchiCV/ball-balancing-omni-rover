# オンボードテスト結果 20261122_215539

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | FAIL | tick 報告行が見つからない |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | FAIL | imu ログ行なし |
| enc | FAIL | enc ログ行なし |
| enc spin | REVIEW | duty 正負双方の rate ログが不足 |
| deadzone | FAIL | dz,done が現れず掃引未完了 |
| omni | FAIL | omni ログ行なし |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
