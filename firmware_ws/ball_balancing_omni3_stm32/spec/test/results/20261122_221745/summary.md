# オンボードテスト結果 20261122_221745

- ポート: COM4

| テスト | 判定 | 備考 |
|---|---|---|
| tick | FAIL | tick 報告行が見つからない |
| estop | REVIEW | requested=1 が未確認（ボタン操作が要検証） |
| imu | PASS | imu ログ 77 行, valid 率 100% |
| enc | REVIEW | 56 行, 最終 cnt=('0', '0', '0')（方向は目視確認） |
| enc spin | REVIEW | +duty rate=[0.0, 0.0, 0.0] / -duty rate=[0.0, 0.0, -1.8] |
| deadzone | REVIEW | 不感帯推定 |duty| w0=0.196 w1=0.184 w2=0.308（最大値超過時は不動） |
| omni | PASS | 93 行, 最終 w=[0.0, 2.76, -1.84] |

PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
