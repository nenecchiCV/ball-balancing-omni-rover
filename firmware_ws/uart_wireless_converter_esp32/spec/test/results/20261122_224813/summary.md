# BLE シリアル通信テスト結果 20261122_224813

| テスト | 判定 | 備考 |
|---|---|---|
| scan | PASS | 9C:CC:01:18:DC:98 name=uwc-esp32c6 rssi=-72 |
| gatt | PASS | mtu=23 payload_max=20 |
| echo small | FAIL | 14 B roundtrip rtt=5002.6 ms recv=0 B |
| echo 256B | FAIL | 256 B pattern rtt=5038.9 ms recv=0 B |
| echo stream | FAIL | 1024 B rtt=15088.1 ms recv=0 B ~0.5 kbps |
| reconnect | FAIL | 切断後に再 advertising が確認できない |

PASS=自動判定合格 / REVIEW=要目視 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
