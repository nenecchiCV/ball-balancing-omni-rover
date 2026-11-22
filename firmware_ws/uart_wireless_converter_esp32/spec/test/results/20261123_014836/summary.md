# BLE シリアル通信テスト結果 20261123_014836

| テスト | 判定 | 備考 |
|---|---|---|
| scan | PASS | 9C:CC:01:18:DC:98 name=uwc-esp32c6 rssi=-84 |
| gatt | PASS | mtu=23 payload_max=20 |
| echo small | PASS | 14 B roundtrip rtt=19.8 ms recv=14 B |
| echo 256B | PASS | 256 B pattern rtt=73.9 ms recv=256 B |
| echo stream | PASS | 1024 B rtt=214.0 ms recv=1024 B ~38.3 kbps |
| reconnect | FAIL | 切断後に再 advertising が確認できない |

PASS=自動判定合格 / REVIEW=要目視 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
