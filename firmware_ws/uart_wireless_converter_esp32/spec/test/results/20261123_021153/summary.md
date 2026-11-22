# BLE シリアル通信テスト結果 20261123_021153

| テスト | 判定 | 備考 |
|---|---|---|
| scan | PASS | 9C:CC:01:18:DC:98 name=uwc-esp32c6 rssi=-70 |
| gatt | PASS | mtu=247 payload_max=244 |
| echo small | PASS | 14 B roundtrip rtt=29.9 ms recv=14 B |
| echo 256B | PASS | 256 B pattern rtt=103.3 ms recv=256 B |
| echo stream | PASS | 1024 B rtt=164.2 ms recv=1024 B ~49.9 kbps |
| reconnect | PASS | 再接続+エコー rtt=22.5 ms recv=16 B |

PASS=自動判定合格 / REVIEW=要目視 / FAIL=要調査
各テストの生ログは同ディレクトリの *.log を参照。
