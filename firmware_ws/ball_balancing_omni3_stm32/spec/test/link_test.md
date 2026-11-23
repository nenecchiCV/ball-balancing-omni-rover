# link_test — ロボットリンクプロトコル応答

## 目的

`uart_echo_test` が USART1 の生バイト折返しを確認するのに対し、本テストは
ロボットリンクプロトコルのフレーム復号・応答符号化を実機構成で検証する。
アプリ側のテストコマンド送受信機能（`robot_controller_tauri` FR-08）と
対にして用い、本番ファームの機能実装を待たずにリンク疎通を確認する。

## 前提

- テストファームウェア書き込み済み。
- USART1 経由の BLE リンク疎通は `test echo` で確認済みであること。
- `uart_echo_test` は `init()` 時点から USART1 を占有するため、`test link`
  実行中は echo を無効化する排他利用とする（排他方式は詳細設計で定める）。

## 手順

```text
uart:~$ test link
link test: protocol responder started
link t=1000 rx_frames=0 crc_drops=0 tx_frames=0
```

1. `test link` でプロトコル応答を開始する。
2. アプリのテストページ（`/test`）から `PING`・`GET_STATUS`・`TEST_ECHO` を
   送信し、応答が返ることを確認する。
3. `test stop` で応答を停止する。

## 受理リクエストと応答

| リクエスト | 応答 |
|---|---|
| `PING` | `PONG`。`robot_caps` はテストビルドを示す bit3 を立て、本番機能ビットは立てない |
| `GET_STATUS` | `STATUS`。`op_state` は常に `0`（待機）、健全性フィールドは実測値（`imu_ok` 等）を返す |
| `TEST_ECHO` | `TEST_ECHO_RESP`（受信ペイロードをそのまま返す） |
| 上記以外 | `ACK` result=`UNSUPPORTED` |

## ログ形式

```text
link t=<ms> rx_frames=<n> crc_drops=<n> rx_overruns=<n> tx_frames=<n>
```

- `rx_frames`: CRC 整合した受理フレーム数。
- `crc_drops`: CRC 不一致・LEN 異常で破棄したフレーム候補数。
- `rx_overruns`: RX リング溢れで失ったバイト数。
- `tx_frames`: 送出したフレーム数。
- 1 Hz で `link` 接頭辞付き `debug_uart` 出力とする。

## 合否基準

- アプリ側で `PING`/`PONG`・`GET_STATUS`/`STATUS`・`TEST_ECHO`/`TEST_ECHO_RESP`
  の往復が成立し、エコーペイロードが一致すること。
- 不正フレーム投入時を除き `crc_drops` が増えないこと。

## 対応コンポーネント

- `tests/onboard/src/link_test/`（`peripheral/link_uart` + `common/link_protocol` を再利用）
