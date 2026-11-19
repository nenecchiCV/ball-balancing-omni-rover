# ロボットリンクプロトコル（API 仮定義）

`robot_controller_tauri`（セントラル）と `ball_balancing_omni3_stm32`（プロトコル終端）の間のアプリケーションプロトコルを定義する。転送は `uart_wireless_converter_esp32` の BLE NUS 透過バイトストリーム上で行い、変換器は内容を解釈しない。

- ステータス: **仮定義（rev A）**。ファームウェア側実装（`ball_balancing_omni3_stm32` FR-09〜FR-11）と実機検証で確定する。
- プロトコルバージョン: `1`

## 設計方針

- リクエスト/レスポンス型を基本とし、周期送出が必要なデータは `SUBSCRIBE`/`UNSUBSCRIBE` による購読型とする。
- マルチバイト数値はすべてリトルエンディアン、小数は IEEE 754 binary32（`f32`）とする。
- フレーム単位で CRC16 を持ち、UART/BLE 経由のバイト欠落・化けを検出して破棄する。
- 座標系: ロボット起動地点を原点・起動時の機体 +X 方向をワールド +X とする 2 次元平面。x, y [m]、yaw [rad]（+X から反時計回り）。速度指令はワールド座標系の並進速度 vx, vy [m/s] とヨーレート yaw_rate [rad/s]（機体 yaw の時間微分）。

## フレーム形式

```
+------+------+-----+-----+==========+=========+
| SYNC | TYPE | SEQ | LEN | PAYLOAD  | CRC16   |
| 0xA5 | u8   | u8  | u8  | LEN バイト| u16 LE  |
+------+------+-----+-----+==========+=========+
```

- `SYNC`: 固定 `0xA5`。受信側は `0xA5` 走査から再同期する。
- `TYPE`: メッセージ種別（下表）。`0x01-0x3F` をリクエスト、`0x80` を ACK、`0x81-0x83` をデータ応答、`0x90-0xAF` をイベント（購読送出）とする。
- `SEQ`: 送信側がリクエストごとにインクリメントする識別子。ACK は同じ `SEQ` を返し、応答対応付けに使う。イベントは `0` 固定。
- `LEN`: ペイロード長（0–255）。`PAYLOAD` の直後に CRC16-CCITT-FALSE（poly `0x1021`、init `0xFFFF`）を `TYPE` 先頭から `PAYLOAD` 末尾までの区間に対して付与する。
- 受信側はストリームから `0xA5` を探索し、`LEN` と CRC が整合したフレームのみ受理する。SYNC 探索は CRC 不一致時に 1 バイト戻って継続する。

## メッセージ一覧

### リクエスト（アプリ → ロボット）

| TYPE | 名称 | ペイロード | 応答 | 説明 |
|---|---|---|---|---|
| `0x01` | `PING` | なし | `0x81 PONG` | 死活監視とプロトコルバージョン照合 |
| `0x10` | `SET_VELOCITY` | `vx f32, vy f32, yaw_rate f32`（12 B） | `0x80 ACK` | 速度指令モードで適用する指令値。値はロボット側で連成リミット・キャップを通過する |
| `0x11` | `SET_POSE_TARGET` | `x f32, y f32, speed_limit f32`（12 B） | `0x80 ACK` | 位置制御モードの目標。`speed_limit` は走行速度上限 [m/s] |
| `0x12` | `STOP` | なし | `0x80 ACK` | 現在モードを維持したまま運動指令をゼロ化する（位置制御中は目標を現在位置に置き換える） |
| `0x13` | `SET_CONTROL_MODE` | `mode u8`（1 B） | `0x80 ACK` | `0`=スタンバイ、`1`=速度指令、`2`=位置制御。指令系リクエスト受理の前提モードを指定する |
| `0x14` | `RESET_ODOMETRY` | なし | `0x80 ACK` | ワールド原点を現在位置へ再設定し、(x, y, yaw) をゼロ化する |
| `0x20` | `SUBSCRIBE` | `event_type u8, rate_hz u8`（2 B） | `0x80 ACK` | イベント購読開始。`rate_hz` は `EVT_POSE` に適用（1–50、`0` はイベント発生時のみの意味で `EVT_LOG`/`EVT_STATUS` 向け） |
| `0x21` | `UNSUBSCRIBE` | `event_type u8`（1 B） | `0x80 ACK` | イベント購読停止 |
| `0x30` | `GET_STATUS` | なし | `0x82 STATUS` | ロボット状態の単発取得 |

### 応答・イベント（ロボット → アプリ）

| TYPE | 名称 | ペイロード | 説明 |
|---|---|---|---|
| `0x80` | `ACK` | `req_type u8, seq u8, result u8`（3 B） | リクエスト受理結果。`result`: `0`=OK、`1`=INVALID_PARAM、`2`=REJECTED_STATE（モード不整合等）、`3`=NOT_READY（推定・制御の準備未完）、`4`=UNSUPPORTED |
| `0x81` | `PONG` | `seq u8, proto_ver u8, robot_caps u8`（3 B） | `PING` 応答。`robot_caps` ビット: bit0=位置制御対応、bit1=オドメトリ送出対応、bit2=ログ送出対応 |
| `0x82` | `STATUS` | `seq u8, mode u8, enable u8, estop u8, imu_ok u8, uptime_ms u32`（9 B） | `GET_STATUS` 応答。`mode` は `SET_CONTROL_MODE` と同じ定義 |
| `0x90` | `EVT_POSE` | `t_ms u32, x f32, y f32, yaw f32, vx f32, vy f32, yaw_rate f32`（28 B） | 自己位置推定結果。`SUBSCRIBE` で指定したレートで送出 |
| `0x91` | `EVT_LOG` | `level u8, text utf8[LEN-1]` | ログメッセージ。`level`: `0`=DEBUG、`1`=INFO、`2`=WARN、`3`=ERROR |
| `0x92` | `EVT_STATUS` | `mode u8, enable u8, estop u8, imu_ok u8`（4 B） | 状態変化時の送出。購読時は変化検出 + 10 Hz 上限のハートビートで送出する |

## シーケンス規則

- リクエストは応答（ACK またはデータ応答）か 1 s タイムアウトで完了とする。同種リクエストの未完了多重送信は行わない。
- `SET_VELOCITY` は高頻度送出のため ACK を省略可能とする運用を許容する（ファーム実装では ACK を返すが、アプリ側は応答を待たずに次の指令を送ってよい）。
- 購読状態は BLE 切断・UART リンク断絶でロボット側がリセットする。アプリは再接続後に `SUBSCRIBE` を再送する。
- `EVT_POSE` の送出レート変更は再度 `SUBSCRIBE` を送ることで上書きする。

## 安全規則（ロボット側義務）

- 速度指令ウォッチドッグ: 最後の `SET_VELOCITY` 受信から 500 ms 経過で指令をゼロとする。アプリ側は 20 Hz 送出を維持し、ゼロ指令を送らない限り停止しない。
- BLE 切断または UART アイドル 1 s 超過で指令をゼロとする（リンク断絶のフェイルセーフ）。
- `SET_POSE_TARGET` の `speed_limit` はロボット側 `max_speed` でクランプする。
- 位置制御モードは自己位置推定が健全（`imu_ok` かつ推定 qualification 成立）なときのみ `result=0` を返す。

## ロボット側実装要求（トレース先）

本プロトコルを成立させるために `ball_balancing_omni3_stm32` 側で必要な実装は以下にトレースする。

| 要求 | firmware 側要求 ID | 内容 |
|---|---|---|
| USART1 受付・フレーム処理 | FR-09 | USART1（115200 8N1）の受信・フレームデコード・応答送出。受信指令を `command_profile` より優先する UART 上書き経路（FR-07 の予定経路） |
| 位置制御 | FR-10 | ワールド位置 (x, y) の推定（オドメトリ積分）と、目標位置への速度指令生成（位置外ループ） |
| オドメトリ・ログ送出 | FR-11 | `EVT_POSE` 周期送出、`EVT_LOG` 送出、`EVT_STATUS` 変化送出 |

## 未確定事項

- `EVT_TELEMETRY`（詳細テレメトリ: euler、輪速、トルク等、デバッグ CSV 相当）の追加は実機帯域確認後に検討する。`EVT_POSE` に統合するか別 TYPE とするかは未確定。
- `SET_POSE_TARGET` に目標 yaw を含めるかはロボット側位置制御の設計確定時に見直す。
- 大きなペイロード（ファームウェア更新等）は本プロトコルの範囲外とし、必要時に別 TYPE で拡張する。
