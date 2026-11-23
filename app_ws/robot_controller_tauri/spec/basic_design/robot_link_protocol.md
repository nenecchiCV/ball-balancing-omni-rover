# ロボットリンクプロトコル

`robot_controller_tauri`（セントラル）と `ball_balancing_omni3_stm32`（プロトコル終端）の間のアプリケーションプロトコルを定義する。転送は `uart_wireless_converter_esp32` の BLE NUS 透過バイトストリーム上で行い、変換器は内容を解釈しない。

- ステータス: rev A は両端実装済み（`link_protocol`/`link_uart`/`link_session`/`odometry`/`position_controller`）。rev B（動作状態管理・キャリブレーション・初期位置設定・`TEST_ECHO`・イベントバッチ送出）は要求確定済み・実装は未着手。ファーム側要求は FR-09〜FR-14 にトレースする。
- プロトコルバージョン: `2`（rev A = `1`）

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
- `TYPE`: メッセージ種別（下表）。`0x01-0x3F` をリクエスト、`0x80` を ACK、`0x81-0x84` をデータ応答、`0x90-0xAF` をイベント（購読送出）とする。
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
| `0x12` | `STOP` | なし | `0x80 ACK` | 現在モードを維持したまま運動指令をゼロ化する（位置制御中は目標を現在位置に置き換える）。`STOP_OPERATION` とは異なり動作中状態は維持する |
| `0x13` | `SET_CONTROL_MODE` | `mode u8`（1 B） | `0x80 ACK` | `0`=指令なし（その場保持）、`1`=速度指令、`2`=位置制御。指令系リクエスト受理の前提モードを指定する。動作中のみ受理 |
| `0x14` | `RESET_ODOMETRY` | なし | `0x80 ACK` | ワールド原点を現在位置へ再設定し、(x, y, yaw) をゼロ化する。受理時に位置目標を解除する |
| `0x15` | `START_OPERATION` | なし | `0x80 ACK` | 待機 → 動作中へ遷移する。受理条件: `imu_ok`・E-Stop 解除・非 `fault`・バイアス資格成立・姿勢が `fallen_tilt` 未満。不成立時は `NOT_READY`/`REJECTED_STATE` |
| `0x16` | `STOP_OPERATION` | なし | `0x80 ACK` | 動作中 → 待機へ遷移し、モータ駆動を即時遮断する。常時受理 |
| `0x17` | `SET_POSE` | `x f32, y f32, yaw f32`（12 B） | `0x80 ACK` | 現在のオドメトリ姿勢・位置を指定値へ設定する（所定位置へのセッティング）。待機中のみ受理、受理時に位置目標を解除する |
| `0x18` | `CALIBRATE_IMU` | なし | `0x80 ACK` + `0x83 CALIB_RESULT` | ジャイロバイアスの再学習を開始する。待機中のみ受理。受理時に ACK を返し、完了・失敗時に `CALIB_RESULT` を即時送出する |
| `0x20` | `SUBSCRIBE` | `event_type u8, rate_hz u8`（2 B） | `0x80 ACK` | イベント購読開始。`rate_hz` は `EVT_POSE` に適用（1–50、`0` はイベント発生時のみの意味で `EVT_LOG`/`EVT_STATUS` 向け） |
| `0x21` | `UNSUBSCRIBE` | `event_type u8`（1 B） | `0x80 ACK` | イベント購読停止 |
| `0x30` | `GET_STATUS` | なし | `0x82 STATUS` | ロボット状態の単発取得 |
| `0x31` | `TEST_ECHO` | 任意（0–60 B） | `0x84 TEST_ECHO_RESP` | リンク疎通・往復検証用のペイロードエコー。本番ファームとテストファーム（`link_test`）の双方で実装する |

### 応答・イベント（ロボット → アプリ）

| TYPE | 名称 | ペイロード | 説明 |
|---|---|---|---|
| `0x80` | `ACK` | `req_type u8, seq u8, result u8`（3 B） | リクエスト受理結果。`result`: `0`=OK、`1`=INVALID_PARAM、`2`=REJECTED_STATE（モード・動作状態の不整合等）、`3`=NOT_READY（推定・制御の準備未完）、`4`=UNSUPPORTED |
| `0x81` | `PONG` | `seq u8, proto_ver u8, robot_caps u8`（3 B） | `PING` 応答。`robot_caps` ビット: bit0=位置制御対応、bit1=オドメトリ送出対応、bit2=ログ送出対応、bit3=テストビルド、bit4=運転管理対応（`START_OPERATION`/`STOP_OPERATION`/`CALIBRATE_IMU`/`SET_POSE`） |
| `0x82` | `STATUS` | `seq u8, op_state u8, mode u8, enable u8, estop u8, imu_ok u8, calib_state u8, uptime_ms u32`（11 B） | `GET_STATUS` 応答。`op_state`: `0`=待機、`1`=動作中、`2`=終端（fault）。`mode` は `SET_CONTROL_MODE` と同じ定義。`calib_state`: `0`=未実施、`1`=実施中、`2`=完了、`3`=失敗 |
| `0x83` | `CALIB_RESULT` | `seq u8, result u8, gyro_bias_z f32`（6 B） | `CALIBRATE_IMU` の完了報告（単発応答として即時送出）。`result`: `0`=成功（静止資格成立）、`1`=失敗（静止未検出・タイムアウト）、`2`=失敗（IMU 異常） |
| `0x84` | `TEST_ECHO_RESP` | `seq u8, data u8[LEN-1]` | `TEST_ECHO` ペイロードのエコー |
| `0x90` | `EVT_POSE` | `t_ms u32, x f32, y f32, yaw f32, vx f32, vy f32, yaw_rate f32`（28 B） | 自己位置推定結果。`SUBSCRIBE` で指定したレートで送出 |
| `0x91` | `EVT_LOG` | `level u8, text utf8[LEN-1]` | ログメッセージ。`level`: `0`=DEBUG、`1`=INFO、`2`=WARN、`3`=ERROR |
| `0x92` | `EVT_STATUS` | `op_state u8, mode u8, enable u8, estop u8, imu_ok u8, calib_state u8`（6 B） | 状態変化時の送出。購読時は変化検出 + 10 Hz 上限のハートビートで送出する。フィールド定義は `STATUS` と同じ |

## 送出規則（ロボット側）

- **単発応答**（`ACK`・`PONG`・`STATUS`・`CALIB_RESULT`・`TEST_ECHO_RESP`）は受信処理後、即時に送出する。
- **購読イベント**（`EVT_*`）はスループット向上のためロボット側送出バッファで集約し、既定 100 ms 周期（`link.event_batch_ms`、実効帯域に応じて調整可能）でまとめて送出する。1 回のフラッシュ内のフレーム列はそのままストリームへ連結されるため、アプリ側は従来どおり連続ストリームとして復号する。
- 同一窓内に複数発生したイベントは `EVT_POSE`・`EVT_STATUS` を最新値のみ、`EVT_LOG` を発生順に保持して送出する。窓内の `EVT_POSE` 実効レートは `min(rate_hz, 1000/event_batch_ms)` となる。
- 滞留量が BLE 通知 1 回分相当のしきい値へ達した場合は周期を待たず早期フラッシュしてよい。バッファ容量超過は破棄してロボット側で計数する。

## シーケンス規則

- リクエストは応答（ACK またはデータ応答）か 1 s タイムアウトで完了とする。同種リクエストの未完了多重送信は行わない。
- `CALIBRATE_IMU` のみ 2 段応答とし、受理 ACK（`seq` エコー）の後に完了報告 `CALIB_RESULT`（同じ `seq`）が来る。アプリ側は `CALIB_RESULT` を最大 `calib_timeout_ms` 程度待ち、超過時は `calib_state` を `GET_STATUS` で確認する。
- `SET_VELOCITY` は高頻度送出のため ACK を省略可能とする運用を許容する（ファーム実装では ACK を返すが、アプリ側は応答を待たずに次の指令を送ってよい）。
- 購読状態は BLE 切断・UART リンク断絶でロボット側がリセットする。アプリは再接続後に `SUBSCRIBE` を再送する。
- `EVT_POSE` の送出レート変更は再度 `SUBSCRIBE` を送ることで上書きする。
- 待機中は運動系リクエスト（`SET_VELOCITY`・`SET_POSE_TARGET`・`SET_CONTROL_MODE`）を `REJECTED_STATE` で拒否する。セットアップ系（`PING`・`GET_STATUS`・`SUBSCRIBE`/`UNSUBSCRIBE`・`SET_POSE`・`RESET_ODOMETRY`・`CALIBRATE_IMU`・`START_OPERATION`・`TEST_ECHO`）は待機中も受理する。
- 推奨運転シーケンス: `CALIBRATE_IMU` → `CALIB_RESULT` 成功 → `SET_POSE`（所定位置）→ `START_OPERATION` → `SET_CONTROL_MODE` → 運転 → `STOP`/`STOP_OPERATION`。

## 安全規則（ロボット側義務）

- 速度指令ウォッチドッグ: 最後の `SET_VELOCITY` 受信から 500 ms 経過で指令をゼロとする。アプリ側は 20 Hz 送出を維持し、ゼロ指令を送らない限り停止しない。
- BLE 切断または UART アイドル 1 s 超過で指令をゼロとする（リンク断絶のフェイルセーフ）。動作中はその場保持を継続し、待機へは遷移しない（転倒防止）。
- `SET_POSE_TARGET` の `speed_limit` はロボット側 `max_speed` でクランプする。
- 位置制御モードは自己位置推定が健全（`imu_ok` かつ推定 qualification 成立）なときのみ `result=0` を返す。
- `START_OPERATION` は `imu_ok`・E-Stop 解除・非 `fault`・バイアス資格成立・姿勢が `fallen_tilt` 未満のときのみ `result=0` を返す。
- 動作中の E-Stop・転倒（`fallen_tilt` 超過）・IMU 異常は終端（`op_state=2`）へ遷移し、全タスクのトルク出力をゼロに固定する。復帰はリセットのみ。待機中のこれらの事象は `START_OPERATION` の拒否条件となる。

## ロボット側実装要求（トレース先）

本プロトコルを成立させるために `ball_balancing_omni3_stm32` 側で必要な実装は以下にトレースする。

| 要求 | firmware 側要求 ID | 内容 |
|---|---|---|
| USART1 受付・フレーム処理 | FR-09 | USART1（115200 8N1）の受信・フレームデコード・応答送出。受信指令を `command_profile` より優先する UART 上書き経路（FR-07 の予定経路） |
| 位置制御 | FR-10 | ワールド位置 (x, y) の推定（オドメトリ積分）と、目標位置への速度指令生成（位置外ループ） |
| オドメトリ・ログ送出 | FR-11 | `EVT_POSE` 周期送出、`EVT_LOG` 送出、`EVT_STATUS` 変化送出 |
| 動作状態管理 | FR-12 | 待機 ⇔ 動作中の遷移、モータ駆動ゲート、`START_OPERATION`/`STOP_OPERATION` 受理判定 |
| キャリブレーション・初期位置 | FR-13 | `CALIBRATE_IMU`/`CALIB_RESULT`、`SET_POSE` の受理と状態報告 |
| イベントバッチ送出 | FR-14 | 購読イベントの周期集約送出 |

## 未確定事項

- `EVT_TELEMETRY`（詳細テレメトリ: euler、輪速、トルク等、デバッグ CSV 相当）の追加は実機帯域確認後に検討する。バッチ送出の導入により帯域見通しは改善したが、`EVT_POSE` に統合するか別 TYPE とするかは未確定。
- `SET_POSE_TARGET` に目標 yaw を含めるかはロボット側位置制御の設計確定時に見直す。
- `fault`（`op_state=2`）からの遠隔復帰手段（ソフトリセット等）は未定義。現状は物理リセットを前提とし、必要時に別 TYPE で拡張する。
- 大きなペイロード（ファームウェア更新等）は本プロトコルの範囲外とし、必要時に別 TYPE で拡張する。
