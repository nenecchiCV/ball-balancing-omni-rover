# テスト計画

`robot_controller_tauri` の検証方針を定義する。実機検証は `ball_balancing_omni3_stm32` 側 FR-09〜FR-11 の実装完了後に行う。

## 検証レベル

| レベル | 対象 | 方法 |
|---|---|---|
| ホスト検証 | フレーム符号化/復号、送信値クランプ | Rust 側 `#[cfg(test)]` 単体テスト（Tauri 非依存モジュール: `protocol`/`ble`/`robot`） |
| 型検査 | 全ソース | `npx tsc --noEmit` と `cargo check`（NFR-04） |
| 実機検証 | 各 FR の端到端動作 | 実ロボット + ESP32-C6 との BLE 接続確認 |

## ホスト検証項目

- `frame`: `encode`→`push` のラウンドトリップ、分割到着、CRC 不一致破棄、`0xA5` 再同期をテストする。
- `message`: 全 Request/Response/Event のペイロードレイアウトが `robot_link_protocol.md` と一致することをテストする。
- 送信値のクランプは `robot::session` 送信部の純粋関数として切り出してテストする。

## 実機検証項目（合否基準）

| 項目 | 手順 | 合否基準 |
|---|---|---|
| 接続 | `/connect` でスキャン→接続 | `uwc-esp32c6` が見つかり、接続後 `PING` が `proto_ver=1` を返す |
| RC | `/rc` でスティック操作 | ロボットが指令速度で移動し、スティック解放で停止する。無操作 500 ms 後も停止を維持する |
| 位置制御 | `/position` で (x, y, speed) 送信 | ロボットが目標へ移動し、`EVT_POSE` の残距離が収束する |
| オドメトリ | `/odometry` で購読 | 起動点原点で位置が更新され、`RESET_ODOMETRY` で原点が再設定される |
| ログ | `/logs` で購読 | ロボットの `EVT_LOG` がレベル付きで表示される |
| 異常系 | BLE 切断・ESP32 電源断 | `watchLinkState` が `disconnected` に遷移し、ロボットが指令をゼロ化する |
