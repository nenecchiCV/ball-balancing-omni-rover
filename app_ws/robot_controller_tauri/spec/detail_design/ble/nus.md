# nus

`btleplug` による BLE NUS クライアント。スキャン・接続・特性確保・送受信を担当し、上位（`robot::Session`）へはバイトストリームの `send` / 通知ストリームを提供する。Tauri 非依存。

## 公開型

```rust
#[serde(rename_all = "camelCase")]
pub struct DiscoveredDevice {
    pub name: String,
    pub address: String,
    pub rssi: Option<i16>,
}

pub struct NusLink { /* Peripheral, RX/TX 特性 */ }
```

## 公開関数

- `async fn scan(name_filter: &str, timeout: Duration) -> Result<Vec<DiscoveredDevice>, AppError>` — アダプタで BLE スキャンし、`name_filter` 部分一致を優先して一覧を返す。既定フィルタは `uwc`。
- `NusLink::connect(address: &str, scan_timeout: Duration) -> Result<NusLink, AppError>` — 指定アドレスのペリフェラルをアダプタ列挙で探し（未発見なら再スキャン）、接続→NUS サービス検出→TX 特性購読まで完了させる。
- `NusLink::notification_stream(&self) -> Result<impl Stream<Item = Vec<u8>>, AppError>` — TX 通知のバイト列ストリームを返す。`Session` のディスパッチャが消費する。
- `NusLink::send(&self, data: &[u8]) -> Result<(), AppError>` — RX 特性へ Write Without Response で送出する。
- `NusLink::is_connected(&self) -> bool` — 接続確認。
- `NusLink::disconnect(&self) -> Result<(), AppError>` — 切断する。

## 内部状態

- `Peripheral` ハンドル・RX/TX `Characteristic`。接続スコープで保持し、`disconnect` で解放する。

## 備考

- UUID は `environment.md` の NUS 定数（Service `6E400001-…`、RX `6E400002-…`、TX `6E400003-…`）を `uuid::Uuid::from_u128` で保持する。
- 接続喪失は通知ストリーム終端または `is_connected()` 監視で検出し、`Session` が `link` バスへ `Disconnected` を発行する。
