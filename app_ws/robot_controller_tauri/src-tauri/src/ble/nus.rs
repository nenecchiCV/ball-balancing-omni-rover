//! BLE NUS クライアント。
//! `uart_wireless_converter_esp32` の NUS 相当 GATT へ接続し、
//! バイトストリームの送受信を上位へ提供する。Tauri には依存しない。

use std::pin::Pin;
use std::time::Duration;

use btleplug::api::{
    Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::{Stream, StreamExt};
use serde::Serialize;
use specta::Type;
use uuid::Uuid;

use crate::error::AppError;

// environment.md の NUS UUID 定数。
const NUS_SERVICE: Uuid = Uuid::from_u128(0x6E400001_B5A3_F393_E0A9_E50E24DCCA9E);
const NUS_RX_CHAR: Uuid = Uuid::from_u128(0x6E400002_B5A3_F393_E0A9_E50E24DCCA9E);
const NUS_TX_CHAR: Uuid = Uuid::from_u128(0x6E400003_B5A3_F393_E0A9_E50E24DCCA9E);

/// 既定のデバイス名フィルタ（`uwc-esp32c6`）。
pub const DEFAULT_NAME_FILTER: &str = "uwc";

/// 接続・通知購読の個別タイムアウト。CoreBluetooth の connectPeripheral は
/// 接続不能な相手に対して長時間 pending し続けるため、必ず時間制限を設ける。
const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);
const SUBSCRIBE_TIMEOUT: Duration = Duration::from_secs(5);

#[derive(Debug, Clone, Serialize, Type)]
pub struct DiscoveredDevice {
    pub name: String,
    pub address: String,
    pub rssi: Option<i16>,
}

pub struct NusLink {
    peripheral: Peripheral,
    rx_char: Characteristic,
}

async fn adapter() -> Result<Adapter, AppError> {
    let manager = Manager::new().await?;
    manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or_else(|| AppError::Ble("BLE アダプタが見つからない".to_string()))
}

/// `timeout` だけスキャンし、発見デバイスを返す。`name_filter` 部分一致を先頭に並べる。
pub async fn scan(
    name_filter: &str,
    timeout: Duration,
) -> Result<Vec<DiscoveredDevice>, AppError> {
    let adapter = adapter().await?;
    adapter.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(timeout).await;
    let peripherals = adapter.peripherals().await?;
    let _ = adapter.stop_scan().await;

    let filter = name_filter.to_lowercase();
    let mut devices = Vec::new();
    for p in peripherals {
        let props = match p.properties().await {
            Ok(Some(props)) => props,
            _ => continue,
        };
        let name = props.local_name.unwrap_or_default();
        if name.is_empty() {
            continue;
        }
        devices.push(DiscoveredDevice {
            name,
            // Apple 系では Peripheral::address() が常にゼロアドレスを返すため、
            // 識別子にはプラットフォーム固有 ID（Apple: UUID / その他: BDAddr）を使う。
            address: p.id().to_string(),
            rssi: props.rssi,
        });
    }
    devices.sort_by_key(|d| {
        if filter.is_empty() || d.name.to_lowercase().contains(&filter) {
            0
        } else {
            1
        }
    });
    Ok(devices)
}

impl NusLink {
    /// `address`（`DiscoveredDevice.address`）のデバイスへ接続し、RX/TX 特性確保と通知購読まで行う。
    pub async fn connect(address: &str, scan_timeout: Duration) -> Result<NusLink, AppError> {
        let adapter = adapter().await?;
        // Apple 系では address() が常にゼロアドレスのため id()（UUID 文字列）で照合する。
        // 他プラットフォームではどちらも MAC 文字列になるため、両方を比較対象にする。
        let matches = |p: &Peripheral| {
            p.id().to_string() == address || p.address().to_string() == address
        };

        // 0.11 系ではアドレスからの直接取得 API が無いため、既知一覧を走査し、
        // 未発見ならスキャン後に再検索する。
        let mut found = adapter.peripherals().await?.into_iter().find(matches);
        if found.is_none() {
            adapter.start_scan(ScanFilter::default()).await?;
            tokio::time::sleep(scan_timeout).await;
            let _ = adapter.stop_scan().await;
            found = adapter.peripherals().await?.into_iter().find(matches);
        }
        let peripheral = found
            .ok_or_else(|| AppError::Ble(format!("デバイスが見つからない: {address}")))?;

        tokio::time::timeout(CONNECT_TIMEOUT, peripheral.connect())
            .await
            .map_err(|_| AppError::Ble("BLE 接続がタイムアウトしました".to_string()))??;
        peripheral.discover_services().await?;

        let chars = peripheral.characteristics();
        let rx_char = chars
            .iter()
            .find(|c| c.uuid == NUS_RX_CHAR && c.service_uuid == NUS_SERVICE)
            .cloned()
            .ok_or_else(|| AppError::Ble("NUS RX 特性が見つからない".to_string()))?;
        let tx_char = chars
            .iter()
            .find(|c| c.uuid == NUS_TX_CHAR && c.service_uuid == NUS_SERVICE)
            .cloned()
            .ok_or_else(|| AppError::Ble("NUS TX 特性が見つからない".to_string()))?;

        tokio::time::timeout(SUBSCRIBE_TIMEOUT, peripheral.subscribe(&tx_char))
            .await
            .map_err(|_| AppError::Ble("通知購読がタイムアウトしました".to_string()))??;

        Ok(NusLink {
            peripheral,
            rx_char,
        })
    }

    /// TX 通知を受け取るバイト列ストリーム。接続ごとに 1 回だけ呼ぶ。
    pub async fn notification_stream(
        &self,
    ) -> Result<Pin<Box<dyn Stream<Item = Vec<u8>> + Send>>, AppError> {
        let stream = self.peripheral.notifications().await?;
        Ok(Box::pin(stream.filter_map(|n| async move {
            if n.uuid == NUS_TX_CHAR {
                Some(n.value)
            } else {
                None
            }
        })))
    }

    /// NUS RX 特性へ Write Without Response で送出する。
    pub async fn send(&self, data: &[u8]) -> Result<(), AppError> {
        self.peripheral
            .write(&self.rx_char, data, WriteType::WithoutResponse)
            .await?;
        Ok(())
    }

    pub async fn is_connected(&self) -> bool {
        self.peripheral.is_connected().await.unwrap_or(false)
    }

    pub async fn disconnect(&self) -> Result<(), AppError> {
        self.peripheral.disconnect().await?;
        Ok(())
    }
}
