//! アプリケーション共通エラー。IPC 境界では `Result<T, AppError>` を返す。
//! シリアライズ形式は `{ kind, message }`（bindings.ts にも反映される）。

use serde::Serialize;
use specta::Type;

#[derive(Debug, thiserror::Error, Serialize, Type)]
#[serde(tag = "kind", content = "message", rename_all = "camelCase")]
pub enum AppError {
    #[error("BLE エラー: {0}")]
    Ble(String),
    #[error("未接続")]
    NotConnected,
    #[error("応答タイムアウト")]
    Timeout,
    #[error("応答形式が不正")]
    BadResponse,
    #[error("パラメータが不正: {0}")]
    InvalidParam(String),
    #[error("ロボットが拒否: {0}")]
    Rejected(String),
    #[error("送信失敗: {0}")]
    Send(String),
}

impl From<btleplug::Error> for AppError {
    fn from(e: btleplug::Error) -> Self {
        AppError::Ble(e.to_string())
    }
}
