//! ログ IPC ペイロード。ロボット `EVT_LOG` とアプリ内イベントを同じ型で UI へ流す。

use serde::Serialize;
use specta::Type;

#[derive(Debug, Clone, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub enum LogSource {
    Robot,
    App,
}

/// `level`: 0=DEBUG, 1=INFO, 2=WARN, 3=ERROR（`robot_link_protocol.md` EVT_LOG と共通）。
#[derive(Debug, Clone, Serialize, Type)]
pub struct LogMessage {
    pub level: u8,
    pub text: String,
    pub source: LogSource,
}
