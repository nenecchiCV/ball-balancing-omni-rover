//! テレメトリ IPC ペイロード。
//! ロボットイベント（`protocol::RobotEvent`）から UI 向け型への変換を担う。

use serde::Serialize;
use specta::Type;

use crate::protocol::message::RobotEvent;

/// EVT_POSE（20 Hz 級）。ワールド座標はロボット起動点原点。
#[derive(Debug, Clone, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct PoseTelemetry {
    pub t_ms: u32,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub vx: f32,
    pub vy: f32,
    pub yaw_rate: f32,
}

/// EVT_STATUS。
#[derive(Debug, Clone, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct StatusTelemetry {
    pub op_state: u8,
    pub mode: u8,
    pub enable: u8,
    pub estop: u8,
    pub imu_ok: u8,
    pub calib_state: u8,
}

impl From<&RobotEvent> for Option<PoseTelemetry> {
    fn from(ev: &RobotEvent) -> Self {
        match ev {
            RobotEvent::Pose {
                t_ms,
                x,
                y,
                yaw,
                vx,
                vy,
                yaw_rate,
            } => Some(PoseTelemetry {
                t_ms: *t_ms,
                x: *x,
                y: *y,
                yaw: *yaw,
                vx: *vx,
                vy: *vy,
                yaw_rate: *yaw_rate,
            }),
            _ => None,
        }
    }
}

impl From<&RobotEvent> for Option<StatusTelemetry> {
    fn from(ev: &RobotEvent) -> Self {
        match ev {
            RobotEvent::Status {
                op_state,
                mode,
                enable,
                estop,
                imu_ok,
                calib_state,
            } => Some(StatusTelemetry {
                op_state: *op_state,
                mode: *mode,
                enable: *enable,
                estop: *estop,
                imu_ok: *imu_ok,
                calib_state: *calib_state,
            }),
            _ => None,
        }
    }
}
