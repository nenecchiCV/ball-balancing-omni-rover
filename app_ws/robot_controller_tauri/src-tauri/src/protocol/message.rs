//! メッセージ定義 — robot_link_protocol.md「メッセージ一覧」準拠。

use super::frame::Frame;

pub mod msg_type {
    // リクエスト（アプリ→ロボット）
    pub const PING: u8 = 0x01;
    pub const SET_VELOCITY: u8 = 0x10;
    pub const SET_POSE_TARGET: u8 = 0x11;
    pub const STOP: u8 = 0x12;
    pub const SET_CONTROL_MODE: u8 = 0x13;
    pub const RESET_ODOMETRY: u8 = 0x14;
    pub const SUBSCRIBE: u8 = 0x20;
    pub const UNSUBSCRIBE: u8 = 0x21;
    pub const GET_STATUS: u8 = 0x30;

    // 応答（ロボット→アプリ）
    pub const ACK: u8 = 0x80;
    pub const PONG: u8 = 0x81;
    pub const STATUS: u8 = 0x82;

    // イベント（ロボット→アプリ、購読送出）
    pub const EVT_POSE: u8 = 0x90;
    pub const EVT_LOG: u8 = 0x91;
    pub const EVT_STATUS: u8 = 0x92;
}

// SET_CONTROL_MODE の mode 値。
#[allow(dead_code)]
pub mod control_mode {
    pub const STANDBY: u8 = 0;
    pub const VELOCITY: u8 = 1;
    pub const POSITION: u8 = 2;
}

// ACK result コード。
#[allow(dead_code)]
pub mod ack_result {
    pub const OK: u8 = 0;
    pub const INVALID_PARAM: u8 = 1;
    pub const REJECTED_STATE: u8 = 2;
    pub const NOT_READY: u8 = 3;
    pub const UNSUPPORTED: u8 = 4;
}

#[derive(Debug, Clone)]
pub enum Request {
    Ping,
    SetVelocity { vx: f32, vy: f32, yaw_rate: f32 },
    SetPoseTarget { x: f32, y: f32, speed_limit: f32 },
    Stop,
    SetControlMode { mode: u8 },
    ResetOdometry,
    Subscribe { event_type: u8, rate_hz: u8 },
    Unsubscribe { event_type: u8 },
    GetStatus,
}

impl Request {
    pub fn encode(&self, seq: u8) -> Frame {
        let mut p = Vec::new();
        let t = match self {
            Request::Ping => msg_type::PING,
            Request::SetVelocity { vx, vy, yaw_rate } => {
                push_f32(&mut p, *vx);
                push_f32(&mut p, *vy);
                push_f32(&mut p, *yaw_rate);
                msg_type::SET_VELOCITY
            }
            Request::SetPoseTarget { x, y, speed_limit } => {
                push_f32(&mut p, *x);
                push_f32(&mut p, *y);
                push_f32(&mut p, *speed_limit);
                msg_type::SET_POSE_TARGET
            }
            Request::Stop => msg_type::STOP,
            Request::SetControlMode { mode } => {
                p.push(*mode);
                msg_type::SET_CONTROL_MODE
            }
            Request::ResetOdometry => msg_type::RESET_ODOMETRY,
            Request::Subscribe { event_type, rate_hz } => {
                p.push(*event_type);
                p.push(*rate_hz);
                msg_type::SUBSCRIBE
            }
            Request::Unsubscribe { event_type } => {
                p.push(*event_type);
                msg_type::UNSUBSCRIBE
            }
            Request::GetStatus => msg_type::GET_STATUS,
        };
        Frame::new(t, seq, p)
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // ワイヤフォーマット上の予約フィールド（seq エコー等）を保持する。
pub enum Response {
    Ack { req_type: u8, seq: u8, result: u8 },
    Pong { seq: u8, proto_ver: u8, robot_caps: u8 },
    Status { seq: u8, mode: u8, enable: u8, estop: u8, imu_ok: u8, uptime_ms: u32 },
}

impl Response {
    pub fn decode(f: &Frame) -> Option<Response> {
        let p = f.payload.as_slice();
        match f.msg_type {
            msg_type::ACK if p.len() == 3 => Some(Response::Ack {
                req_type: p[0],
                seq: p[1],
                result: p[2],
            }),
            msg_type::PONG if p.len() == 3 => Some(Response::Pong {
                seq: p[0],
                proto_ver: p[1],
                robot_caps: p[2],
            }),
            msg_type::STATUS if p.len() == 9 => Some(Response::Status {
                seq: p[0],
                mode: p[1],
                enable: p[2],
                estop: p[3],
                imu_ok: p[4],
                uptime_ms: u32::from_le_bytes(p[5..9].try_into().ok()?),
            }),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, serde::Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RobotEvent {
    Pose {
        t_ms: u32,
        x: f32,
        y: f32,
        yaw: f32,
        vx: f32,
        vy: f32,
        yaw_rate: f32,
    },
    Log {
        level: u8,
        text: String,
    },
    Status {
        mode: u8,
        enable: u8,
        estop: u8,
        imu_ok: u8,
    },
}

impl RobotEvent {
    /// Tauri event 名（UI 側 `listen` のキー）を返す。
    pub fn event_name(&self) -> &'static str {
        match self {
            RobotEvent::Pose { .. } => "pose",
            RobotEvent::Log { .. } => "log",
            RobotEvent::Status { .. } => "status",
        }
    }

    pub fn decode(f: &Frame) -> Option<RobotEvent> {
        let p = f.payload.as_slice();
        match f.msg_type {
            msg_type::EVT_POSE if p.len() == 28 => Some(RobotEvent::Pose {
                t_ms: u32::from_le_bytes(p[0..4].try_into().ok()?),
                x: f32::from_le_bytes(p[4..8].try_into().ok()?),
                y: f32::from_le_bytes(p[8..12].try_into().ok()?),
                yaw: f32::from_le_bytes(p[12..16].try_into().ok()?),
                vx: f32::from_le_bytes(p[16..20].try_into().ok()?),
                vy: f32::from_le_bytes(p[20..24].try_into().ok()?),
                yaw_rate: f32::from_le_bytes(p[24..28].try_into().ok()?),
            }),
            msg_type::EVT_LOG if !p.is_empty() => Some(RobotEvent::Log {
                level: p[0],
                text: String::from_utf8_lossy(&p[1..]).into_owned(),
            }),
            msg_type::EVT_STATUS if p.len() == 4 => Some(RobotEvent::Status {
                mode: p[0],
                enable: p[1],
                estop: p[2],
                imu_ok: p[3],
            }),
            _ => None,
        }
    }
}

fn push_f32(buf: &mut Vec<u8>, v: f32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_velocity_layout() {
        let f = Request::SetVelocity { vx: 0.5, vy: -0.25, yaw_rate: 1.0 }.encode(3);
        assert_eq!(f.msg_type, msg_type::SET_VELOCITY);
        assert_eq!(f.seq, 3);
        assert_eq!(f.payload.len(), 12);
        assert_eq!(&f.payload[0..4], &0.5f32.to_le_bytes());
    }

    #[test]
    fn decode_ack() {
        let f = Frame::new(msg_type::ACK, 0, vec![0x10, 5, ack_result::OK]);
        match Response::decode(&f) {
            Some(Response::Ack { req_type, seq, result }) => {
                assert_eq!((req_type, seq, result), (0x10, 5, 0));
            }
            _ => panic!("decode failed"),
        }
    }

    #[test]
    fn decode_pose() {
        let mut p = vec![0u8; 28];
        p[4..8].copy_from_slice(&1.5f32.to_le_bytes());
        let f = Frame::new(msg_type::EVT_POSE, 0, p);
        match RobotEvent::decode(&f) {
            Some(RobotEvent::Pose { x, .. }) => assert_eq!(x, 1.5),
            _ => panic!("decode failed"),
        }
    }
}
