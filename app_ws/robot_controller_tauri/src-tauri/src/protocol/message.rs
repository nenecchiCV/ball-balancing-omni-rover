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
    pub const START_OPERATION: u8 = 0x15;
    pub const STOP_OPERATION: u8 = 0x16;
    pub const SET_POSE: u8 = 0x17;
    pub const CALIBRATE_IMU: u8 = 0x18;
    pub const SET_PARAM: u8 = 0x19;
    pub const SAVE_PARAMS: u8 = 0x1A;
    pub const RESET_PARAMS: u8 = 0x1B;
    pub const TUNE_DEADZONE: u8 = 0x1C;
    pub const SUBSCRIBE: u8 = 0x20;
    pub const UNSUBSCRIBE: u8 = 0x21;
    pub const GET_STATUS: u8 = 0x30;
    pub const TEST_ECHO: u8 = 0x31;
    pub const GET_PARAMS: u8 = 0x32;

    // 応答（ロボット→アプリ）
    pub const ACK: u8 = 0x80;
    pub const PONG: u8 = 0x81;
    pub const STATUS: u8 = 0x82;
    pub const CALIB_RESULT: u8 = 0x83;
    pub const TEST_ECHO_RESP: u8 = 0x84;
    pub const PARAMS: u8 = 0x85;
    pub const TUNE_RESULT: u8 = 0x86;

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

// STATUS/EVT_STATUS の op_state 値。
#[allow(dead_code)]
pub mod op_state {
    pub const STANDBY: u8 = 0;
    pub const OPERATING: u8 = 1;
    pub const FAULT: u8 = 2;
    pub const TUNING: u8 = 3; // FR-17 同定セッション中
}

// STATUS/EVT_STATUS の calib_state 値。
#[allow(dead_code)]
pub mod calib_state {
    pub const NONE: u8 = 0;
    pub const RUNNING: u8 = 1;
    pub const DONE: u8 = 2;
    pub const FAILED: u8 = 3;
}

#[derive(Debug, Clone)]
pub enum Request {
    Ping,
    SetVelocity { vx: f32, vy: f32, yaw_rate: f32 },
    SetPoseTarget { x: f32, y: f32, speed_limit: f32 },
    Stop,
    SetControlMode { mode: u8 },
    ResetOdometry,
    StartOperation,
    StopOperation,
    SetPose { x: f32, y: f32, yaw: f32 },
    CalibrateImu,
    SetParam { key: u8, value: f32 },
    SaveParams,
    ResetParams,
    TuneDeadzone { dir: i8, max_duty: f32, ramp_s: f32 },
    Subscribe { event_type: u8, rate_hz: u8 },
    Unsubscribe { event_type: u8 },
    GetStatus,
    TestEcho { data: Vec<u8> },
    GetParams { start_key: u8 },
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
            Request::StartOperation => msg_type::START_OPERATION,
            Request::StopOperation => msg_type::STOP_OPERATION,
            Request::SetPose { x, y, yaw } => {
                push_f32(&mut p, *x);
                push_f32(&mut p, *y);
                push_f32(&mut p, *yaw);
                msg_type::SET_POSE
            }
            Request::CalibrateImu => msg_type::CALIBRATE_IMU,
            Request::SetParam { key, value } => {
                p.push(*key);
                push_f32(&mut p, *value);
                msg_type::SET_PARAM
            }
            Request::SaveParams => msg_type::SAVE_PARAMS,
            Request::ResetParams => msg_type::RESET_PARAMS,
            Request::TuneDeadzone { dir, max_duty, ramp_s } => {
                p.push(*dir as u8);
                push_f32(&mut p, *max_duty);
                push_f32(&mut p, *ramp_s);
                msg_type::TUNE_DEADZONE
            }
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
            Request::TestEcho { data } => {
                p.extend_from_slice(data);
                msg_type::TEST_ECHO
            }
            Request::GetParams { start_key } => {
                p.push(*start_key);
                msg_type::GET_PARAMS
            }
        };
        Frame::new(t, seq, p)
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)] // ワイヤフォーマット上の予約フィールド（seq エコー等）を保持する。
pub enum Response {
    Ack { req_type: u8, seq: u8, result: u8 },
    Pong { seq: u8, proto_ver: u8, robot_caps: u8 },
    Status { seq: u8, op_state: u8, mode: u8, enable: u8, estop: u8, imu_ok: u8,
             calib_state: u8, uptime_ms: u32 },
    CalibResult { seq: u8, result: u8, gyro_bias_z: f32 },
    TestEchoResp { seq: u8, data: Vec<u8> },
    Params { seq: u8, total: u8, entries: Vec<(u8, f32)> },
    TuneResult { seq: u8, result: u8, onset: [f32; 3], sustain: [f32; 3],
                 suggest: [f32; 3] },
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
            msg_type::STATUS if p.len() == 11 => Some(Response::Status {
                seq: p[0],
                op_state: p[1],
                mode: p[2],
                enable: p[3],
                estop: p[4],
                imu_ok: p[5],
                calib_state: p[6],
                uptime_ms: u32::from_le_bytes(p[7..11].try_into().ok()?),
            }),
            msg_type::CALIB_RESULT if p.len() == 6 => Some(Response::CalibResult {
                seq: p[0],
                result: p[1],
                gyro_bias_z: f32::from_le_bytes(p[2..6].try_into().ok()?),
            }),
            msg_type::TEST_ECHO_RESP if !p.is_empty() => Some(Response::TestEchoResp {
                seq: p[0],
                data: p[1..].to_vec(),
            }),
            // {seq u8, total u8, count u8, [key u8, value f32]*}
            msg_type::PARAMS if p.len() >= 3 => {
                let count = p[2] as usize;
                if p.len() != 3 + count * 5 {
                    return None;
                }
                let mut entries = Vec::with_capacity(count);
                for i in 0..count {
                    let key = p[3 + i * 5];
                    let v = f32::from_le_bytes(
                        p[4 + i * 5..8 + i * 5].try_into().ok()?);
                    entries.push((key, v));
                }
                Some(Response::Params { seq: p[0], total: p[1], entries })
            }
            // {seq u8, result u8, onset f32x3, sustain f32x3, suggest f32x3}
            msg_type::TUNE_RESULT if p.len() == 38 => {
                let rd = |o: usize| -> Option<f32> {
                    Some(f32::from_le_bytes(p[o..o + 4].try_into().ok()?))
                };
                Some(Response::TuneResult {
                    seq: p[0],
                    result: p[1],
                    onset: [rd(2)?, rd(6)?, rd(10)?],
                    sustain: [rd(14)?, rd(18)?, rd(22)?],
                    suggest: [rd(26)?, rd(30)?, rd(34)?],
                })
            }
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
        op_state: u8,
        mode: u8,
        enable: u8,
        estop: u8,
        imu_ok: u8,
        calib_state: u8,
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
            msg_type::EVT_STATUS if p.len() == 6 => Some(RobotEvent::Status {
                op_state: p[0],
                mode: p[1],
                enable: p[2],
                estop: p[3],
                imu_ok: p[4],
                calib_state: p[5],
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
