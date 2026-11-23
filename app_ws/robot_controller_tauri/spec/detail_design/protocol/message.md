# message

`robot_link_protocol.md` のメッセージ定義を型として提供する。TYPE 定数・ペイロード構造・エンコード/デコードを 1 モジュールに集約する。

## 公開型

```rust
pub mod msg_type { /* PING=0x01, SET_VELOCITY=0x10, SET_POSE_TARGET=0x11, STOP=0x12,
                      SET_CONTROL_MODE=0x13, RESET_ODOMETRY=0x14,
                      START_OPERATION=0x15, STOP_OPERATION=0x16, SET_POSE=0x17,
                      CALIBRATE_IMU=0x18, SUBSCRIBE=0x20, UNSUBSCRIBE=0x21,
                      GET_STATUS=0x30, TEST_ECHO=0x31,
                      ACK=0x80, PONG=0x81, STATUS=0x82, CALIB_RESULT=0x83,
                      TEST_ECHO_RESP=0x84, EVT_POSE=0x90, EVT_LOG=0x91, EVT_STATUS=0x92 */ }

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
    Subscribe { event_type: u8, rate_hz: u8 },
    Unsubscribe { event_type: u8 },
    GetStatus,
    TestEcho { data: Vec<u8> },
}

pub enum Response {
    Ack { req_type: u8, seq: u8, result: u8 },
    Pong { seq: u8, proto_ver: u8, robot_caps: u8 },
    Status { seq: u8, op_state: u8, mode: u8, enable: u8, estop: u8, imu_ok: u8,
             calib_state: u8, uptime_ms: u32 },
    CalibResult { seq: u8, result: u8, gyro_bias_z: f32 },
    TestEchoResp { seq: u8, data: Vec<u8> },
}

pub enum RobotEvent {
    Pose { t_ms: u32, x: f32, y: f32, yaw: f32, vx: f32, vy: f32, yaw_rate: f32 },
    Log { level: u8, text: String },
    Status { op_state: u8, mode: u8, enable: u8, estop: u8, imu_ok: u8, calib_state: u8 },
}
```

## 公開関数

- `Request::encode(&self, seq: u8) -> Frame` — TYPE・ペイロードを `robot_link_protocol.md` のレイアウトで符号化する。
- `Response::decode(frame: &Frame) -> Option<Response>` — `0x80-0x84` のフレームを復号する。未知 TYPE・長さ不一致は `None`。
- `RobotEvent::decode(frame: &Frame) -> Option<RobotEvent>` — `0x90-0xAF` のフレームを復号する。

## 内部状態

なし（純粋関数のみ）。

## 備考

- 数値はリトルエンディアン `f32`。`EVT_LOG` の `text` は UTF-8 として損失復号し、不正シーケンスは置換文字で受理する。
