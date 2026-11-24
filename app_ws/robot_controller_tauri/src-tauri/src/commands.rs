//! IPC 境界: Tauri command 群。
//! UI 引数の検証と `robot::Session` 呼出しのみを担い、ドメインロジックは持たない。

use serde::Serialize;
use specta::Type;
use tauri::{ipc::Channel, State};
use tokio::sync::broadcast;

use crate::ble::nus::DiscoveredDevice;
use crate::error::AppError;
use crate::protocol::message::{ack_result, control_mode, msg_type, Request, Response};
use crate::robot::log::LogMessage;
use crate::robot::session::LinkState;
use crate::robot::telemetry::{PoseTelemetry, SensorTelemetry, StatusTelemetry};
use crate::state::AppState;

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct PongInfo {
    pub proto_ver: u8,
    pub robot_caps: u8,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct RobotStatusInfo {
    pub op_state: u8,
    pub mode: u8,
    pub enable: u8,
    pub estop: u8,
    pub imu_ok: u8,
    pub calib_state: u8,
    pub uptime_ms: u32,
    /// START_OPERATION 受理条件の充足ビット（ready_mask）。旧ファームは None。
    pub start_ready: Option<u8>,
    /// 推定姿勢の傾斜角 [rad]。旧ファームは None。
    pub tilt: Option<f32>,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct CalibResultInfo {
    pub result: u8,
    pub gyro_bias_z: f32,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct TestEchoInfo {
    /// エコーバックされたペイロード（バイト配列）。
    pub data: Vec<u8>,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct ParamEntry {
    pub key: u8,
    pub value: f32,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct ParamsInfo {
    /// ファーム側の調整対象キー総数。
    pub total: u8,
    /// 収集したエントリ（key 昇順）。total と件数が一致するとき全件。
    pub entries: Vec<ParamEntry>,
}

#[derive(Debug, Serialize, Type)]
#[serde(rename_all = "camelCase")]
pub struct TuneResultInfo {
    /// 0=完了、1=中断（E-Stop/キャンセル）。
    pub result: u8,
    /// 立上り duty（回転開始時）の車輪ごとの絶対値。
    pub onset: [f32; 3],
    /// 維持 duty（下降側で回転継続した最小値）。
    pub sustain: [f32; 3],
    /// ファーム算出の推奨 deadzone_comp。
    pub suggest: [f32; 3],
}

/// CALIB_RESULT 待機上限。firmware `link.calib_timeout_ms`(5000) より長め。
const CALIB_TIMEOUT_MS: u32 = 6_000;

#[tauri::command]
#[specta::specta]
pub async fn ble_scan(
    state: State<'_, AppState>,
    timeout_ms: Option<u32>,
) -> Result<Vec<DiscoveredDevice>, AppError> {
    state.session.scan(timeout_ms.unwrap_or(5_000)).await
}

#[tauri::command]
#[specta::specta]
pub async fn ble_connect(state: State<'_, AppState>, address: String) -> Result<(), AppError> {
    state.session.connect(&address).await
}

#[tauri::command]
#[specta::specta]
pub async fn ble_disconnect(state: State<'_, AppState>) -> Result<(), AppError> {
    state.session.disconnect().await;
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn connection_state(state: State<'_, AppState>) -> Result<LinkState, AppError> {
    Ok(state.session.state().await)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_ping(state: State<'_, AppState>) -> Result<PongInfo, AppError> {
    match state.session.request(&Request::Ping).await? {
        Response::Pong {
            proto_ver,
            robot_caps,
            ..
        } => Ok(PongInfo {
            proto_ver,
            robot_caps,
        }),
        _ => Err(AppError::BadResponse),
    }
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_velocity(
    state: State<'_, AppState>,
    vx: f32,
    vy: f32,
    yaw_rate: f32,
) -> Result<(), AppError> {
    state
        .session
        .send_no_ack(&Request::SetVelocity { vx, vy, yaw_rate })
        .await
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_pose_target(
    state: State<'_, AppState>,
    x: f32,
    y: f32,
    speed_limit: f32,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::SetPoseTarget { x, y, speed_limit })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_stop(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::Stop).await?)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_mode(state: State<'_, AppState>, mode: u8) -> Result<(), AppError> {
    if mode > control_mode::POSITION {
        return Err(AppError::InvalidParam(format!("mode が不正: {mode}")));
    }
    ack_to_result(
        state
            .session
            .request(&Request::SetControlMode { mode })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_reset_odometry(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::ResetOdometry).await?)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_get_status(state: State<'_, AppState>) -> Result<RobotStatusInfo, AppError> {
    match state.session.request(&Request::GetStatus).await? {
        Response::Status {
            op_state,
            mode,
            enable,
            estop,
            imu_ok,
            calib_state,
            uptime_ms,
            ready,
            tilt,
            ..
        } => Ok(RobotStatusInfo {
            op_state,
            mode,
            enable,
            estop,
            imu_ok,
            calib_state,
            uptime_ms,
            start_ready: ready,
            tilt,
        }),
        _ => Err(AppError::BadResponse),
    }
}

#[tauri::command]
#[specta::specta]
pub async fn robot_start_operation(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::StartOperation).await?)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_stop_operation(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::StopOperation).await?)
}

#[tauri::command]
#[specta::specta]
pub async fn robot_set_pose(
    state: State<'_, AppState>,
    x: f32,
    y: f32,
    yaw: f32,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::SetPose { x, y, yaw })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_calibrate_imu(state: State<'_, AppState>) -> Result<CalibResultInfo, AppError> {
    match state
        .session
        .request_data(&Request::CalibrateImu, msg_type::CALIB_RESULT, CALIB_TIMEOUT_MS)
        .await?
    {
        Response::CalibResult {
            result,
            gyro_bias_z,
            ..
        } => Ok(CalibResultInfo {
            result,
            gyro_bias_z,
        }),
        _ => Err(AppError::BadResponse),
    }
}

#[tauri::command]
#[specta::specta]
pub async fn robot_test_echo(
    state: State<'_, AppState>,
    data: Vec<u8>,
) -> Result<TestEchoInfo, AppError> {
    if data.is_empty() || data.len() > 63 {
        return Err(AppError::InvalidParam(format!(
            "data は 1-63 バイト: {} バイト",
            data.len()
        )));
    }
    match state
        .session
        .request_data(&Request::TestEcho { data }, msg_type::TEST_ECHO_RESP, 1_000)
        .await?
    {
        Response::TestEchoResp { data, .. } => Ok(TestEchoInfo { data }),
        _ => Err(AppError::BadResponse),
    }
}

/// 調整対象パラメータを全件取得する（GET_PARAMS ページング、FR-09）。
/// `total` 件に達するまで start_key を進めて複数リクエストを送る。
#[tauri::command]
#[specta::specta]
pub async fn robot_get_params(state: State<'_, AppState>) -> Result<ParamsInfo, AppError> {
    let mut entries: Vec<ParamEntry> = Vec::new();
    let mut total: u8 = 0;
    let mut start_key: u8 = 0;
    // 最大 4 ページ（30 件/ページで全 48 キーをカバー）。
    for _ in 0..4 {
        match state
            .session
            .request(&Request::GetParams { start_key })
            .await?
        {
            Response::Params {
                total: t, entries: e, ..
            } => {
                total = t;
                for (key, value) in e {
                    entries.push(ParamEntry { key, value });
                    start_key = key.saturating_add(1);
                }
                if entries.len() >= total as usize || start_key == 0 {
                    break;
                }
            }
            _ => return Err(AppError::BadResponse),
        }
    }
    // key 0x60 = controller.max_speed。指令クランプへ反映する。
    if let Some(e) = entries.iter().find(|e| e.key == PARAM_KEY_MAX_SPEED) {
        state.session.set_max_speed(e.value);
    }
    Ok(ParamsInfo { total, entries })
}

/// 調整対象 key（robot_link_protocol.md「調整対象パラメータ」）。
const PARAM_KEY_MAX_SPEED: u8 = 0x60;

/// パラメータを 1 キーだけ更新する（RAM のみ、SAVE_PARAMS で不揮発化）。
#[tauri::command]
#[specta::specta]
pub async fn robot_set_param(
    state: State<'_, AppState>,
    key: u8,
    value: f32,
) -> Result<(), AppError> {
    if !value.is_finite() {
        return Err(AppError::InvalidParam("value が非有限".into()));
    }
    ack_to_result(
        state
            .session
            .request(&Request::SetParam { key, value })
            .await?,
    )?;
    if key == PARAM_KEY_MAX_SPEED {
        state.session.set_max_speed(value);
    }
    Ok(())
}

/// 動作パラメータをフラッシュへ保存する（ロボット側 STANDBY のみ受理）。
/// 消去+書込みのため数百 ms 掛かる場合がある。
#[tauri::command]
#[specta::specta]
pub async fn robot_save_params(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::SaveParams).await?)
}

/// 保存領域を消去して動作パラメータを既定値へ戻す。
#[tauri::command]
#[specta::specta]
pub async fn robot_reset_params(state: State<'_, AppState>) -> Result<(), AppError> {
    ack_to_result(state.session.request(&Request::ResetParams).await?)?;
    // 既定 max_speed = 0.4（シム検証値）へ戻る。
    state.session.set_max_speed(0.4);
    Ok(())
}

/// 不感帯掃引セッションを実行し TUNE_RESULT を返す（FR-09/FR-17）。
/// 車輪を無負荷にして実行すること。所要時間は 2*ramp_s 秒。
#[tauri::command]
#[specta::specta]
pub async fn robot_tune_deadzone(
    state: State<'_, AppState>,
    dir: Option<i8>,
    max_duty: Option<f32>,
    ramp_s: Option<f32>,
) -> Result<TuneResultInfo, AppError> {
    let dir = dir.unwrap_or(1).clamp(-1, 1);
    let max_duty = max_duty.unwrap_or(0.25).clamp(0.01, 0.8);
    let ramp_s = ramp_s.unwrap_or(15.0).clamp(1.0, 120.0);
    // 掃引全体 + マージンの待機時間。
    let timeout_ms = ((2.0 * ramp_s + 5.0) * 1000.0) as u32;
    match state
        .session
        .request_data(
            &Request::TuneDeadzone { dir, max_duty, ramp_s },
            msg_type::TUNE_RESULT,
            timeout_ms,
        )
        .await?
    {
        Response::TuneResult {
            result,
            onset,
            sustain,
            suggest,
            ..
        } => Ok(TuneResultInfo {
            result,
            onset,
            sustain,
            suggest,
        }),
        _ => Err(AppError::BadResponse),
    }
}

#[tauri::command]
#[specta::specta]
pub async fn robot_subscribe(
    state: State<'_, AppState>,
    event_type: u8,
    rate_hz: u8,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::Subscribe {
                event_type,
                rate_hz,
            })
            .await?,
    )
}

#[tauri::command]
#[specta::specta]
pub async fn robot_unsubscribe(
    state: State<'_, AppState>,
    event_type: u8,
) -> Result<(), AppError> {
    ack_to_result(
        state
            .session
            .request(&Request::Unsubscribe { event_type })
            .await?,
    )
}

// --- UI 向けイベントストリーム（broadcast → Channel 橋渡し） ------------------

/// 現在値を即時送った後、以後の変化をストリームする。
#[tauri::command]
#[specta::specta]
pub async fn watch_link_state(
    state: State<'_, AppState>,
    channel: Channel<LinkState>,
) -> Result<(), AppError> {
    let current = state.session.state().await;
    let _ = channel.send(current);
    spawn_forward(state.session.subscribe_link(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_telemetry(
    state: State<'_, AppState>,
    channel: Channel<PoseTelemetry>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_telemetry(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_sensors(
    state: State<'_, AppState>,
    channel: Channel<SensorTelemetry>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_sensors(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_status(
    state: State<'_, AppState>,
    channel: Channel<StatusTelemetry>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_status(), channel);
    Ok(())
}

#[tauri::command]
#[specta::specta]
pub async fn watch_log(
    state: State<'_, AppState>,
    channel: Channel<LogMessage>,
) -> Result<(), AppError> {
    spawn_forward(state.session.subscribe_log(), channel);
    Ok(())
}

fn spawn_forward<T: Clone + serde::Serialize + Send + 'static>(
    mut rx: broadcast::Receiver<T>,
    channel: Channel<T>,
) {
    tauri::async_runtime::spawn(async move {
        loop {
            match rx.recv().await {
                Ok(v) => {
                    if channel.send(v).is_err() {
                        break;
                    }
                }
                // 遅延分は捨てて最新へ追従する。
                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                Err(broadcast::error::RecvError::Closed) => break,
            }
        }
    });
}

fn ack_to_result(resp: Response) -> Result<(), AppError> {
    match resp {
        Response::Ack {
            req_type, result, ..
        } => match result {
            ack_result::OK => Ok(()),
            r => Err(AppError::Rejected(format!(
                "ACK 拒否: req=0x{req_type:02X} result={r}"
            ))),
        },
        _ => Err(AppError::BadResponse),
    }
}
