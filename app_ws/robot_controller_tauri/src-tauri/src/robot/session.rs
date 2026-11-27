//! BLE セッション: 接続ライフサイクル・リクエスト送出・受信振分け。
//! `AppState` から `Arc<Session>` として共有される。
//! UI への配信は `tokio::sync::broadcast` で行い、Tauri に依存しない。
//! 購読は `watch_*` command が broadcast → `tauri::ipc::Channel` で橋渡しする。

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Duration;

use serde::Serialize;
use specta::Type;
use tokio::sync::{broadcast, oneshot, Mutex};
use tokio::task::JoinHandle;
use tokio::time::timeout;

use crate::ble::nus::{self, DiscoveredDevice, NusLink, DEFAULT_NAME_FILTER};
use crate::error::AppError;
use crate::protocol::frame::{Frame, FrameDecoder};
use crate::protocol::message::{ack_result, Request, Response, RobotEvent};
use crate::robot::log::{LogMessage, LogSource};
use crate::robot::telemetry::{PoseTelemetry, SensorTelemetry, StatusTelemetry};
use futures::StreamExt;

const REQUEST_TIMEOUT: Duration = Duration::from_secs(1);
const CONNECT_SCAN_TIMEOUT: Duration = Duration::from_secs(5);
const MONITOR_INTERVAL: Duration = Duration::from_millis(500);
const PROTO_VER: u8 = 3;

// 送信直前クランプ（NFR-02 の 2 段階目）。`control_params.controller` と同期。
// max_speed はロボットの調整パラメータ（key 0x60）に追随する。
const DEFAULT_MAX_SPEED: f32 = 0.35;
const MAX_YAW_RATE: f32 = 0.8;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Type)]
#[serde(rename_all = "lowercase")]
pub enum LinkState {
    Disconnected,
    Scanning,
    Connecting,
    Connected,
}

/// UI 向けイベント配信バス。受信者が居なくても送信側は失敗しない。
struct EventBus {
    link: broadcast::Sender<LinkState>,
    telemetry: broadcast::Sender<PoseTelemetry>,
    sensors: broadcast::Sender<SensorTelemetry>,
    status: broadcast::Sender<StatusTelemetry>,
    log: broadcast::Sender<LogMessage>,
}

/// ACK 後にデータ応答（STATUS/CALIB_RESULT 等）を待つか、ACK のみで完了するか。
enum PendingEntry {
    /// 最初に届いた応答で完了する。
    OneShot(oneshot::Sender<Frame>),
    /// ACK(OK) では待機継続し、データ応答（または NACK）で完了する。
    /// 期待 TYPE の検査は `request_data` が行う。
    AckThen(oneshot::Sender<Frame>),
}

impl PendingEntry {
    fn into_tx(self) -> oneshot::Sender<Frame> {
        match self {
            PendingEntry::OneShot(tx) | PendingEntry::AckThen(tx) => tx,
        }
    }
}

type PendingMap = Arc<StdMutex<HashMap<u8, PendingEntry>>>;

struct Inner {
    link: Option<Arc<NusLink>>,
    state: LinkState,
    seq: u8,
    pending: PendingMap,
    subscriptions: HashSet<u8>,
    dispatcher: Option<JoinHandle<()>>,
    monitor: Option<JoinHandle<()>>,
}

pub struct Session {
    inner: Mutex<Inner>,
    bus: EventBus,
    /// ロボットの `controller.max_speed`（key 0x60）キャッシュ。
    /// GET_PARAMS / SET_PARAM / RESET_PARAMS 時に更新し、指令クランプに使う。
    max_speed_bits: std::sync::atomic::AtomicU32,
}

impl Session {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(Inner {
                link: None,
                state: LinkState::Disconnected,
                seq: 0,
                pending: Arc::new(StdMutex::new(HashMap::new())),
                subscriptions: HashSet::new(),
                dispatcher: None,
                monitor: None,
            }),
            bus: EventBus {
                link: broadcast::channel(16).0,
                telemetry: broadcast::channel(64).0,
                sensors: broadcast::channel(64).0,
                status: broadcast::channel(16).0,
                log: broadcast::channel(256).0,
            },
            max_speed_bits: std::sync::atomic::AtomicU32::new(
                DEFAULT_MAX_SPEED.to_bits()),
        }
    }

    /// 指令クランプに使うロボット max_speed を更新する。
    pub fn set_max_speed(&self, v: f32) {
        if v.is_finite() && v > 0.0 {
            self.max_speed_bits
                .store(v.to_bits(), std::sync::atomic::Ordering::Relaxed);
        }
    }

    fn max_speed(&self) -> f32 {
        f32::from_bits(
            self.max_speed_bits.load(std::sync::atomic::Ordering::Relaxed))
    }

    /// 指令値クランプ（NFR-02）。速度上限はロボットの `max_speed` に追随する。
    fn clamped(&self, req: &Request) -> Request {
        let ms = self.max_speed();
        match req {
            Request::SetVelocity { vx, vy, yaw_rate } => Request::SetVelocity {
                vx: vx.clamp(-ms, ms),
                vy: vy.clamp(-ms, ms),
                yaw_rate: yaw_rate.clamp(-MAX_YAW_RATE, MAX_YAW_RATE),
            },
            Request::SetPoseTarget { x, y, speed_limit } => {
                Request::SetPoseTarget {
                    x: *x,
                    y: *y,
                    speed_limit: speed_limit.clamp(0.0, ms),
                }
            }
            _ => req.clone(),
        }
    }

    // --- 購読 --------------------------------------------------------------

    pub fn subscribe_link(&self) -> broadcast::Receiver<LinkState> {
        self.bus.link.subscribe()
    }
    pub fn subscribe_telemetry(&self) -> broadcast::Receiver<PoseTelemetry> {
        self.bus.telemetry.subscribe()
    }
    pub fn subscribe_sensors(&self) -> broadcast::Receiver<SensorTelemetry> {
        self.bus.sensors.subscribe()
    }
    pub fn subscribe_status(&self) -> broadcast::Receiver<StatusTelemetry> {
        self.bus.status.subscribe()
    }
    pub fn subscribe_log(&self) -> broadcast::Receiver<LogMessage> {
        self.bus.log.subscribe()
    }

    /// アプリ側イベントをログバスへ流す（接続/切断/警告）。
    fn app_log(&self, level: u8, text: impl Into<String>) {
        let _ = self.bus.log.send(LogMessage {
            level,
            text: text.into(),
            source: LogSource::App,
        });
    }

    // --- 接続状態 ------------------------------------------------------------

    pub async fn state(&self) -> LinkState {
        self.inner.lock().await.state
    }

    async fn set_state(&self, s: LinkState) {
        {
            self.inner.lock().await.state = s;
        }
        let _ = self.bus.link.send(s);
    }

    /// 周辺機器スキャン。`name_filter`（`DEFAULT_NAME_FILTER`）一致を先頭に並べて返す。
    pub async fn scan(&self, timeout_ms: u32) -> Result<Vec<DiscoveredDevice>, AppError> {
        self.set_state(LinkState::Scanning).await;
        let res = nus::scan(DEFAULT_NAME_FILTER, Duration::from_millis(timeout_ms.into())).await;
        let mut inner = self.inner.lock().await;
        if inner.state == LinkState::Scanning {
            inner.state = LinkState::Disconnected;
            let _ = self.bus.link.send(LinkState::Disconnected);
        }
        res
    }

    /// デバイス接続 → 特性確保 → 通知購読 → ディスパッチャ/監視タスク起動 → PING 照合。
    pub async fn connect(self: &Arc<Self>, address: &str) -> Result<(), AppError> {
        self.set_state(LinkState::Connecting).await;
        let link = match NusLink::connect(address, CONNECT_SCAN_TIMEOUT).await {
            Ok(l) => Arc::new(l),
            Err(e) => {
                self.set_state(LinkState::Disconnected).await;
                return Err(e);
            }
        };
        let stream = match link.notification_stream().await {
            Ok(s) => s,
            Err(e) => {
                let _ = link.disconnect().await;
                self.set_state(LinkState::Disconnected).await;
                return Err(e);
            }
        };

        // ディスパッチャ: 通知バイト → フレーム復号 → pending 完了 / event 配信。
        let pending = self.inner.lock().await.pending.clone();
        let weak = Arc::downgrade(self);
        let dispatcher = tokio::spawn(async move {
            let mut dec = FrameDecoder::new();
            let mut stream = stream;
            while let Some(bytes) = stream.next().await {
                for frame in dec.push(&bytes) {
                    if let Some(resp) = Response::decode(&frame) {
                        // 応答相関はペイロードの seq エコーで行う（ヘッダ SEQ は応答では 0）。
                        let seq = resp_seq(&resp);
                        let mut map = pending.lock().unwrap();
                        let completes = match map.get(&seq) {
                            Some(PendingEntry::AckThen(_)) => match &resp {
                                // ACK(OK) は継続待機。NACK/データ応答/不整合は完了。
                                Response::Ack { result, .. } => *result != ack_result::OK,
                                _ => true,
                            },
                            Some(PendingEntry::OneShot(_)) => true,
                            None => false,
                        };
                        if completes {
                            if let Some(entry) = map.remove(&seq) {
                                let _ = entry.into_tx().send(frame);
                            }
                        }
                    } else if let Some(ev) = RobotEvent::decode(&frame) {
                        if let Some(client) = weak.upgrade() {
                            client.publish_robot_event(&ev);
                        }
                    }
                }
            }
            // 通知ストリーム終端 = リンク断。
            if let Some(client) = weak.upgrade() {
                client.link_lost().await;
            }
        });

        // 接続監視: 定期的に is_connected を確認する。
        let weak = Arc::downgrade(self);
        let link_mon = link.clone();
        let monitor = tokio::spawn(async move {
            loop {
                tokio::time::sleep(MONITOR_INTERVAL).await;
                if !link_mon.is_connected().await {
                    if let Some(client) = weak.upgrade() {
                        client.link_lost().await;
                    }
                    break;
                }
            }
        });

        {
            let mut inner = self.inner.lock().await;
            inner.link = Some(link);
            inner.dispatcher = Some(dispatcher);
            inner.monitor = Some(monitor);
            inner.subscriptions.clear();
        }
        self.set_state(LinkState::Connected).await;
        self.app_log(1, format!("BLE 接続: {address}"));

        // プロトコルバージョン照合。失敗は警告のみで接続は維持する。
        match self.request(&Request::Ping).await {
            Ok(Response::Pong { proto_ver, .. }) if proto_ver != PROTO_VER => {
                self.app_log(
                    2,
                    format!("プロトコルバージョン不一致: robot={proto_ver} app={PROTO_VER}"),
                );
            }
            Ok(_) => {}
            Err(e) => self.app_log(2, format!("PING 失敗: {e}")),
        }
        Ok(())
    }

    /// 明示切断。ディスパッチャ・監視タスクを止めて BLE を切断する。
    pub async fn disconnect(&self) {
        let (link, dispatcher, monitor) = {
            let mut inner = self.inner.lock().await;
            inner.state = LinkState::Disconnected;
            inner.subscriptions.clear();
            inner.pending.lock().unwrap().clear();
            (
                inner.link.take(),
                inner.dispatcher.take(),
                inner.monitor.take(),
            )
        };
        if let Some(h) = dispatcher {
            h.abort();
        }
        if let Some(h) = monitor {
            h.abort();
        }
        if let Some(l) = link {
            let _ = l.disconnect().await;
        }
        let _ = self.bus.link.send(LinkState::Disconnected);
        self.app_log(1, "BLE 切断");
    }

    /// リクエスト送出して応答（ACK/PONG/STATUS）を待つ。タイムアウト 1 s。
    pub async fn request(&self, req: &Request) -> Result<Response, AppError> {
        let (link, seq, rx) = {
            let mut inner = self.inner.lock().await;
            let link = inner.link.clone().ok_or(AppError::NotConnected)?;
            inner.seq = inner.seq.wrapping_add(1);
            if inner.seq == 0 {
                inner.seq = 1;
            }
            let seq = inner.seq;
            let (tx, rx) = oneshot::channel();
            inner
                .pending
                .lock()
                .unwrap()
                .insert(seq, PendingEntry::OneShot(tx));
            (link, seq, rx)
        };

        let frame = self.clamped(req).encode(seq);
        if let Err(e) = link.send(&frame.encode()).await {
            self.drop_pending(seq).await;
            return Err(AppError::Send(e.to_string()));
        }

        match timeout(REQUEST_TIMEOUT, rx).await {
            Ok(Ok(resp_frame)) => {
                let resp = Response::decode(&resp_frame).ok_or(AppError::BadResponse)?;
                self.note_subscription(req, &resp).await;
                Ok(resp)
            }
            Ok(Err(_)) => Err(AppError::NotConnected),
            Err(_) => {
                self.drop_pending(seq).await;
                Err(AppError::Timeout)
            }
        }
    }

    /// ACK(OK) 後に `expect_type` のデータ応答を待つ 2 段リクエスト（CALIBRATE_IMU 等）。
    /// `timeout_ms` は ACK+データ応答全体の待機上限。
    pub async fn request_data(
        &self,
        req: &Request,
        expect_type: u8,
        timeout_ms: u32,
    ) -> Result<Response, AppError> {
        let (link, seq, rx) = {
            let mut inner = self.inner.lock().await;
            let link = inner.link.clone().ok_or(AppError::NotConnected)?;
            inner.seq = inner.seq.wrapping_add(1);
            if inner.seq == 0 {
                inner.seq = 1;
            }
            let seq = inner.seq;
            let (tx, rx) = oneshot::channel();
            inner
                .pending
                .lock()
                .unwrap()
                .insert(seq, PendingEntry::AckThen(tx));
            (link, seq, rx)
        };

        let frame = self.clamped(req).encode(seq);
        if let Err(e) = link.send(&frame.encode()).await {
            self.drop_pending(seq).await;
            return Err(AppError::Send(e.to_string()));
        }

        match timeout(Duration::from_millis(timeout_ms.into()), rx).await {
            Ok(Ok(resp_frame)) => {
                let resp = Response::decode(&resp_frame).ok_or(AppError::BadResponse)?;
                match &resp {
                    // NACK または ACK 以外の応答で終了した場合を仕分ける。
                    Response::Ack { req_type, result, .. } => Err(AppError::Rejected(format!(
                        "ACK 拒否: req=0x{req_type:02X} result={result}"
                    ))),
                    _ if resp_frame.msg_type == expect_type => Ok(resp),
                    _ => Err(AppError::BadResponse),
                }
            }
            Ok(Err(_)) => Err(AppError::NotConnected),
            Err(_) => {
                self.drop_pending(seq).await;
                Err(AppError::Timeout)
            }
        }
    }

    /// 応答を待たない送出（SET_VELOCITY 等高頻度経路）。
    pub async fn send_no_ack(&self, req: &Request) -> Result<(), AppError> {
        let (link, seq) = {
            let mut inner = self.inner.lock().await;
            let link = inner.link.clone().ok_or(AppError::NotConnected)?;
            inner.seq = inner.seq.wrapping_add(1);
            if inner.seq == 0 {
                inner.seq = 1;
            }
            (link, inner.seq)
        };
        let frame = self.clamped(req).encode(seq);
        link.send(&frame.encode())
            .await
            .map_err(|e| AppError::Send(e.to_string()))
    }

    async fn drop_pending(&self, seq: u8) {
        self.inner.lock().await.pending.lock().unwrap().remove(&seq);
    }

    /// ACK 成功時に購読状態を帳簿へ反映する。
    async fn note_subscription(&self, req: &Request, resp: &Response) {
        let ok = matches!(resp, Response::Ack { result: 0, .. });
        if !ok {
            return;
        }
        let mut inner = self.inner.lock().await;
        match req {
            Request::Subscribe { event_type, .. } => {
                inner.subscriptions.insert(*event_type);
            }
            Request::Unsubscribe { event_type } => {
                inner.subscriptions.remove(event_type);
            }
            _ => {}
        }
    }

    /// ロボットイベントを対応するバスへ配信する。
    fn publish_robot_event(&self, ev: &RobotEvent) {
        if let Some(p) = Option::<PoseTelemetry>::from(ev) {
            let _ = self.bus.telemetry.send(p);
        }
        if let Some(sv) = Option::<SensorTelemetry>::from(ev) {
            let _ = self.bus.sensors.send(sv);
        }
        if let Some(s) = Option::<StatusTelemetry>::from(ev) {
            let _ = self.bus.status.send(s);
        }
        if let RobotEvent::Log { level, text } = ev {
            let _ = self.bus.log.send(LogMessage {
                level: *level,
                text: text.clone(),
                source: LogSource::Robot,
            });
        }
    }

    /// リンク断検出（ディスパッチャ/監視タスクから呼ばれる）。
    async fn link_lost(self: &Arc<Self>) {
        {
            let mut inner = self.inner.lock().await;
            if inner.state == LinkState::Disconnected {
                return;
            }
            inner.state = LinkState::Disconnected;
            inner.link = None;
            inner.subscriptions.clear();
            inner.pending.lock().unwrap().clear();
            if let Some(h) = inner.dispatcher.take() {
                h.abort();
            }
            if let Some(h) = inner.monitor.take() {
                h.abort();
            }
        }
        let _ = self.bus.link.send(LinkState::Disconnected);
        self.app_log(2, "BLE リンク断絶");
    }
}

/// `Response` ペイロードの seq エコーを取り出す（応答相関キー）。
fn resp_seq(resp: &Response) -> u8 {
    match resp {
        Response::Ack { seq, .. }
        | Response::Pong { seq, .. }
        | Response::Status { seq, .. }
        | Response::CalibResult { seq, .. }
        | Response::TestEchoResp { seq, .. }
        | Response::Params { seq, .. }
        | Response::TuneResult { seq, .. } => *seq,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_velocity() {
        let s = Session::new();
        match s.clamped(&Request::SetVelocity {
            vx: 1.0,
            vy: -1.0,
            yaw_rate: 9.0,
        }) {
            Request::SetVelocity { vx, vy, yaw_rate } => {
                assert_eq!((vx, vy, yaw_rate), (0.35, -0.35, 0.8));
            }
            _ => panic!(),
        }
    }

    #[test]
    fn clamps_follow_robot_max_speed() {
        let s = Session::new();
        s.set_max_speed(0.8);
        match s.clamped(&Request::SetVelocity {
            vx: 1.0,
            vy: 0.0,
            yaw_rate: 0.0,
        }) {
            Request::SetVelocity { vx, .. } => assert_eq!(vx, 0.8),
            _ => panic!(),
        }
    }
}
