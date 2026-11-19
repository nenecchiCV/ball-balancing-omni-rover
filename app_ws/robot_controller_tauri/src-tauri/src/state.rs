//! `tauri::Builder::manage` に登録する共有状態。
//! ドメイン本体は `robot::Session` にあり、本型は IPC 側のハンドルとして機能する。

use std::sync::Arc;

use crate::robot::session::Session;

pub struct AppState {
    pub session: Arc<Session>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            session: Arc::new(Session::new()),
        }
    }
}
