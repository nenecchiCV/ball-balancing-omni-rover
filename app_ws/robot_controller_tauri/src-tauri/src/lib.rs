mod ble;
mod commands;
mod error;
mod protocol;
mod robot;
mod state;

use state::AppState;

fn specta_builder() -> tauri_specta::Builder<tauri::Wry> {
    tauri_specta::Builder::<tauri::Wry>::new().commands(tauri_specta::collect_commands![
        commands::ble_scan,
        commands::ble_connect,
        commands::ble_disconnect,
        commands::connection_state,
        commands::robot_ping,
        commands::robot_set_velocity,
        commands::robot_set_pose_target,
        commands::robot_stop,
        commands::robot_set_mode,
        commands::robot_reset_odometry,
        commands::robot_get_status,
        commands::robot_start_operation,
        commands::robot_stop_operation,
        commands::robot_set_pose,
        commands::robot_calibrate_imu,
        commands::robot_test_echo,
        commands::robot_subscribe,
        commands::robot_unsubscribe,
        commands::watch_link_state,
        commands::watch_telemetry,
        commands::watch_status,
        commands::watch_log,
    ])
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let specta = specta_builder();

    // 開発ビルドのたびに TypeScript 型を再生成する（src/lib/bindings.ts）。
    // モバイルでは相対パス ../src が解決できないためデスクトップのみで実行する。
    #[cfg(all(debug_assertions, desktop))]
    specta
        .export(
            specta_typescript::Typescript::default(),
            "../src/lib/bindings.ts",
        )
        .expect("typescript bindings export");

    let builder = tauri::Builder::default();
    #[cfg(desktop)]
    let builder = builder.plugin(tauri_plugin_shell::init());
    builder
        .manage(AppState::new())
        .invoke_handler(specta.invoke_handler())
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

#[cfg(test)]
mod tests {
    /// GUI を起動せずに bindings.ts を再生成する（`cargo test export_bindings`）。
    #[test]
    fn export_bindings() {
        super::specta_builder()
            .export(
                specta_typescript::Typescript::default(),
                "../src/lib/bindings.ts",
            )
            .expect("typescript bindings export");
    }
}
