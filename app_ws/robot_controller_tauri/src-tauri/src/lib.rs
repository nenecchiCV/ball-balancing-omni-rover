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
    #[cfg(debug_assertions)]
    specta
        .export(
            specta_typescript::Typescript::default(),
            "../src/lib/bindings.ts",
        )
        .expect("typescript bindings export");

    tauri::Builder::default()
        .plugin(tauri_plugin_shell::init())
        .manage(AppState::new())
        .invoke_handler(specta.invoke_handler())
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
