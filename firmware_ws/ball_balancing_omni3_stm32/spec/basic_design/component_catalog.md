# コンポーネント一覧

1 コンポーネント = 1 クラス。層は下から `common → peripheral → device → feature → manager` の順で、依存は上位→下位のみ。
システムに 1 つ常駐するコンポーネントは **static class**（全メンバ `static`、ctor `= delete`）とし、複数インスタンスが必要なもののみインスタンス化可能なクラスとする。

## common（純粋ユーティリティ・定数・データ構造）

| コンポーネント | クラス/型 | 責務 |
|---|---|---|
| vector_math | `vec2`/`vec3`/`vec4`/`vec10`/`mat33`/`mat55` 等 | 固定長ベクトル・行列演算、quaternion・回転・skew・3x3 逆行列 |
| control_params | `param_set` 構造体群 | MATLAB `p` 構造に対応する全定数の集約 |
| imu_sample | `imu_sample` 構造体 | IMU 1 サンプルのデータ構造（比力・ジャイロ・有効フラグ）。device 非依存で feature が直接参照する |
| link_protocol | `link_protocol` 名前空間 + `frame_decoder` | ロボットリンクプロトコルのフレーム符号化/復号・CRC16・メッセージ型（純粋ロジック、ホストテスト対象） |

## peripheral（ペリフェラル制御）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| gpio_pin | `gpio_pin` | インスタンス | `gpio_dt_spec` ベースの単一ピン入出力・割込設定（`gpio_irq_cb` 登録） |
| pwm_output | `pwm_output` | インスタンス | `pwm_dt_spec` ベースの 1 チャネル duty 設定 |
| encoder_counter | `encoder_counter` | インスタンス | QDEC 構成済み TIM の `CNT` 読取り・16bit 差分 |
| control_tick | `control_tick` | static | TIM6 1 kHz 更新割込 → `k_sem` 発行の制御タイムベース（`init`/`start`/`stop`/`wait`） |
| timebase | `timebase` | static | `k_uptime` ミリ秒時刻 + DWT サイクル μs 計測 |
| debug_uart | `debug_uart` | static | 診断テキスト送出（`printk`/console バックエンド） |
| link_uart | `link_uart` | static | USART1 割込駆動 RX リング + TX リング（非ブロッキング送受信、`k_sem` 受信通知） |

## device（IC・センサ/アクチュエータ）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| lsm6dso | `lsm6dso` | static | Zephyr sensor ドライバ経由の IMU 読取 + 機体軸への `mat33` 写像 |
| mdd3a_motor | `mdd3a_motor` | インスタンス | 1 チャネルの CW/CCW 2入力（両 PWM）駆動、デューティ→符号付き出力 |
| wheel_encoder | `wheel_encoder` | インスタンス | カウント差分→変位 [rad]・輪速算出用のサンプル |
| estop_input | `estop_input` | static | 外部 E-Stop + 基板ボタンの停止要求検出・EXTI 有効化 |
| status_led | `status_led` | static | 状態表示 LED の点滅パターン |

## feature（機能）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| wheel_speed_meter | `wheel_speed_meter` | static | 5 ms 窓の後退差分輪速 + 保持値提供 |
| state_estimator | `state_estimator` | static | 14 状態 + 5 状態 EKF の推定（FR-01） |
| balance_controller | `balance_controller` | static | LQI 外側タスク + モード/安全判定（FR-03, FR-05） |
| wheel_servo | `wheel_servo` | static | 1 kHz 推進サーボ + duty 反転（FR-04） |
| command_profile | `command_profile` | static | サイクル指令生成（FR-07） |
| link_session | `link_session` | static | ロボットリンクのフレーム振分け・モード/指令スロット・購読・ウォッチドッグ・イベント生成・リンクスレッドポンプ（FR-09, FR-11） |
| odometry | `odometry` | static | ワールド速度の積分による位置推定・原点オフセット管理（FR-10, FR-11） |
| position_controller | `position_controller` | static | 位置外ループ（目標位置→ワールド速度指令生成、FR-10） |

## manager（機能管理）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| task_scheduler | `task_scheduler` | static | 1 kHz/200 Hz/診断の周期実行管理（NFR-01） |
| robot_application | `robot_application` | static | コンポーネント設定注入・init/run ループ・shell コマンド（FR-06, FR-08） |

## src/main

- `main.cpp`: `robot_application::init(cfg)` → `run()` の呼出のみ

## test（オンボードテスト層・別ビルド）

実機検証用。`tests/onboard/` の別 Zephyr アプリとしてビルドし、
`common〜feature` の下位層を再利用する。各テストの仕様は `spec/test/` を参照。

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| test_runner | `test_runner` | static | shell `test` コマンド・1 kHz ティック駆動のテスト実行管理（`start_cb`/`update_cb`/`stop_cb` ディスパッチ、オーバーラン計数、mutex 排他） |
| test_application | `test_application` | static | テストアプリのコンポジションルート（robot_application と同配線＋テストへの注入） |
| estop_test | `estop_test` | static | E-stop 入力の変化・要求検出の表示 |
| tick_timing_test | `tick_timing_test` | static | 1 kHz ティック周期の統計報告 |
| imu_attitude_test | `imu_attitude_test` | static | IMU+推定器の姿勢出力表示（軸写像検証） |
| encoder_test | `encoder_test` | static | エンコーダ方向・カウント表示（手回し/モータ駆動） |
| omni_drive_test | `omni_drive_test` | static | [vx,vy,yaw]→輪速指令のオムニ駆動（テスト固有 FF+P サーボ） |
| motor_deadzone_test | `motor_deadzone_test` | static | デューティ三角掃引によるモータ不感帯同定 |
