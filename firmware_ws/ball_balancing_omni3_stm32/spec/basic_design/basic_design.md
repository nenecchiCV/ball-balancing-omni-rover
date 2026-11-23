# 基本設計書

`ball_balancing_omni3_stm32` のソフトウェア構成・実行モデル・実装方針を定義する。制御・推定アルゴリズムの正本は `matlab_ws/ball_balancing_omni3` の rev2 構成（外側 200 Hz / 内側 1 kHz のマルチレート）であり、本設計はその分割を Zephyr RTOS 上の実時間タスクに写像する。

## 実行基盤

- **RTOS**: Zephyr 4.3（Zephyr SDK 0.17.4、arm-zephyr-eabi-gcc 12.2）。対象ボード `nucleo_g474re`。
- **ハードウェア記述**: ピン割当・タイマ・I2C・PWM・QDEC・IMU・E-Stop はすべて devicetree（`boards/nucleo_g474re.overlay` + `dts/bindings/bb,pins.yaml`）で宣言し、コードは `DT_*` マクロ経由でのみ参照する。`spec/requirements/hardware_config.md` はこの overlay の人間可読な対応表とする。
- **ビルド**: `west` 経由の Zephyr アプリケーション。Zephyr 本体・HAL モジュールはワークスペース（`C:\zephyr_ws`）に置き、アプリ側ソースへ混入させない。
- **C++**: 制限付き C++ を採用する（詳細は「言語規約」）。例外・RTTI・動的確保・STL を使わず、STL 相当が必要な箇所は ETL（`third_party/etl`、git submodule、ヘッダオンリー）で代替する。

## ソフト構成

### 層構造

下から上への一方向依存のみ許可する（同一層内は循環しない範囲で許可）。

| 層 | 責務 | 例 |
|---|---|---|
| `manager` | 機能管理・タスク統合・コンポーネント配線 | `robot_application`, `task_scheduler` |
| `feature` | 機能（推定・制御・計測・指令生成・リンク管理） | `state_estimator`, `balance_controller`, `wheel_servo`, `link_session`, `odometry`, `position_controller`, `operation_manager` |
| `device` | 1 つの IC・センサ/アクチュエータ | `lsm6dso`, `mdd3a_motor`, `wheel_encoder` |
| `peripheral` | MCU ペリフェラル制御（Zephyr ドライバ/LL の薄いラッパ） | `gpio_pin`, `pwm_output`, `encoder_counter`, `control_tick`, `timebase`, `debug_uart`, `link_uart` |
| `common` | 純粋ユーティリティ・定数・データ構造（依存なし） | `vector_math`, `control_params`, `imu_sample` |

### クラス設計ルール

- 1 コンポーネント = 1 クラス。継承は使わず、状態はメンバ変数に保持する。
- システムに 1 つ常駐するコンポーネントは static class（全メンバ `static`、ctor `= delete`）とする。複数インスタンスが必要なもの（モータ・エンコーダ・PWM・GPIO）は `init(const config&)` で設定を受けるクラスとする。
- MATLAB の `persistent` 変数はクラスメンバに写像する（`state_estimator`・`balance_controller`・`wheel_servo` の主要な状態変数）。
- インターフェースは値型（`etl::array` ベースの `vec2`/`vec3`/`vec4`/`mat33`）で受け渡し、ポインタや共有所有を避ける。
- `float` 統一。例外・RTTI・動的メモリは使わない。

## タスク管理

TIM6 の 1 kHz 更新割込を基準に、単一の制御スレッド上で 3 タスクを分周実行する。ISR は `k_sem` を give するだけで、処理本体は制御スレッドコンテキストで実行する。全タスクが同一スレッドのため、`est_out_`/`ctrl_out_`/`wheel_rate_` 等のタスク間共有コンテキストは本質的に排他される（ZOH 更新は外側タスク末尾で行う）。

| タスク | 周期 | 内容 | 実行時間目標 |
|---|---|---|---|
| 外側（Outer） | 5 ms | IMU 読取 → 輪速計測 → 推定 → 指令生成 → LQI 外側タスク → サーボコンテキスト更新 | < 3 ms |
| 内側（Inner） | 1 ms | サーボコンテキスト（ZOH）+ 保持輪速 → 内側タスク → duty → CW/CCW-PWM 出力 | < 0.2 ms |
| 診断（Diag） | 200 ms | LED 更新 + UART テレメトリ（`bb telem` で有効化） | < 1 ms |

セマフォの滞留はオーバーランとして計数し、追いつき実行は行わない（位相歪み防止）。実行時間は `timebase::cycles()` で DWT サイクル計測し、`bb status` で最大値を参照できる。

### スレッドと共有データ

- 制御スレッド（本アプリの `run()`）が上記 3 タスクを逐次実行する。
- **リンクスレッド**（`robot_application` が生成する別 Zephyr スレッド）が USART1 の受信フレーム処理とイベント送出を担当する。制御スレッドの実時間性を阻害しないよう、受信デコード・応答生成・TX 送出はすべてリンクスレッド側で行う。
- Zephyr の shell スレッドからは `snapshot()` 経由でのみ状態を参照する。`snapshot()` は `irq_lock()` で短いクリティカル区間を取り、構造体ごと整合コピーを返す。
- リンクスレッド ↔ 制御スレッド間の共有データ（速度指令スロット・ロボット状態メールボックス・購読状態・位置目標スロット・動作状態）は、小さい構造体の `irq_lock()` 整合コピーで受け渡す。イベント送出は `k_msgq`（イベントキュー）で受け渡し、ドロップ発生時は計数のみ行う。
- `telem_enabled_` のような 1 ワードフラグは `volatile` で共有する（Cortex-M 上でロード/ストアはアトミック）。

## 通信

- **センサ**: LSM6DSO は Zephyr sensor ドライバ（`st,lsm6dso`）経由で I2C1 から読取る。読取は外側タスクの先頭で行い、推定・制御は同一サンプルを共有する。軸写像は `device/lsm6dso` が `mat33` で適用する。
- **車輪**: エンコーダは devicetree の `st,stm32-qdec` ノードが TIM エンコーダモードを初期化する。`encoder_counter` は `LL_TIM` 経由で `CNT` を読み、2^n CPR 設定による自由周回の 16 ビット差分を取得する。MxA/MxB の CW/CCW-PWM は内側タスクで Zephyr PWM ドライバ経由で更新する。
- **ホスト**: USART2（115200 8N1）を Zephyr console/shell バックエンドとして使用する。`bb status` で状態照会、`bb telem on|off` で CSV テレメトリの送出を切替える。テレメトリ本体は `printk` 経由で送出する。
- **リンク**: USART1（115200 8N1、PC4=TX / PC5=RX）は `uart_wireless_converter_esp32` 経由の BLE 透過リンクで、`app_ws/robot_controller_tauri` とロボットリンクプロトコル（`app_ws/robot_controller_tauri/spec/basic_design/robot_link_protocol.md` が正本）を交換する。フレーム符号化は `common/link_protocol` に、USART1 ドライバは `peripheral/link_uart` に、セッション管理は `feature/link_session` に、動作状態（待機/動作中/終端）の管理は `feature/operation_manager` に集約する。受信は割込駆動リングバッファ + リンクスレッド処理、送信は割込駆動 TX リングで制御スレッドをブロックしない。購読イベントは `link_session` がバッファへ集約し `link.event_batch_ms`（既定 100 ms）周期でまとめて送出し、単発応答は即時送出する（FR-14）。

## メモリ管理

- 全バッファ・行列は静的/スタック確保。ヒープは使わない（`CONFIG_HEAP_MEM_POOL_SIZE=0` でカーネルヒープも無効化）。
- 制御・推定の内部状態は各クラスのメンバ（合計で数 KB）に集約する。
- `etl::array` 等の固定容量コンテナのみ使用する。可変長コンテナは使わない。

## 言語規約（制限付き C++）

- 標準は C++23（`CONFIG_STD_CPP23`）。ただし **STL 禁止**（`-nostdinc++`）・**例外禁止**（`-fno-exceptions`）・**RTTI 禁止**（`-fno-rtti`）・**動的確保禁止**（`new`/`delete`/`malloc` 不使用）。
- STL 相当の機能が必要な場合は ETL（`third_party/etl`）を使う。`ETL_NO_STL` を定義し、ETL が STL ヘッダへ依存しない構成とする。
- 関数ローカル static のスレッドセーフガードは使わない（`-fno-threadsafe-statics`）。初期化は `main` 単一スレッド内で完了する前提とする。
- `CONFIG_STATIC_INIT_GNU=n` により実行時コンストラクタは走らない。static メンバ・グローバルはゼロ/定数初期化のみで成り立たせ、非ゼロの値は `init()` で設定する。
- **static class ルール**: システムに 1 つ常駐するコンポーネント（タスク・機能・デバイス・アプリケーション）は全メンバ `static` の static class で実装し、デフォルトコンストラクタを `= delete` で禁止する。モータ・エンコーダ・PWM・GPIO など複数インスタンスが必要なクラスのみインスタンス化を許可する。
- **ライフサイクル**: コンストラクタ・デストラクタは使わない。設定注入は `init(const config&)`、開始・停止が必要なものは `start()`/`stop()` メンバ関数で行う。
- **命名規則**: 識別子は原則すべて `snake_case`（クラス・関数・変数・ファイル名を含む）。定数のみ `UPPER_SNAKE_CASE` とし、`constexpr` で定義する（マクロによる定数定義は行わない）。
- **コールバック命名**: ISR・shell ハンドラ・タスクフック等の登録されて呼び返される関数は `*_cb` サフィックスとし、名前からコールバックと分かるようにする。static class 化によりインスタンス参照用グローバルポインタは使わない。

## 実装方針

- **移植**: MATLAB rev2 の各関数を「入力→内部状態→出力」のままメンバ関数に写像し、数式・定数・ゲート条件を変えない。`isfield` による機能スイッチは `constexpr bool` 相当のフラグ判定に置き換え、有効な分岐のみ実装する（wheel_rate_observer は既定無効のため実装対象外とし、有効化時は同じフィールド名で追加する）。
- **型**: ベクトル/行列は `common/vector_math` の `vec2`/`vec3`/`vec4`/`mat33` を使用。固定 5x5 は `mat55`。IMU サンプルは `common/imu_sample` の `imu_sample`（device 非依存のデータ構造）で受け渡す。
- **定数**: 制御・推定・ハードの全定数は `common/control_params.hpp` に `constexpr`/集約初期化で集約する。元の `ballbotRev1Parameters` のフィールド名を保持する。
- **ヘッダ/ソース分割**: 公開型とクラス宣言は `*.hpp`、実装と内部関数は `*.cpp` に置く。inline に適する小さい定数・単純 getter はヘッダに残す。MATLAB のローカルヘルパ関数は `*.cpp` 内の無名名前空間に写像する。
- **初期化**: `robot_application` がコンストラクタで全コンポーネントを配線し、`init()` でドライバ初期化・IMU 検出・割込有効化を行い、`run()` でスケジューラに制御を渡す。

## 制御構成の対応

| MATLAB 構成要素 | ファームウェア側コンポーネント |
|---|---|
| `ballbotEstimatorStep` | `feature/state_estimator` |
| `ballbotLqiControllerUpdateRev2` | `feature/balance_controller` |
| `ballbotWheelServoDutyStep` | `feature/wheel_servo` |
| `ballbotYawBiasStartupGuard` | `balance_controller` 内のラッチ |
| `ballbotWheelRateFromDisplacement` | `feature/wheel_speed_meter` |
| 指令プロファイル生成 | `feature/command_profile` |
| UART 上書き指令・位置外ループ（新規） | `feature/link_session`, `feature/position_controller`, `feature/odometry` |
| エンコーダ/IMU ドライバ | `device/lsm6dso`, `device/wheel_encoder`, `device/mdd3a_motor` |
| MCU ペリフェラル | `peripheral/*` |

## 安全性

- 動作状態（FR-12）: 起動直後は `standby`（モータ無駆動）とし、`START_OPERATION` の受理で `operating` へ、`STOP_OPERATION` で `standby` へ戻る。`fault`（mode 0 終端相当）は `operating` 中の異常のみで遷移し、`STOP_OPERATION` で `standby` へ復帰する（故障リセット）。待機中に蓄積する推定ドリフトを動作へ持ち込まないため、`standby` → `operating` 遷移時に推定器の動的状態リセット（バイアス・yaw・資格は保持）とオドメトリ宣言位置の再適用を行う。
- E-Stop は GPIO コールバックで即時検出し、イネーブルを落とす。mode 0 は終端で、以後全タスクのトルク出力が 0 に固定される。
- IMU 不在・読取失敗時は推定を停止し mode 0 に移行する（`state_estimator` が健全性フラグを返す）。
- 動作開始時は `startup_torque_ramp_time`（0.4 s）で全トルク経路をランプし、初期トランジェントを抑える。
- リンク安全機構（FR-09）: 最終 `SET_VELOCITY` 受信から 500 ms 無指令で指令をゼロとするウォッチドッグと、UART アイドル 1 s 超過での指令ゼロ・購読リセットを `link_session` が行う。リンク断絶時は `operating` を維持してその場保持とし、`standby` へは遷移しない（転倒防止）。
