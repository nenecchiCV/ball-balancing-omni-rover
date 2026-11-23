# 詳細設計索引

詳細設計書は `spec/detail_design/<層>/<component>.md` に 1 コンポーネント 1 ファイルで配置する。`src/` のディレクトリ構造はこの層構造に対応する。

## 層とソース構造の対応

| 層 | 詳細設計ディレクトリ | ソースディレクトリ |
|---|---|---|
| common | `common/` | `src/common/` |
| peripheral | `peripheral/` | `src/peripheral/` |
| device | `device/` | `src/device/` |
| feature | `feature/` | `src/feature/` |
| manager | `manager/` | `src/manager/` |

`src/main.cpp` は `src/` 直下。

### オンボードテスト（別ビルド）

実機検証用のテストコンポーネントは `tests/onboard/src/<component>/` に配置する。
これは別の Zephyr アプリケーションであり、`src/` の層構造には含めない。
各テストの仕様（手順・ログ形式・合否基準）は `spec/test/` に定義する。

| 層 | 仕様ディレクトリ | ソースディレクトリ |
|---|---|---|
| test（オンボード） | `spec/test/` | `tests/onboard/src/` |

## コンポーネント一覧

### common
- [vector_math](common/vector_math.md)
- [control_params](common/control_params.md)
- [imu_sample](common/imu_sample.md)
- [link_protocol](common/link_protocol.md)
- [param_store](common/param_store.md)

### peripheral
- [gpio_pin](peripheral/gpio_pin.md)
- [pwm_output](peripheral/pwm_output.md)
- [encoder_counter](peripheral/encoder_counter.md)
- [control_tick](peripheral/control_tick.md)
- [timebase](peripheral/timebase.md)
- [debug_uart](peripheral/debug_uart.md)
- [link_uart](peripheral/link_uart.md)
- [flash_store](peripheral/flash_store.md)

### device
- [lsm6dso](device/lsm6dso.md)
- [mdd3a_motor](device/mdd3a_motor.md)
- [wheel_encoder](device/wheel_encoder.md)
- [estop_input](device/estop_input.md)
- [status_led](device/status_led.md)

### feature
- [wheel_speed_meter](feature/wheel_speed_meter.md)
- [state_estimator](feature/state_estimator.md)
- [balance_controller](feature/balance_controller.md)
- [wheel_servo](feature/wheel_servo.md)
- [command_profile](feature/command_profile.md)
- [link_session](feature/link_session.md)
- [operation_manager](feature/operation_manager.md)
- [tune_deadzone](feature/tune_deadzone.md)
- [odometry](feature/odometry.md)
- [position_controller](feature/position_controller.md)

### manager
- [task_scheduler](manager/task_scheduler.md)
- [robot_application](manager/robot_application.md)

## インターフェース記法

各ファイルは「公開型」「公開関数（シグネチャ + 説明）」「内部状態」「MATLAB との対応・適応事項」の節で構成する。関数の実装詳細は文章で説明し、コード断片は必要最小限とする。
