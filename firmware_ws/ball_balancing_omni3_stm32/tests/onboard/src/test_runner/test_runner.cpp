#include "test_runner/test_runner.hpp"

#include <string.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include "device/status_led/status_led.hpp"
#include "encoder_test/encoder_test.hpp"
#include "estop_test/estop_test.hpp"
#include "imu_attitude_test/imu_attitude_test.hpp"
#include "motor_deadzone_test/motor_deadzone_test.hpp"
#include "omni_drive_test/omni_drive_test.hpp"
#include "peripheral/control_tick/control_tick.hpp"
#include "peripheral/timebase/timebase.hpp"
#include "tick_timing_test/tick_timing_test.hpp"
#include "uart_echo_test/uart_echo_test.hpp"

namespace bb {

namespace {

// start/stop（shell スレッド）と update（テストスレッド）間の排他。
// K_MUTEX_DEFINE で定数初期化する。
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
K_MUTEX_DEFINE(runner_mutex);

// 登録テストテーブル。テスト追加時はここへ 1 行追加する。
const test_runner::test_entry TESTS[] = {
    {"estop", "estop — E-stop 入力の変化表示", &estop_test::start_cb,
     &estop_test::update_cb, &estop_test::stop_cb, 100U},
    {"tick", "tick — 1 kHz ティック周期の統計", &tick_timing_test::start_cb,
     &tick_timing_test::update_cb, &tick_timing_test::stop_cb, 1U},
    {"imu", "imu — IMU+推定器の姿勢出力", &imu_attitude_test::start_cb,
     &imu_attitude_test::update_cb, &imu_attitude_test::stop_cb, 5U},
    {"enc", "enc [spin [duty]] — エンコーダ方向・カウント表示",
     &encoder_test::start_cb, &encoder_test::update_cb, &encoder_test::stop_cb,
     100U},
    {"omni", "omni <vx> <vy> <yaw> — 速度・回転指令",
     &omni_drive_test::start_cb, &omni_drive_test::update_cb,
     &omni_drive_test::stop_cb, 1U},
    {"deadzone", "deadzone [dir] [max] [sec] — 不感帯同定掃引",
     &motor_deadzone_test::start_cb, &motor_deadzone_test::update_cb,
     &motor_deadzone_test::stop_cb, 1U},
    {"echo", "echo — USART1 バイトエコー（無線変換疎通用）",
     &uart_echo_test::start_cb, &uart_echo_test::update_cb,
     &uart_echo_test::stop_cb, 1000U},
};

const test_runner::test_entry* find_entry(const char* name) {
  for (const auto& e : TESTS) {
    if (strcmp(e.name, name) == 0) return &e;
  }
  return nullptr;
}

int test_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  if (argc < 2) {
    shell_print(sh, "active=%s overrun=%lu isr=%lu loop=%lu upd=%lu phase=%lu",
                test_runner::active() ? test_runner::active_name() : "none",
                static_cast<unsigned long>(test_runner::overrun_count()),
                static_cast<unsigned long>(control_tick::isr_count()),
                static_cast<unsigned long>(test_runner::loop_iters()),
                static_cast<unsigned long>(test_runner::upd_calls()),
                static_cast<unsigned long>(test_runner::phase()));
    shell_print(sh, "usage: test <name> [args...] / test stop");
    for (size_t i = 0; i < test_runner::entry_count(); ++i) {
      const auto& e = test_runner::entries()[i];
      shell_print(sh, "  %s", e.usage);
    }
    return 0;
  }
  if (strcmp(argv[1], "stop") == 0) {
    test_runner::stop();
    shell_print(sh, "stopped");
    return 0;
  }
  const bool ok =
      test_runner::start(argv[1], static_cast<int>(argc - 2), argv + 2);
  if (!ok) {
    shell_error(sh, "test '%s' not found or rejected arguments", argv[1]);
    return -EINVAL;
  }
  shell_print(sh, "started %s", argv[1]);
  return 0;
}

// NOLINTBEGIN: Zephyr の SHELL_* マクロ内部の初期化子・分岐に由来する警告
SHELL_CMD_ARG_REGISTER(test, NULL, "onboard tests (run 'test' for list)",
                       test_cmd_cb, 0, 10);
// NOLINTEND

}  // namespace

const test_runner::test_entry* test_runner::entries() {
  return TESTS;
}

size_t test_runner::entry_count() {
  return sizeof(TESTS) / sizeof(TESTS[0]);
}

bool test_runner::start(const char* name, int argc, char** argv) {
  const test_entry* e = find_entry(name);
  if (e == nullptr) return false;
  k_mutex_lock(&runner_mutex, K_FOREVER);
  apply_stop_locked();
  const bool ok = e->start_cb(argc, argv);
  if (ok) {
    active_ = e;
    phase_ = 0;
  }
  k_mutex_unlock(&runner_mutex);
  return ok;
}

void test_runner::stop() {
  k_mutex_lock(&runner_mutex, K_FOREVER);
  apply_stop_locked();
  k_mutex_unlock(&runner_mutex);
}

void test_runner::request_stop() {
  stop_requested_ = true;
}

bool test_runner::active() {
  return active_ != nullptr;
}

const char* test_runner::active_name() {
  return (active_ != nullptr) ? active_->name : "none";
}

uint32_t test_runner::overrun_count() {
  return overruns_;
}

uint32_t test_runner::loop_iters() {
  return loop_iters_;
}

uint32_t test_runner::upd_calls() {
  return upd_calls_;
}

uint32_t test_runner::phase() {
  return phase_;
}

void test_runner::apply_stop_locked() {
  if (active_ != nullptr && active_->stop_cb != nullptr) {
    active_->stop_cb();
  }
  active_ = nullptr;
  phase_ = 0;
  stop_requested_ = false;
}

void test_runner::run() {
  for (;;) {
    control_tick::wait();
    ++loop_iters_;
    k_mutex_lock(&runner_mutex, K_FOREVER);
    if (stop_requested_) {
      apply_stop_locked();
    }
    if (active_ != nullptr && ++phase_ >= active_->period_ticks) {
      phase_ = 0;
      ++upd_calls_;
      active_->update_cb();
    }
    k_mutex_unlock(&runner_mutex);
    // 実行中に溜まったティックは追いかけずオーバーランとして計数する。
    const int32_t backlog =
        static_cast<int32_t>(k_sem_count_get(control_tick::sem()));
    if (backlog > 0) {
      overruns_ += static_cast<uint32_t>(backlog);
      k_sem_reset(control_tick::sem());
    }
    // 状態 LED の点滅駆動（テスト実行管理の一部として常駐処理する）。
    if (++led_phase_ >= 200U) {
      led_phase_ = 0;
      status_led::update(timebase::millis());
    }
  }
}

}  // namespace bb
