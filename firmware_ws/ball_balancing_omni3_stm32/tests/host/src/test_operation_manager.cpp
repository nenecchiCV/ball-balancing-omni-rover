// operation_manager ホストテスト: 動作状態遷移・START 受理判定・
// キャリブレーション状態機械・fault 終端（FR-12, FR-13）。
#include "feature/operation_manager/operation_manager.hpp"

#include "common/control_params/control_params.hpp"
#include "common/link_protocol/link_protocol.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

namespace ack = link_protocol::ack_result;
using om = operation_manager;

operation_manager::config cfg() {
  return {&get_params()};
}

operation_manager::start_cond ready_cond() {
  return {true, false, true, 0.0F};
}

void init() {
  operation_manager::init(cfg());
}

void start_rejects_when_not_standby() {
  init();
  // STANDBY 中に受理条件を満たせば OPERATING へ。
  CHECK_TRUE(operation_manager::request_start(ready_cond()) == ack::OK);
  CHECK_TRUE(operation_manager::state() == om::op_state::OPERATING);
  CHECK_TRUE(operation_manager::drives_motors());
  // OPERATING/FAULT からの START は REJECTED_STATE。
  CHECK_TRUE(operation_manager::request_start(ready_cond()) ==
             ack::REJECTED_STATE);
}

void start_checks_conditions() {
  init();
  operation_manager::start_cond c = ready_cond();
  c.imu_ok = false;
  CHECK_TRUE(operation_manager::request_start(c) == ack::NOT_READY);
  c = ready_cond();
  c.estop = true;
  CHECK_TRUE(operation_manager::request_start(c) == ack::NOT_READY);
  c = ready_cond();
  c.qualified = false;
  CHECK_TRUE(operation_manager::request_start(c) == ack::NOT_READY);
  c = ready_cond();
  c.tilt = get_params().controller.fallen_tilt;  // 閾値以上は不可
  CHECK_TRUE(operation_manager::request_start(c) == ack::NOT_READY);
  // 全て拒否されても STANDBY のまま。
  CHECK_TRUE(operation_manager::state() == om::op_state::STANDBY);
  CHECK_TRUE(!operation_manager::drives_motors());
}

void stop_returns_to_standby() {
  init();
  (void)operation_manager::request_start(ready_cond());
  operation_manager::request_stop();
  CHECK_TRUE(operation_manager::state() == om::op_state::STANDBY);
  CHECK_TRUE(!operation_manager::drives_motors());
  // 停止後に再度 START できる（往復遷移）。
  CHECK_TRUE(operation_manager::request_start(ready_cond()) == ack::OK);
  CHECK_TRUE(operation_manager::state() == om::op_state::OPERATING);
}

void fault_is_terminal() {
  init();
  (void)operation_manager::request_start(ready_cond());
  operation_manager::notify_controller_terminal();
  CHECK_TRUE(operation_manager::state() == om::op_state::FAULT);
  CHECK_TRUE(!operation_manager::drives_motors());
  // FAULT からは STOP でも START でも戻れない。
  operation_manager::request_stop();
  CHECK_TRUE(operation_manager::state() == om::op_state::FAULT);
  CHECK_TRUE(operation_manager::request_start(ready_cond()) ==
             ack::REJECTED_STATE);
  // STANDBY 中の終端通知は無視される。
  init();
  operation_manager::notify_controller_terminal();
  CHECK_TRUE(operation_manager::state() == om::op_state::STANDBY);
}

void calib_lifecycle() {
  init();
  // STANDBY 中のみ受理。
  CHECK_TRUE(operation_manager::request_calib(42) == ack::OK);
  CHECK_TRUE(operation_manager::calib() == om::calib_state::RUNNING);
  CHECK_TRUE(operation_manager::calib_seq() == 42);
  // バイアスリセット要求は一度だけ取り出せる。
  CHECK_TRUE(operation_manager::take_bias_reset_req());
  CHECK_TRUE(!operation_manager::take_bias_reset_req());
  // 実施中は -1、資格成立で 0。
  CHECK_TRUE(operation_manager::calib_update(1000, false, true) == -1);
  CHECK_TRUE(operation_manager::calib_update(1500, true, true) == 0);
  CHECK_TRUE(operation_manager::calib() == om::calib_state::DONE);
  // 完了後は -1。
  CHECK_TRUE(operation_manager::calib_update(2000, true, true) == -1);
}

void calib_timeout_and_imu_failure() {
  init();
  (void)operation_manager::request_calib(7);
  // deadline は最初の calib_update の now から計る。
  const uint32_t t0 = 100;
  CHECK_TRUE(operation_manager::calib_update(t0, false, true) == -1);
  const uint32_t deadline = t0 + get_params().link.calib_timeout_ms;
  CHECK_TRUE(operation_manager::calib_update(deadline - 1, false, true) == -1);
  CHECK_TRUE(operation_manager::calib_update(deadline, false, true) == 1);
  CHECK_TRUE(operation_manager::calib() == om::calib_state::FAILED);

  init();
  (void)operation_manager::request_calib(7);
  CHECK_TRUE(operation_manager::calib_update(100, false, false) == 2);
  CHECK_TRUE(operation_manager::calib() == om::calib_state::FAILED);
}

void calib_rejected_while_operating() {
  init();
  (void)operation_manager::request_start(ready_cond());
  CHECK_TRUE(operation_manager::request_calib(1) == ack::REJECTED_STATE);
  CHECK_TRUE(operation_manager::calib() == om::calib_state::NONE);
  // 停止すれば再び受理される。
  operation_manager::request_stop();
  CHECK_TRUE(operation_manager::request_calib(1) == ack::OK);
}

}  // namespace

void operation_manager_tests() {
  start_rejects_when_not_standby();
  start_checks_conditions();
  stop_returns_to_standby();
  fault_is_terminal();
  calib_lifecycle();
  calib_timeout_and_imu_failure();
  calib_rejected_while_operating();
}

}  // namespace bb
