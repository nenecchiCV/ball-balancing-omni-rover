// feature 層ホストテストのエントリポイント。
#include "test_util.hpp"

namespace bb {
void balance_controller_tests();
void command_profile_tests();
void link_protocol_tests();
void odometry_tests();
void operation_manager_tests();
void position_controller_tests();
void state_estimator_tests();
void wheel_servo_tests();
}  // namespace bb

TEST_MAIN_BEGIN()
TEST_RUN(bb::balance_controller_tests);
TEST_RUN(bb::command_profile_tests);
TEST_RUN(bb::link_protocol_tests);
TEST_RUN(bb::odometry_tests);
TEST_RUN(bb::operation_manager_tests);
TEST_RUN(bb::position_controller_tests);
TEST_RUN(bb::state_estimator_tests);
TEST_RUN(bb::wheel_servo_tests);
TEST_MAIN_END()
