#include "test_application/test_application.hpp"

// オンボード検証用ファームウェア。実機ソフトとは別ビルドで、
// shell コマンド `test` から各検証テストを実行する。
// ハードウェア初期化は devicetree + Zephyr ドライバの SYS 初期化で
// 完了している。ここでは各 static class を初期化してテスト実行
// ループ（control_tick 1 kHz 駆動）を開始するだけ。

int main() {
  bb::test_application::init();
  bb::test_application::run();
}
