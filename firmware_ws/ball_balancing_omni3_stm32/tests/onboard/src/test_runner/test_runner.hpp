#pragma once

#include <cstddef>
#include <cstdint>

namespace bb {

// オンボードテストの実行管理（static class）。
// shell コマンド `test` からテストを選択・開始し、1 kHz 制御ティックに
// 同期してアクティブテストの update_cb を呼び出す。
// 各テストコンポーネントは start_cb/update_cb/stop_cb を実装し、
// TESTS_ テーブルへ登録する（継承は使わず関数ポインタで結合する）。
class test_runner {
 public:
  test_runner() = delete;

  struct test_entry {
    const char* name;   // shell からの起動名（`test <name>`）
    const char* usage;  // `test` 一覧表示用の説明
    // 開始要求。引数は `test <name>` 以降の shell 引数。
    // true で開始受理。shell コンテキストから mutex 保持下で呼ばれる。
    bool (*start_cb)(int argc, char** argv);
    // 制御ティックごとに period_ticks 回に 1 回呼ばれる。テストスレッド
    // （main）コンテキスト、mutex 保持下。nullptr 不可。
    void (*update_cb)();
    // 停止時の後始末（モータ停止等）。nullptr 可。
    void (*stop_cb)();
    // update_cb の呼出周期（ティック数）。1 = 1 kHz。
    uint32_t period_ticks;
  };

  // shell コンテキストから呼ぶ。name に一致するテストを開始する。
  // 実行中テストがあれば先に停止する。
  static bool start(const char* name, int argc, char** argv);
  // アクティブテストを停止する（stop_cb 呼出）。
  static void stop();
  // テスト自身の update_cb 内から停止を要求する。次ティックで停止する。
  static void request_stop();
  static bool active();
  static const char* active_name();
  static uint32_t overrun_count();

  // 登録テーブル（`test` 一覧表示用）。
  static const test_entry* entries();
  static size_t entry_count();

  // main コンテキストの実行ループ。戻らない。
  [[noreturn]] static void run();

 private:
  // mutex 保持下で呼ぶこと。
  static void apply_stop_locked();

  static inline const test_entry* active_ = nullptr;
  static inline uint32_t phase_ = 0;
  static inline uint32_t overruns_ = 0;
  static inline uint32_t led_phase_ = 0;
  static inline bool stop_requested_ = false;
};

}  // namespace bb
