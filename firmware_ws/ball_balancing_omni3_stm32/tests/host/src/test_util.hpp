#pragma once

// 最小限のホストテスト用チェックマクロ（フレームワーク非依存・確保なし）。
#include <math.h>
#include <stdio.h>

namespace test {

inline int& failure_count() {
  static int count = 0;
  return count;
}
inline int& check_count() {
  static int count = 0;
  return count;
}

}  // namespace test

#define CHECK_TRUE(expr)                                                 \
  do {                                                                   \
    ++test::check_count();                                               \
    if (!(expr)) {                                                       \
      ++test::failure_count();                                           \
      printf("FAIL %s:%d: CHECK_TRUE(%s)\n", __FILE__, __LINE__, #expr); \
    }                                                                    \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                      \
  do {                                                                         \
    ++test::check_count();                                                     \
    const float a_ = static_cast<float>(actual);                               \
    const float e_ = static_cast<float>(expected);                             \
    if (!(fabsf(a_ - e_) <= (tol))) {                                          \
      ++test::failure_count();                                                 \
      printf("FAIL %s:%d: CHECK_NEAR(%s=%.6f, %s=%.6f, tol=%.6f)\n", __FILE__, \
             __LINE__, #actual, static_cast<double>(a_), #expected,            \
             static_cast<double>(e_), static_cast<double>(tol));               \
    }                                                                          \
  } while (0)

#define CHECK_FINITE(expr)                                                 \
  do {                                                                     \
    ++test::check_count();                                                 \
    if (!isfinite(static_cast<float>(expr))) {                             \
      ++test::failure_count();                                             \
      printf("FAIL %s:%d: CHECK_FINITE(%s)\n", __FILE__, __LINE__, #expr); \
    }                                                                      \
  } while (0)

#define TEST_MAIN_BEGIN() \
  int main() {            \
    printf("running host feature tests\n");
#define TEST_RUN(fn)                                                    \
  do {                                                                  \
    printf("  %-56s", #fn);                                             \
    const int before_ = test::failure_count();                          \
    fn();                                                               \
    printf("%s\n", test::failure_count() == before_ ? "PASS" : "FAIL"); \
  } while (0)
#define TEST_MAIN_END()                                         \
  printf("done: %d checks, %d failures\n", test::check_count(), \
         test::failure_count());                                \
  return test::failure_count() == 0 ? 0 : 1;                    \
  }
