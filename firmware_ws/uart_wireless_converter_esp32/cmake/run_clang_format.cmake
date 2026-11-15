# src/ 配下へ clang-format --dry-run を実行する。
file(GLOB_RECURSE FILES
  "${SRC_ROOT}/common/*.hpp" "${SRC_ROOT}/common/*.cpp"
  "${SRC_ROOT}/peripheral/*.hpp" "${SRC_ROOT}/peripheral/*.cpp"
  "${SRC_ROOT}/device/*.hpp" "${SRC_ROOT}/device/*.cpp"
  "${SRC_ROOT}/feature/*.hpp" "${SRC_ROOT}/feature/*.cpp"
  "${SRC_ROOT}/manager/*.hpp" "${SRC_ROOT}/manager/*.cpp"
  "${SRC_ROOT}/main.cpp"
)
if(DEFINED TEST_ROOT)
  file(GLOB_RECURSE TEST_FILES "${TEST_ROOT}/*.hpp" "${TEST_ROOT}/*.cpp")
  list(APPEND FILES ${TEST_FILES})
endif()
set(FAILURES 0)
foreach(F ${FILES})
  execute_process(
    COMMAND "${CLANG_FORMAT}" --dry-run --Werror "${F}"
    RESULT_VARIABLE RC OUTPUT_VARIABLE OUT ERROR_VARIABLE ERR
  )
  if(NOT RC EQUAL 0)
    math(EXPR FAILURES "${FAILURES} + 1")
    message(STATUS "clang-format 差分: ${F}\n${ERR}")
  endif()
endforeach()
message(STATUS "clang-format: ${FAILURES} ファイルに差分あり")
if(FAILURES GREATER 0)
  message(FATAL_ERROR "clang-format の差分が残っています")
endif()
