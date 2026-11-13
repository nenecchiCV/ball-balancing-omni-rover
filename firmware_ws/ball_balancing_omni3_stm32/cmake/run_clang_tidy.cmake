# compile_commands.json に含まれる src/ 配下の翻訳単位へ clang-tidy を実行する。
# Zephyr ビルドの GCC 固有フラグは clang が解釈できないため、
# フラグ除去済みの compile DB を <db>-tidy/compile_commands.json に生成する。
# 使い方: cmake -DCLANG_TIDY=... -DCOMPILE_COMMANDS=... -DSRC_ROOT=... -P run_clang_tidy.cmake
file(READ "${COMPILE_COMMANDS}" COMP_DB)

# clang が解釈できない GCC 専用フラグを除去する。
foreach(FLAG
        "-fno-reorder-functions"
        "-fno-defer-pop"
        "-mfp16-format=ieee"
        "-mtp=soft"
        "-mabi=aapcs"
        "--param=min-pagesize=0"
        "-specs=picolibc.specs"
        "-fcheck-new"
        "-fno-asynchronous-unwind-tables"
        "-Wexpansion-to-defined"
        "-Wno-format-zero-length"
        "-Wdouble-promotion")
  string(REPLACE " ${FLAG}" "" COMP_DB "${COMP_DB}")
  string(REPLACE " ${FLAG} " " " COMP_DB "${COMP_DB}")
endforeach()
get_filename_component(DB_DIR "${COMPILE_COMMANDS}" DIRECTORY)
set(TIDY_DB_DIR "${DB_DIR}-tidy")
file(MAKE_DIRECTORY "${TIDY_DB_DIR}")
file(WRITE "${TIDY_DB_DIR}/compile_commands.json" "${COMP_DB}")

string(REGEX MATCHALL "\"file\": \"[^\"]+\"" FILE_ENTRIES "${COMP_DB}")
set(FAILURES 0)
set(COUNT 0)
foreach(ENTRY ${FILE_ENTRIES})
  string(REGEX REPLACE "\"file\": \"([^\"]+)\"" "\\1" FILE_PATH "${ENTRY}")
  if(FILE_PATH MATCHES "${SRC_ROOT}")
    math(EXPR COUNT "${COUNT} + 1")
    execute_process(
      COMMAND "${CLANG_TIDY}" -p "${TIDY_DB_DIR}"
              "--extra-arg=-Wno-deprecated-volatile"
              "--extra-arg=-Wno-unknown-attributes"
              "--extra-arg=-Wno-ignored-optimization-argument"
              "${FILE_PATH}"
      RESULT_VARIABLE RC
      OUTPUT_VARIABLE OUT
      ERROR_VARIABLE ERR
    )
    if(OUT MATCHES "warning:|error:" OR NOT RC EQUAL 0)
      math(EXPR FAILURES "${FAILURES} + 1")
      message(STATUS "clang-tidy 指摘: ${FILE_PATH}")
      message(STATUS "${OUT}${ERR}")
    endif()
  endif()
endforeach()
message(STATUS "clang-tidy: ${COUNT} ファイル解析、${FAILURES} 件に指摘あり")
if(FAILURES GREATER 0)
  message(FATAL_ERROR "clang-tidy で指摘が残っています")
endif()
