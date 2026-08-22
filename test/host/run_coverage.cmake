# Run gcovr after the coverage target has already rebuilt and run ctest.
#
#   cmake -DREPO_ROOT=... -DBUILD_DIR=... -DGCOV_EXECUTABLE=... -P run_coverage.cmake

if(NOT REPO_ROOT OR NOT BUILD_DIR)
  message(FATAL_ERROR "REPO_ROOT and BUILD_DIR are required")
endif()

find_program(GCOVR gcovr)
if(NOT GCOVR)
  message(FATAL_ERROR "gcovr not found; pip install gcovr")
endif()

file(MAKE_DIRECTORY "${BUILD_DIR}/coverage")

set(_gcov_args)
if(GCOV_EXECUTABLE)
  list(APPEND _gcov_args --gcov-executable "${GCOV_EXECUTABLE}")
endif()

# Run from the repo root so relative --filter lib/|src/|include/ match
# gcovr's relative paths. Search gcda/gcno under the host build tree.
execute_process(
  COMMAND "${GCOVR}"
    --root "${REPO_ROOT}"
    --object-directory "${BUILD_DIR}"
    ${_gcov_args}
    --filter "lib/"
    --filter "src/"
    --filter "include/"
    --exclude "test/"
    --exclude "build/"
    --html-details "${BUILD_DIR}/coverage/index.html"
    --txt "${BUILD_DIR}/coverage/summary.txt"
    --print-summary
    "${BUILD_DIR}"
  WORKING_DIRECTORY "${REPO_ROOT}"
  RESULT_VARIABLE gcovr_rc
)
if(NOT gcovr_rc EQUAL 0)
  message(FATAL_ERROR "gcovr failed (exit ${gcovr_rc})")
endif()
