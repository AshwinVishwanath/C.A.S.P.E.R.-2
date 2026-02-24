#!/bin/bash
# coverage.sh — Generate coverage report
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEST_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$TEST_DIR/build"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake "$TEST_DIR" -DCMAKE_C_FLAGS="--coverage -fprofile-arcs -ftest-coverage"
cmake --build . -j$(nproc 2>/dev/null || echo 4)
ctest --output-on-failure

if command -v lcov &>/dev/null; then
    lcov --capture --directory . --output-file coverage.info
    lcov --remove coverage.info '*/stubs/*' '*/unity/*' '*/mocks/*' --output-file filtered.info
    genhtml filtered.info --output-directory coverage_report
    echo "Report: $BUILD_DIR/coverage_report/index.html"
else
    echo "lcov not found — run gcov manually"
    find . -name "*.gcno" -exec gcov {} \;
fi
