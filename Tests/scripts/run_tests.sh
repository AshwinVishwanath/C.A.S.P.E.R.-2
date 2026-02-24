#!/bin/bash
# run_tests.sh — Build and run C.A.S.P.E.R.-2 test suite
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEST_DIR="$(dirname "$SCRIPT_DIR")"

# Parse args
TIER="${1:-all}"
BUILD_DIR="$TEST_DIR/build"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
cmake "$TEST_DIR" 2>&1

# Build
cmake --build . -j$(nproc 2>/dev/null || echo 4) 2>&1

# Run
if [ "$TIER" = "all" ]; then
    ctest --output-on-failure
elif [ "$TIER" = "coverage" ]; then
    ctest --output-on-failure
    # Coverage report if lcov available
    if command -v lcov &>/dev/null; then
        lcov --capture --directory . --output-file coverage.info 2>/dev/null
        lcov --remove coverage.info '*/stubs/*' '*/unity/*' '*/mocks/*' --output-file filtered.info 2>/dev/null
        if command -v genhtml &>/dev/null; then
            genhtml filtered.info --output-directory coverage_report 2>/dev/null
            echo "Coverage report: $BUILD_DIR/coverage_report/index.html"
        fi
    fi
else
    ctest --output-on-failure -R "$TIER"
fi

echo ""
echo "=== Test run complete ==="
