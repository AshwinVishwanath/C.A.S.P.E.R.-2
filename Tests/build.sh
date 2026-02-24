#!/bin/bash
# C.A.S.P.E.R.-2 Test Harness Build Script
# Bypasses make to avoid MSYS2/MinGW temp directory issues
# Usage: bash build.sh [test|clean]

set -e

# Fix temp directory for MinGW gcc (MSYS2 sets TMP=/tmp which gcc.exe can't use)
_WINUSER="${USERNAME:-${USER:-Ashwin}}"
export TMP="C:\\Users\\${_WINUSER}\\AppData\\Local\\Temp"
export TEMP="$TMP"

# Ensure MinGW64 toolchain is on PATH (needed when running from PowerShell/CMD)
if ! command -v gcc &>/dev/null; then
    for _p in "/c/msys64/mingw64/bin" "/c/msys64/usr/bin" \
              "C:/msys64/mingw64/bin" "C:/msys64/usr/bin"; do
        [ -x "$_p/gcc.exe" ] || [ -x "$_p/gcc" ] && export PATH="$_p:$PATH"
    done
    if ! command -v gcc &>/dev/null; then
        echo "ERROR: gcc not found. Install MSYS2 MinGW64 or add gcc to PATH."
        exit 1
    fi
fi

CC="${CC:-gcc}"
CFLAGS="-std=c11 -Wall -Wextra -DUNIT_TEST -DARM_MATH_CM7"
CFLAGS="$CFLAGS -Wno-unused-parameter -Wno-unused-variable -Wno-missing-field-initializers"
CFLAGS="$CFLAGS -Wno-sign-compare -Wno-builtin-declaration-mismatch"
LDFLAGS="-lm"

TEST_DIR="."
FW_ROOT="../Software"
APP_ROOT="$FW_ROOT/App"
BUILD_DIR="build"

INCLUDES="-I$TEST_DIR/stubs -I$TEST_DIR/mocks -I$TEST_DIR/unity -I$TEST_DIR"
INCLUDES="$INCLUDES -I$APP_ROOT/nav -I$APP_ROOT/command -I$APP_ROOT/pyro"
INCLUDES="$INCLUDES -I$APP_ROOT/pack -I$APP_ROOT/telemetry -I$APP_ROOT/fsm"
INCLUDES="$INCLUDES -I$APP_ROOT/flight -I$APP_ROOT/drivers -I$APP_ROOT/cal"
INCLUDES="$INCLUDES -I$APP_ROOT/diag -I$APP_ROOT/util -I$TEST_DIR/regression"

# Source groups
UNITY="unity/unity.c"
STUBS="stubs/stub_helpers.c stubs/arm_math_stub.c stubs/global_handles.c"
MOCK_TICK="mocks/mock_tick.c"
MOCK_GPIO="mocks/mock_gpio.c"
MOCK_ADC="mocks/mock_adc.c"
# Always link all mocks (MinGW can't do weak symbol override)
COMMON="$UNITY $STUBS $MOCK_TICK $MOCK_GPIO $MOCK_ADC"
CSV_LOADER="regression/csv_loader.c"
REG_HELPERS="regression/regression_helpers.c"

QUAT="$APP_ROOT/nav/casper_quat.c"
EKF="$APP_ROOT/nav/casper_ekf.c"
ATT="$APP_ROOT/nav/casper_attitude.c"
GYRO_INT="$APP_ROOT/nav/casper_gyro_int.c"
COBS="$APP_ROOT/telemetry/cobs.c"
CRC="$APP_ROOT/telemetry/crc32_hw.c"
STATUS="$APP_ROOT/pack/status_pack.c"
QPACK="$APP_ROOT/pack/quat_pack.c"
MS5611="$APP_ROOT/drivers/ms5611.c"
FSM="$APP_ROOT/fsm/flight_fsm.c"
PYRO_MGR="$APP_ROOT/pyro/pyro_manager.c"
PYRO_DRV="$APP_ROOT/pyro/casper_pyro.c"
CAC="$APP_ROOT/command/cac_handler.c"
CMD="$APP_ROOT/command/cmd_router.c"

if [ "$1" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned."
    exit 0
fi

mkdir -p "$BUILD_DIR"

PASS=0
FAIL=0
TOTAL=0
FAILED_TARGETS=""

compile() {
    local name="$1"
    shift
    TOTAL=$((TOTAL+1))
    echo -n "  Building $name... "
    if $CC $CFLAGS $INCLUDES -o "$BUILD_DIR/$name" "$@" $LDFLAGS 2>>"$BUILD_DIR/errors.log"; then
        echo "OK"
        PASS=$((PASS+1))
    else
        echo "FAIL"
        FAIL=$((FAIL+1))
        FAILED_TARGETS="$FAILED_TARGETS $name"
        echo "    --- errors ---"
        tail -20 "$BUILD_DIR/errors.log" | head -15
        echo "    ---"
    fi
}

echo "=== Building C.A.S.P.E.R.-2 Test Suite ==="
echo ""
> "$BUILD_DIR/errors.log"

# ---- TIER 1: Pure Math ----
echo "-- Tier 1: Unit Tests --"
compile test_casper_quat     tier1_unit/test_casper_quat.c $QUAT $COMMON
compile test_casper_ekf      tier1_unit/test_casper_ekf.c $EKF $QUAT $COMMON
compile test_casper_attitude tier1_unit/test_casper_attitude.c $ATT $QUAT $COMMON
compile test_cobs            tier1_unit/test_cobs.c $COBS $COMMON
compile test_crc32           tier1_unit/test_crc32.c $CRC $COMMON
compile test_status_pack     tier1_unit/test_status_pack.c $STATUS $COMMON
compile test_ms5611_math     tier1_unit/test_ms5611_math.c $MS5611 $COMMON
compile test_tlm_pack        tier1_unit/test_tlm_pack.c $QPACK $STATUS $QUAT $COMMON

if [ -f "$GYRO_INT" ]; then
    compile test_casper_gyro_int tier1_unit/test_casper_gyro_int.c $GYRO_INT $QUAT $COMMON
fi

# ---- TIER 2: Protocol/FSM ----
echo ""
echo "-- Tier 2: Protocol Tests --"
compile test_telemetry_framing tier2_protocol/test_telemetry_framing.c $COBS $CRC $COMMON

if [ -f "$FSM" ]; then
    compile test_flight_fsm tier2_protocol/test_flight_fsm.c $FSM $COBS $CRC $COMMON
fi
if [ -f "$PYRO_MGR" ]; then
    compile test_pyro_manager tier2_protocol/test_pyro_manager.c $PYRO_MGR $PYRO_DRV $COMMON
fi
if [ -f "$CAC" ]; then
    compile test_cac_handler tier2_protocol/test_cac_handler.c $CAC $COBS $CRC $PYRO_MGR $PYRO_DRV $FSM $COMMON
fi
if [ -f "$CMD" ]; then
    compile test_cmd_router tier2_protocol/test_cmd_router.c $CMD $COBS $CRC $COMMON
fi

# ---- TIER 3: Integration ----
echo ""
echo "-- Tier 3: Integration Tests --"
compile test_nav_pipeline tier3_integration/test_nav_pipeline.c $EKF $ATT $QUAT $COMMON
compile test_flight_loop  tier3_integration/test_flight_loop.c $EKF $ATT $QUAT $COMMON

# ---- REGRESSION ----
echo ""
echo "-- Regression Tests --"
compile test_ekf_regression       regression/test_ekf_regression.c $EKF $ATT $QUAT $CSV_LOADER $REG_HELPERS $COMMON
compile test_attitude_regression  regression/test_attitude_regression.c $ATT $QUAT $CSV_LOADER $REG_HELPERS $COMMON

# ---- COMPLIANCE ----
echo ""
echo "-- Compliance Tests --"
compile test_ekf_spec           compliance/test_ekf_spec.c $EKF $ATT $QUAT $COMMON
compile test_interface_spec     compliance/test_interface_spec.c $COMMON
compile test_sensor_spec        compliance/test_sensor_spec.c $COMMON
compile test_pyro_spec          compliance/test_pyro_spec.c $COMMON
compile test_naming_convention  compliance/test_naming_convention.c $COMMON

echo ""
echo "=== Build Results: $PASS/$TOTAL compiled, $FAIL failed ==="
if [ $FAIL -gt 0 ]; then
    echo "Failed targets:$FAILED_TARGETS"
    echo "Full error log: $BUILD_DIR/errors.log"
fi

# ---- Run tests if requested ----
if [ "$1" = "test" ] && [ $FAIL -eq 0 ]; then
    echo ""
    echo "=== Running C.A.S.P.E.R.-2 Test Suite ==="
    TPASS=0; TFAIL=0; TTOTAL=0
    for exe in "$BUILD_DIR"/test_*.exe; do
        [ -x "$exe" ] || continue
        TTOTAL=$((TTOTAL+1))
        name=$(basename "$exe" .exe)
        if "$exe" > /dev/null 2>&1; then
            TPASS=$((TPASS+1))
            echo "  PASS: $name"
        else
            TFAIL=$((TFAIL+1))
            echo "  FAIL: $name"
            "$exe" 2>&1 | grep -i "fail" | head -5
        fi
    done
    echo ""
    echo "=== Test Results: $TPASS/$TTOTAL passed, $TFAIL failed ==="
fi

exit $FAIL
