#!/usr/bin/env bash
# guard_no_hal.sh — portability guard for App/ HAL includes
#
# PURPOSE:
#   Fail loudly if any App/ source or header includes stm32h7xx_hal.h or
#   main.h outside the two approved exceptions:
#     1. App/port/board_casper2/board_casper2.{c,h}  (the board layer — HAL owner)
#     2. App/radio/sx1276.c                          (Casper-2 radio, replaced by EEbyte on C3)
#
# USAGE (from Software/):
#   bash test/guard_no_hal.sh
#   echo $?   # 0 = PASS, 1 = FAIL
#
# Run this after any edit to App/ to confirm the portability seam is intact.
# It is also run as part of CI or pre-commit checks.

set -euo pipefail

# Resolve the App/ directory relative to this script's location (Software/).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/../App" 2>/dev/null && pwd)" || {
    echo "guard_no_hal: ERROR — cannot find App/ from $SCRIPT_DIR"
    exit 1
}

# Grep for forbidden includes in App/, then strip the three approved exceptions.
VIOLATIONS=$(grep -rn \
    -e '#include[[:space:]]*"stm32h7xx_hal\.h"' \
    -e '#include[[:space:]]*"main\.h"' \
    "$APP_DIR" \
    --include='*.c' --include='*.h' \
    | grep -v 'App/port/board_casper2/board_casper2\.c' \
    | grep -v 'App/port/board_casper2/board_casper2\.h' \
    | grep -v 'App/radio/sx1276\.c' \
    || true)

if [ -n "$VIOLATIONS" ]; then
    echo "guard_no_hal: FAIL — forbidden HAL/main.h includes found in App/:"
    echo "$VIOLATIONS"
    echo ""
    echo "Only the board layer may include stm32h7xx_hal.h or main.h:"
    echo "  App/port/board_casper2/board_casper2.{c,h}  (portability seam — HAL owner)"
    echo "  App/radio/sx1276.c                          (Casper-2 radio exception)"
    echo ""
    echo "Route all HAL access through casper_port.h / board_casper2.h instead."
    exit 1
fi

echo "guard_no_hal: PASS — no forbidden includes in App/"
exit 0
