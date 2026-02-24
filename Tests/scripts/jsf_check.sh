#!/bin/bash
# jsf_check.sh — JSF AV C++ subset compliance checker
# Returns 0 if no undeviated HARD FAIL violations
# Returns 1 if any HARD FAIL without JSF-DEV marker
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
SCAN_DIR="$REPO_ROOT/Software/App"

HARD_FAIL=0
WARNINGS=0

echo "=== JSF AV Compliance Check ==="
echo "Scanning: $SCAN_DIR"
echo ""

# Helper: check for JSF-DEV within 10 lines
check_deviation() {
    local file="$1"
    local line="$2"
    local start=$((line > 10 ? line - 10 : 1))
    local end=$((line + 10))
    if sed -n "${start},${end}p" "$file" 2>/dev/null | grep -q 'JSF-DEV'; then
        return 0  # deviation documented
    fi
    return 1  # no deviation
}

# HARD FAIL: No dynamic allocation
echo "--- Checking: No dynamic allocation ---"
while IFS=: read -r file line content; do
    [ -z "$file" ] && continue
    if check_deviation "$file" "$line"; then
        echo "WARN: $file:$line: $content (deviation documented)"
        WARNINGS=$((WARNINGS+1))
    else
        echo "FAIL: $file:$line: $content"
        HARD_FAIL=$((HARD_FAIL+1))
    fi
done < <(grep -rn '\bmalloc\b\|\bcalloc\b\|\brealloc\b\|\bfree(' "$SCAN_DIR" 2>/dev/null || true)

# HARD FAIL: No setjmp/longjmp
echo "--- Checking: No setjmp/longjmp ---"
while IFS=: read -r file line content; do
    [ -z "$file" ] && continue
    if check_deviation "$file" "$line"; then
        echo "WARN: $file:$line: $content (deviation documented)"
        WARNINGS=$((WARNINGS+1))
    else
        echo "FAIL: $file:$line: $content"
        HARD_FAIL=$((HARD_FAIL+1))
    fi
done < <(grep -rn '\bsetjmp\b\|\blongjmp\b' "$SCAN_DIR" 2>/dev/null || true)

# HARD FAIL: No atoi/atof/atol
echo "--- Checking: No atoi/atof/atol ---"
while IFS=: read -r file line content; do
    [ -z "$file" ] && continue
    if check_deviation "$file" "$line"; then
        echo "WARN: $file:$line: $content (deviation documented)"
        WARNINGS=$((WARNINGS+1))
    else
        echo "FAIL: $file:$line: $content"
        HARD_FAIL=$((HARD_FAIL+1))
    fi
done < <(grep -rn '\batoi\b\|\batof\b\|\batol\b' "$SCAN_DIR" 2>/dev/null || true)

# HARD FAIL: No goto
echo "--- Checking: No goto ---"
while IFS=: read -r file line content; do
    [ -z "$file" ] && continue
    if check_deviation "$file" "$line"; then
        echo "WARN: $file:$line: $content (deviation documented)"
        WARNINGS=$((WARNINGS+1))
    else
        echo "FAIL: $file:$line: $content"
        HARD_FAIL=$((HARD_FAIL+1))
    fi
done < <(grep -rn '\bgoto\b' "$SCAN_DIR" 2>/dev/null || true)

# HARD FAIL: Include guards
echo "--- Checking: Include guards ---"
for header in $(find "$SCAN_DIR" -name "*.h" 2>/dev/null); do
    if ! grep -q '#ifndef\|#pragma once' "$header" 2>/dev/null; then
        echo "FAIL: $header: Missing include guard"
        HARD_FAIL=$((HARD_FAIL+1))
    fi
done

# SOFT WARN: Function length (awk-based)
echo "--- Checking: Function length (warn > 200 SLOC) ---"
for src in $(find "$SCAN_DIR" -name "*.c" 2>/dev/null); do
    awk '
    /^[a-zA-Z_].*\(.*\)/ && !/;/ { fname=$0; start=NR; depth=0 }
    /{/ { depth++ }
    /}/ { depth--; if (depth==0 && start>0) { len=NR-start; if (len>200) printf "WARN: %s:%d: Function %d lines (>200)\n", FILENAME, start, len; if (len>400) printf "FAIL: %s:%d: Function %d lines (>400)\n", FILENAME, start, len; start=0 } }
    ' "$src"
done

echo ""
echo "=== JSF Summary ==="
echo "Hard failures: $HARD_FAIL"
echo "Warnings: $WARNINGS"
exit $HARD_FAIL
