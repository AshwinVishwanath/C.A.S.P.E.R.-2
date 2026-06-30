# run.ps1 — build & run host-side characterization tests for pure flight modules.
# Uses MSYS2 mingw64 gcc. Run from the Software/ directory:  powershell -File test/run.ps1
#
# Optional: -Only <name>       run only the named suite(s), comma-separated.
#           -Only "a,b,c"      quotes optional; spaces around commas ok.
# Examples:
#   powershell -File test/run.ps1                  # run ALL suites
#   powershell -File test/run.ps1 -Only endian     # single suite
#   powershell -File test/run.ps1 -Only "crc32,buzzer,lsm6dso32"
param(
    [string]$Only = ""
)

$ErrorActionPreference = "Stop"
$env:Path = "C:\msys64\mingw64\bin;" + $env:Path

$root = Split-Path -Parent $PSScriptRoot   # Software/
Set-Location $root

$CC   = "gcc"
$CFLAGS = @("-std=c11","-O2","-Wall","-Wextra","-Itest",
            "-IApp/port",
            "-IApp/telemetry","-IApp/logging","-IApp/pack","-IApp/nav",
            "-IApp/fsm","-IApp/pyro","-IApp/util")

# Each entry: name, src list, and optional extra include dirs (xtra).
# All new driver suites include test/board_mock.c and the module source under App/.
# board_casper2.c and any HAL file are NEVER included here.
#
# Note: suites for not-yet-migrated modules build-fail by design (RED state).
# Use -Only <name> to run a single suite while others are still red.
$suites = @(
  # ── existing pure-logic suites (no board mock needed) ────────────────────────
  @{ name="cobs";        src=@("test/test_cobs.c","App/telemetry/cobs.c") },
  @{ name="hamming";     src=@("test/test_hamming.c","App/logging/hamming.c") },
  @{ name="quat_pack";   src=@("test/test_quat_pack.c","App/pack/quat_pack.c") },
  @{ name="status_pack"; src=@("test/test_status_pack.c","App/pack/status_pack.c") },
  @{ name="casper_quat"; src=@("test/test_casper_quat.c","App/nav/casper_quat.c") },
  @{ name="endian";      src=@("test/test_endian.c") },
  @{ name="casper_attitude"; src=@("test/test_casper_attitude.c","App/nav/casper_attitude.c","App/nav/casper_quat.c") },

  # ── CRC-32 ───────────────────────────────────────────────────────────────────
  # CRC implementation lives in board_mock.c (casper_crc32_init/compute).
  # The test pulls in casper_crc.h via -IApp/port; no separate App source needed.
  @{ name="crc32";
     src=@("test/test_crc32.c","test/board_mock.c") },

  # ── Buzzer ───────────────────────────────────────────────────────────────────
  # App/buzzer/buzzer.c + casper_pwm seam (via board_mock).
  @{ name="buzzer";
     src=@("test/test_buzzer.c","App/buzzer/buzzer.c","test/board_mock.c");
     xtra=@("-IApp/buzzer") },

  # ── SPI drivers ──────────────────────────────────────────────────────────────
  @{ name="lsm6dso32";
     src=@("test/test_lsm6dso32.c","App/drivers/lsm6dso32.c","test/board_mock.c");
     xtra=@("-IApp/drivers") },

  @{ name="ms5611";
     src=@("test/test_ms5611.c","App/drivers/ms5611.c","test/board_mock.c");
     xtra=@("-IApp/drivers") },

  @{ name="adxl372";
     src=@("test/test_adxl372.c","App/drivers/adxl372.c","test/board_mock.c");
     xtra=@("-IApp/drivers") },

  # ── I2C drivers ──────────────────────────────────────────────────────────────
  @{ name="max_m10m";
     src=@("test/test_max_m10m.c","App/drivers/max_m10m.c","test/board_mock.c");
     xtra=@("-IApp/drivers") },

  @{ name="mmc5983ma";
     src=@("test/test_mmc5983ma.c","App/drivers/mmc5983ma.c","test/board_mock.c");
     xtra=@("-IApp/drivers") },

  # ── Pyro ADC path ────────────────────────────────────────────────────────────
  # Tests casper_pyro.c only (pyro_manager.c is FSM-level, tested separately later).
  @{ name="pyro_adc";
     src=@("test/test_pyro_adc.c","App/pyro/casper_pyro.c","test/board_mock.c") },

  # ── QSPI flash ───────────────────────────────────────────────────────────────
  @{ name="qspi_flash";
     src=@("test/test_qspi_flash.c","App/drivers/w25q512jv.c","test/board_mock.c");
     xtra=@("-IApp/drivers") }
)

# ── -Only filter ─────────────────────────────────────────────────────────────
# Parse comma-separated names; trim whitespace; empty string means "all".
if ($Only -ne "") {
    $wanted = ($Only -split ",") | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne "" }
    $suites = $suites | Where-Object { $wanted -contains $_.name }
    if ($suites.Count -eq 0) {
        Write-Output "ERROR: -Only '$Only' matched no suites."
        exit 1
    }
}

# ── Build & run ──────────────────────────────────────────────────────────────
$fail = 0
foreach ($s in $suites) {
  $exe = "test/bin_$($s.name).exe"
  # Force array context: a single-element xtra collapses to a scalar string,
  # and splatting a string iterates it char-by-char (mangling -IApp/buzzer).
  $extra = @(if ($s.ContainsKey("xtra")) { $s.xtra } else { @() })
  & $CC @CFLAGS @extra @($s.src) -lm -o $exe
  if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL: $($s.name)"; $fail = 1; continue }
  Write-Output "==== suite: $($s.name) ===="
  & ".\$exe"
  if ($LASTEXITCODE -ne 0) { $fail = 1 }
}
Write-Output ""
if ($fail -ne 0) { Write-Output "RESULT: SOME TESTS FAILED"; exit 1 }
else { Write-Output "RESULT: ALL TESTS PASSED"; exit 0 }
