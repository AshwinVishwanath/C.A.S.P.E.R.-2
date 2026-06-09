# run.ps1 — build & run all host-side characterization tests for pure flight modules.
# Uses MSYS2 mingw64 gcc. Run from the Software/ directory:  powershell -File test/run.ps1
$ErrorActionPreference = "Stop"
$env:Path = "C:\msys64\mingw64\bin;" + $env:Path

$root = Split-Path -Parent $PSScriptRoot   # Software/
Set-Location $root

$CC   = "gcc"
$CFLAGS = @("-std=c11","-O2","-Wall","-Wextra","-Itest",
            "-IApp/telemetry","-IApp/logging","-IApp/pack","-IApp/nav",
            "-IApp/fsm","-IApp/pyro","-IApp/util")

# Each entry: test driver + the module source(s) under test.
$suites = @(
  @{ name="cobs";        src=@("test/test_cobs.c","App/telemetry/cobs.c") },
  @{ name="hamming";     src=@("test/test_hamming.c","App/logging/hamming.c") },
  @{ name="quat_pack";   src=@("test/test_quat_pack.c","App/pack/quat_pack.c") },
  @{ name="status_pack"; src=@("test/test_status_pack.c","App/pack/status_pack.c") },
  @{ name="casper_quat"; src=@("test/test_casper_quat.c","App/nav/casper_quat.c") },
  @{ name="endian";      src=@("test/test_endian.c") },
  @{ name="casper_attitude"; src=@("test/test_casper_attitude.c","App/nav/casper_attitude.c","App/nav/casper_quat.c") }
)

$fail = 0
foreach ($s in $suites) {
  $exe = "test/bin_$($s.name).exe"
  & $CC @CFLAGS @($s.src) -lm -o $exe
  if ($LASTEXITCODE -ne 0) { Write-Output "BUILD FAIL: $($s.name)"; $fail = 1; continue }
  Write-Output "==== suite: $($s.name) ===="
  & ".\$exe"
  if ($LASTEXITCODE -ne 0) { $fail = 1 }
}
Write-Output ""
if ($fail -ne 0) { Write-Output "RESULT: SOME TESTS FAILED"; exit 1 }
else { Write-Output "RESULT: ALL TESTS PASSED"; exit 0 }