# C.A.S.P.E.R-3 — Repo Bring-up / Migration Brief

**Read this fully before copying anything.** You are the Claude Code agent working in the **new, empty `C.A.S.P.E.R-3` repo**. Your job is to pull across only the parts of C.A.S.P.E.R.-2 that are **not CubeMX-regenerateable** — primarily the firmware in `Software/App/` (which now sits behind a clean HAL-free *port layer*), plus the supporting docs, tests, tooling, sensor characterization data, and editor/agent config. Everything CubeMX produces is regenerated fresh in this repo, not copied.

This is a **fresh start**. The Casper-2 root reports and much of the doc set come over as *lessons-learnt / reference only* — not as the spec of record for Casper 3.

---

## 0. Source locations

| What | Where |
|---|---|
| Casper-2 working tree (on this machine) | `E:\C.A.S.P.E.R\C.A.S.P.E.R Flight Software V2\Casper 2 flight firmware\C.A.S.P.E.R.-2` |
| Casper-2 private GitHub | `https://github.com/AshwinVishwanath/C.A.S.P.E.R.-2-Private.git` |
| Branch / commit to migrate from | `casper3-port-layer` @ `8afdf9f` |

> **CRITICAL — use the local working tree as the source, not a fresh `git clone`.**
> `.claude/`, `.vscode/`, `*.ioc`, `startup_*.s`, `Middlewares/`, `*.hex`, and all `*.csv`/`*.png` data are **git-ignored** in Casper-2, so they are **NOT in either GitHub repo**. A clone will silently miss them. Copy from the local path above (referred to below as **`$SRC`**).

Let `$SRC = "E:\C.A.S.P.E.R\C.A.S.P.E.R Flight Software V2\Casper 2 flight firmware\C.A.S.P.E.R.-2"`.

---

## 1. The one architectural fact that makes this clean

Casper-2's `casper3-port-layer` branch extracted **all** application firmware behind a port seam:

- `Software/App/port/` — pure C11 headers (`casper_spi.h`, `casper_i2c.h`, `casper_gpio.h`, `casper_qspi.h`, `casper_adc.h`, `casper_pwm.h`, `casper_time.h`, `casper_crc.h`, umbrella `casper_port.h`). **No HAL or CMSIS types anywhere.**
- `Software/App/port/board_casper2/board_casper2.c` — the **only** firmware file that includes `stm32h7xx_hal.h` / `main.h` (plus `App/radio/sx1276.c` by documented exception). All HAL calls live here as one-line wrappers.
- `Software/App/flight/app_main.c` — the portable bring-up + superloop, extracted out of the CubeMX `main.c`.

**Consequence:** `Software/App/` is fully portable. You drop it onto a freshly-generated CubeMX project, write a `board_casper3/board_casper3.c` that implements the same port API against the new `.ioc`'s handles, and the entire nav / telemetry / FSM / pyro / radio stack compiles unchanged. Read `App/port/PORT_SPEC.md` first — it is the contract.

---

## 2. COPY THESE (the keep-set)

Preserve the relative paths unless a "→ dest" is given. Create the directory under the new repo, then copy.

### 2a. Firmware — the core of the migration
```
$SRC/Software/App/                         →  Software/App/      (ENTIRE tree, verbatim)
```
That includes `port/` (with `board_casper2/`), `drivers/`, `nav/`, `flight/`, `fsm/`, `pyro/`, `radio/`, `telemetry/`, `command/`, `cal/`, `logging/`, `pack/`, `diag/`, `buzzer/`, `ground/`, `util/`, `test/`.
> Drop the build artifact `App/test/test_pyro_logic.elf` — do not copy compiled binaries.

### 2b. Build config — copy as REFERENCE (do not assume drop-in)
```
$SRC/Software/Makefile                     →  Software/Makefile
$SRC/Software/STM32H750XX_FLASH.ld         →  Software/STM32H750XX_FLASH.ld
```
The Makefile encodes the hard-won bits: `-DUSE_PWR_LDO_SUPPLY`, `USB_MODE`/`DATA_PHASE`/`TEST_MODE` defines, `ARM_MATH_CM7`, CMSIS-DSP sources + include paths, and the full `C_SOURCES` list for `App/`. You will adapt the CubeMX-generated half (Core/Drivers/Middlewares paths) for Casper-3, but **keep every `App/` source line and every custom define**.

### 2c. CubeMX-dir files that are actually CUSTOM (copy, then re-home after you regenerate)
These live inside CubeMX folders but were hand-written; CubeMX will NOT regenerate them:
```
$SRC/Software/USB_DEVICE/App/usbd_msc_storage_if.{c,h}   (W25Q512JV ↔ USB MSC bridge)
$SRC/Software/FATFS/Target/user_diskio.{c,h}             (FatFs ↔ W25Q512JV bridge)
```
Copy them into a holding folder `casper2-reference/custom-usb-fatfs/` for now. After you regenerate the Casper-3 CubeMX project with USB-MSC + FatFs enabled, fold these back into the generated `USB_DEVICE/`/`FATFS/` trees inside the `USER CODE` markers.

### 2d. User-modified CubeMX files — copy as REFERENCE ONLY (never overwrite fresh output with these)
The Casper-3 CubeMX project must be generated fresh, but you need to see *what edits to re-apply*. Copy these into `casper2-reference/casper2-core/`:
```
$SRC/Software/Core/Src/main.c
$SRC/Software/Core/Src/stm32h7xx_it.c
$SRC/Software/Core/Src/stm32h7xx_hal_msp.c
$SRC/Software/Core/Src/system_stm32h7xx.c
$SRC/Software/Core/Inc/main.h
$SRC/Software/Core/Inc/stm32h7xx_hal_conf.h
$SRC/Software/USB_DEVICE/App/usbd_cdc_if.c
$SRC/Software/USB_DEVICE/Target/usbd_conf.c
$SRC/Software/USB_DEVICE/App/usbd_desc.c
```
These hold the LDO-supply fix, HSI+PLL clock tree, per-SPI DataSize/prescaler/mode overrides, the PC2 MODER fix, EXTI wiring, and the `app_main()` call that replaces the generated superloop. **Re-apply the relevant diffs by hand into Casper-3's freshly generated files** — the pin map and clock tree may differ on the Casper-3 board, so port intent, not bytes.

### 2e. Specification + requirement docs (design references)
```
$SRC/Specification Docs/                   →  Specification Docs/      (EKF, FSM, HARDWARE, INTERFACE, ORIENTATION, PYRO, SENSOR specs)
$SRC/Product Requirement Docs/             →  Product Requirement Docs/   (radio, test-harness, flight-critical, FSM, logging PRDs, L0/L2)
```

### 2f. Host-side test suite + tooling
```
$SRC/Tests/                                →  Tests/    (Unity, tier1-3, compliance, regression, mocks, stubs, scripts)
$SRC/tools/                                →  tools/    (HIL harness, casper_decode.py, sim senders)
```
> Do **not** copy `Tests/build/*.exe`, `tools/**/__pycache__/`, or `Tests/CSVs/*.csv` — these are regenerated by running the suite. Copy source only.

### 2g. Sensor docs + statistical-analysis data (the "keep the LSM6/ADXL/MS5611 docs" ask)
There are **no datasheet PDFs in Casper-2** — they live outside the repo. Create a place for them and bring the analysis outputs that the EKF noise model depends on:
```
# placeholder for the user's external datasheets
Datasheets/README.md   ← create new; list LSM6DSO32, ADXL372, MS5611 (and MAX-M10M, MMC5983MA, SX1276 for completeness) for the user to drop PDFs into

# sensor characterization (Allan-variance / noise-model source of truth)
$SRC/Matlab Code/sensorcal/casper_sensor_characterization.m   →  SensorCharacterization/casper_sensor_characterization.m
$SRC/Matlab Code/sensorcal/noise_params.mat                   →  SensorCharacterization/noise_params.mat
$SRC/Matlab Code/sensorcal/baro_noise_params.mat              →  SensorCharacterization/baro_noise_params.mat
$SRC/Matlab Code/plots/allan_lsm6_accel.png                   →  SensorCharacterization/plots/
$SRC/Matlab Code/plots/allan_lsm6_gyro.png                    →  SensorCharacterization/plots/
$SRC/Matlab Code/plots/allan_adxl372.png                      →  SensorCharacterization/plots/
$SRC/Matlab Code/plots/baro_allan_dev.png                     →  SensorCharacterization/plots/
$SRC/Matlab Code/plots/baro_allan_overlay.png                 →  SensorCharacterization/plots/
$SRC/Matlab Code/plots/qc_noise_histograms.png                →  SensorCharacterization/plots/
```
> This is the carve-out: the broader `Matlab Code/` (Simulink dev, EKF dev, raw CSV captures) is **excluded** per the migration scope — only the sensor noise-characterization artifacts come across, because the EKF `R`/`Q` values are derived from them.

### 2h. Editor + agent config (git-ignored in Casper-2 — must come from `$SRC` on disk)
```
$SRC/.claude/                              →  .claude/   (agents, commands, memory, skills, CLAUDE.md, settings.local.json)
$SRC/.vscode/                              →  .vscode/   (settings.json, tasks.json)
```
> `.claude/CLAUDE.md` and `.claude/memory/` are **Casper-2-specific**. Bring them, but treat as a starting template: update the board (STM32H750VBT6 may change), clock tree, pin map, and "Current Firmware State" for Casper-3 once the new hardware is pinned down. Don't trust the old pin/bus tables until re-verified against the Casper-3 schematic.

### 2i. Repo hygiene
```
$SRC/.gitignore        →  .gitignore     (adapt: it already ignores .ioc, *.s, Middlewares/, build/, *.hex, *.csv, *.png, .claude/)
$SRC/.gitattributes    →  .gitattributes
```
> Note `.gitignore` ignores `.claude/`. Decide deliberately whether Casper-3 should track `.claude/` (force-add) or keep ignoring it as Casper-2 did.

### 2j. Casper-2 root reports — REFERENCE / LESSONS-LEARNT ONLY
```
$SRC/DEV_UPDATE.md           →  casper2-reference/DEV_UPDATE.md
$SRC/OPTIMIZATION_REPORT.md  →  casper2-reference/OPTIMIZATION_REPORT.md
$SRC/README.md               →  casper2-reference/README.md
```
> **These describe Casper 2, not Casper 3.** Park them under `casper2-reference/` with a header note: *"Historical — Casper-2 bring-up log and optimization notes. Kept as lessons-learnt. Not the spec of record for Casper-3; write a new README/DEV log for this repo."* Then author a **fresh** `README.md` at the Casper-3 root.

---

## 3. DO NOT COPY — regenerate fresh from CubeMX in Casper-3

Everything CubeMX owns. Generate a new `.ioc` for the Casper-3 board and let CubeMX emit:
```
Software/Core/            (HAL main.c skeleton, MSP, IT, syscalls, sysmem, system_stm32h7xx, linker stubs)
Software/Drivers/STM32H7xx_HAL_Driver/
Software/FATFS/  +  Software/USB_DEVICE/   (generated halves — middleware glue)
Software/Middlewares/     (git-ignored in C2 anyway; ST USB Device Lib + FatFs)
Software/*.ioc
Software/startup_*.s
```
After regenerating, re-add by hand (CubeMX won't): **CMSIS-DSP** (`Drivers/CMSIS/DSP/` Include + the matrix-function `Source/*.c` files) and the `ARM_MATH_CM7` define — the nav stack needs it. The Casper-2 `Drivers/CMSIS/DSP/` tree is a convenient source to copy that subset from.

Also **excluded entirely** (not regenerated, just not wanted):
```
Software/build/  Software/prebuilt/  Software/*.hex  Software/.mxproject  Software/test/
Matlab Code/          (except the 2g carve-out)
Hardware/             (PCB prints, schematics, MAX-M10M manual — Casper-2 hardware)
CSVs/  Bench Test Data/  Flight Images and Raw Data/    (raw capture data)
```

---

## 4. After-copy checklist (port-in order)

1. `git init` the Casper-3 repo (if not already) and commit the keep-set from §2 as the baseline.
2. Generate the Casper-3 CubeMX project (`.ioc`) for the target MCU; let it emit Core/Drivers/Middlewares/FATFS/USB_DEVICE.
3. Re-add CMSIS-DSP + `ARM_MATH_CM7`; fold in the custom `usbd_msc_storage_if` and `user_diskio` bridges (§2c).
4. Re-apply the Casper-2 Core edits (§2d) into the fresh generated files — **LDO supply, clock tree, SPI DataSize/prescaler/mode, PC2 fix, EXTI, `app_main()` call**. Use Casper-2's `.claude/CLAUDE.md` → "After CubeMX Regeneration" checklist as the line-by-line guide, adjusting pins/clocks for Casper-3 hardware.
5. Write `Software/App/port/board_casper3/board_casper3.c` implementing the port API (`PORT_SPEC.md`) against the new `.ioc` handles. This is the bulk of new work; `board_casper2.c` is your template.
6. Update the Makefile (§2b) for the regenerated tree; keep all `App/` sources + custom defines.
7. `make clean && make -j8` from `Software/`. Resolve until 0 warnings (Casper-2 rule: warnings are build failures).
8. Run `Tests/` host suite to confirm the portable logic survived the move.
9. Author a fresh Casper-3 `README.md`; refresh `.claude/CLAUDE.md` + memory for the new board.

---

## 5. Quick scope summary

| Category | Action |
|---|---|
| `Software/App/` (incl. port seam + board_casper2) | **Copy verbatim** |
| Makefile, linker script | Copy as reference, adapt |
| Custom USB-MSC / FatFs bridges | Copy, re-home after regen |
| User-modified Core / USB files | Copy to `casper2-reference/`, re-apply diffs |
| Spec Docs, PRD Docs, Tests, tools | **Copy (source only)** |
| Sensor characterization (.m/.mat/plots) + `Datasheets/` placeholder | **Copy carve-out** |
| `.claude/`, `.vscode/`, `.gitignore`, `.gitattributes` | **Copy from local disk** |
| Root reports (DEV_UPDATE, OPTIMIZATION_REPORT, README) | Copy to `casper2-reference/`, mark reference-only |
| Core/Drivers/Middlewares/FATFS/USB_DEVICE generated halves, `.ioc`, `startup_*.s` | **Regenerate — do not copy** |
| `Matlab Code/` (bulk), `Hardware/`, CSVs, Bench/Flight data, build artifacts | **Exclude** |
