# C.A.S.P.E.R.-2 Simulink Simulator — Phase 0

This directory contains the **planning bundle** for a Simulink-native 6-DOF simulator that validates the C.A.S.P.E.R.-2 flight computer's navigation stack against a known truth trajectory. It is structured for execution by a Claude Code agent team (one Opus 4.7 manager + multiple Sonnet sub-agents) running inside this firmware repository.

## You are an agent reading this file. What now?

**If you are the Opus 4.7 manager**: read these files in order, then proceed.
1. `ARCHITECTURE.md` — locked architectural decisions. Don't re-litigate.
2. `PHASE0_SPEC.md` — Phase 0 mission, deliverables, and acceptance criteria (the trust gate).
3. `MANAGER_PLAYBOOK.md` — your orchestration playbook. Task DAG, sub-agent delegation, integration protocol.
4. `references/FIRMWARE_CONSTANTS.md` — authoritative table of constants extracted from firmware.
5. `references/SIMULINK_PATTERNS.md` — programmatic Simulink construction idioms.
6. `tasks/T01..T11_*.md` — eleven per-task PRDs you will dispatch to Sonnet sub-agents.

Then start dispatching tasks per the DAG in `MANAGER_PLAYBOOK.md`.

**If you are a Sonnet sub-agent**: do **not** read this file. Read your assigned task file at `tasks/T0X_*.md`, plus the three files it lists in its "References" section. That is your complete scope. Anything outside it is out of scope.

## What this simulator is

A Simulink model that:
- Reads a known truth trajectory (RasAero CSV in Phase 0; 6-DOF EOM in later phases).
- Generates simulated sensor outputs (IMU, ADXL, baro, mag with radio interference, GPS) matching the firmware's hardware sensors bit-for-bit at the byte level where possible.
- Feeds them through a stripped-down MATLAB port of the firmware's ESKF + attitude estimator.
- Compares the estimator's recovered state against truth and produces pass/fail metrics.

## What this simulator is NOT (Phase 0)

- Not a 6-DOF aero forward dynamics simulator. Truth comes from RasAero CSV; vehicle is force-posed along it.
- Not a real-time HIL test bench (no STM32 in the loop).
- Not a flight-firmware identical port. The estimator port is **stripped** — fixed predict rate, no adaptive dt, no safety nets. Quirks are added back in Phase 1+.
- Not authoritative for sensor noise models. Radio→mag interference uses a placeholder ±10 µT rectangular pulse; will be calibrated against bench data later.

## What success looks like for Phase 0

Run the full sim against the provided `Flight_Test.CSV` and recover:
- Apogee within ±10 m of RasAero truth (~31 km)
- Attitude RMS error < 1° during powered flight
- Vertical velocity RMS error < 2 m/s at burnout

If these pass, the sim is "trustworthy" for forward exploration of estimator changes, sensor model refinements, and Phase 2+ aero augmentation work.

## File layout

```
Software/Sim/
├── README.md                          (this file — entry point only)
├── ARCHITECTURE.md                    (locked decisions, frame conventions)
├── PHASE0_SPEC.md                     (mission + acceptance criteria)
├── MANAGER_PLAYBOOK.md                (Opus orchestration playbook)
├── tasks/                             (sub-agent PRDs — one per task)
│   ├── T01_truth_pipeline.md
│   ├── T02_sensor_params.md
│   ├── T03_imu_sensor_model.md
│   ├── T04_baro_sensor_model.md
│   ├── T05_mag_sensor_model.md
│   ├── T06_gps_sensor_model.md
│   ├── T07_frame_switch.md
│   ├── T08_eskf_port.md
│   ├── T09_attitude_port.md
│   ├── T10_validation_block.md
│   └── T11_integration.md
└── references/                        (authoritative reference material)
    ├── FIRMWARE_CONSTANTS.md
    └── SIMULINK_PATTERNS.md
```

All sub-agent build outputs land in `Software/Sim/build/T0X_*/` directories created during execution.

## Hard constraints on the manager and sub-agents

1. **No firmware modifications.** This simulator must not edit anything under `Software/App/`. Constants must be *read* from firmware, never written back.
2. **No silent assumptions about frame conventions.** All frame transformations must be explicit and tested. See `ARCHITECTURE.md` §3.
3. **No skipping the trust gate.** If `T11_integration.md` acceptance criteria do not pass, Phase 0 is incomplete. Do not declare success on partial results.
4. **Reproducibility is non-negotiable.** Fixed seeds, fixed-step solvers, deterministic outputs across runs. See `ARCHITECTURE.md` §6.
5. **No real-time assumptions.** Everything fixed-step, offline.

## Input artifact

The RasAero truth trajectory lives at the repo root or wherever the user has placed it:
- File: `Flight_Test.CSV`
- 12,800 rows, t = 0..549 s, imperial units
- Columns documented in `tasks/T01_truth_pipeline.md`

Locate it before starting; if it is missing, halt and ask the user.

## Upstream context

The flight computer this simulator validates lives in `Software/App/`. Key firmware files referenced repeatedly:
- `Software/App/nav/casper_ekf.[ch]` — 4-state vertical EKF
- `Software/App/nav/casper_attitude.[ch]` — Mahony + RK4 attitude
- `Software/App/nav/casper_quat.[ch]` — Hamilton quaternion math
- `Software/App/drivers/lsm6dso32.[ch]` — primary IMU driver
- `Software/App/drivers/ms5611.[ch]` — barometer driver
- `Software/App/drivers/mmc5983ma.[ch]` — magnetometer driver
- `Software/App/drivers/adxl372.[ch]` — high-g accel driver
- `Software/App/drivers/max_m10m.[ch]` — GPS driver
- `Software/App/cal/mag_cal.[ch]` — magnetometer calibration constants
- `Software/App/nav/temp_cal_coeffs.h` — gyro temperature coefficients
- `Software/App/radio/radio_manager.[ch]` — radio TX state machine (for interference modeling)
- `Software/App/flight/flight_loop.c` — superloop with all rate decisions

Sub-agents should read these directly; do not paraphrase the firmware into the markdown.

## License & ownership

This is internal C.A.S.P.E.R.-2 project tooling. Built by and for Ashwin Vishwanath. Not licensed for external distribution.
