# Simulink Development Tree Reorg — 2026-05-21

Replaced the per-task `build/T0X_*/` layout with a flat, function-grouped
tree and a single top-level entry point (`casper.m`). Every test that
PASSED before still PASSES after the reorg (see Verification below).

## Old -> New mapping

| Old path                                     | New path                              |
|----------------------------------------------|---------------------------------------|
| `build/T01_truth_pipeline/`                  | `truth/`                              |
| `build/T02_sensor_params/`                   | `params/`                             |
| `build/T03_imu_sensor_model/`                | `sensors/imu/`                        |
| `build/T04_baro_sensor_model/`               | `sensors/baro/`                       |
| `build/T05_mag_sensor_model/`                | `sensors/mag/`                        |
| `build/T06_gps_sensor_model/`                | `sensors/gps/`                        |
| `build/T07_frame_switch/`                    | `nav/frame_switch/`                   |
| `build/T08_eskf_port/`                       | `nav/eskf/`                           |
| `build/T09_attitude_port/`                   | `nav/attitude/`                       |
| `build/T10_validation_block/`                | `validation/`                         |
| `build/T11_integration/`                     | `integration/`                        |
| `build/casper_plot_style.m`                  | `shared/casper_plot_style.m`          |
| `build/test_casper_plot_style.m`             | `shared/test_casper_plot_style.m`     |
| `build/plots/` (style smoke)                 | `shared/plots/`                       |
| `build/T11_integration/casper_sim_config.m`         | `shared/casper_sim_config.m`         |
| `build/T11_integration/casper_build_unified_buses.m`| `shared/casper_build_unified_buses.m`|
| `build/T11_integration/casper_load_truth_ts.m`      | `shared/casper_load_truth_ts.m`      |
| `build/T11_integration/casper_setup_visual.m`       | `shared/casper_setup_visual.m`       |
| `MarkDown Claude Docs/`                      | `docs/`                               |
| `MarkDown Claude Docs/tasks/T0X_*.md`        | `docs/tasks/<name>.md` (T0X_ prefix stripped) |
| `CSVs/Flight Test.CSV`                       | `inputs/Flight_Test.CSV` (space -> underscore) |

Top-level directories after reorg:
```
Matlab Code/Simulink Development/
├── casper.m              <- NEW top-level entry point
├── docs/
├── inputs/
├── shared/               <- cross-cutting helpers + plot style
├── truth/
├── params/
├── sensors/{imu,baro,mag,gps}/
├── nav/{frame_switch,attitude,eskf}/
├── validation/
└── integration/
```

## What `casper.m` does

Single entry point that the user runs first. In order:
1. Adds every functional subdir to the MATLAB path.
2. Calls `casper_sim_config` (sensor params + 6 unified `Simulink.Bus`
   objects -> base workspace).
3. Applies visual-model overrides: GPS launch-site augmentation + rate
   snap (IMU/ADXL 833 -> 1000 Hz, EKF 416 -> 500 Hz). Legacy MATLAB
   driver path is unaffected.
4. Loads `truth_trajectory.mat` into base WS as `truth_ts`. Regenerates
   from `inputs/Flight_Test.CSV` if `'Regenerate', true`.
5. Applies tuning-knob overrides (see TUNING SECTION in `casper.m`).
6. Prints a ready-to-go summary.

Usage:
```matlab
cd('Matlab Code/Simulink Development')
casper()                       % defaults: seed 20260519, 5 s smoke
casper('StopTime', 549.0)      % full trajectory
casper('Seed', 20260520)       % different seed
casper('Regenerate', true)     % force truth re-run
casper('Tuning', 'pad')        % preset hook ('pad' | 'flight' | '')
cfg = casper(...);             % returns the config struct
```

The TUNING section inside `casper.m` exposes day-to-day knobs (IMU noise
sigmas, baro Mach-shock magnitude, mag radio-interference amplitude,
GPS CEP, EKF Q/R, Mahony Kp/Ki, Mach-gate hysteresis). Edit a field's
value (currently `[]` = use firmware default) and re-run.

## Path-reference updates

All MATLAB code paths now resolve via:
```matlab
here    = fileparts(mfilename('fullpath'));   % .../<my-subdir>
simroot = fileparts(here);                    % .../Simulink Development
%   (or fileparts(fileparts(here)) for sensors/<x>/ and nav/<x>/)
addpath(fullfile(simroot, 'shared'));
addpath(fullfile(simroot, 'truth'));
addpath(fullfile(simroot, 'params'));
%   ... etc
```

The CSV input is now `simroot/inputs/Flight_Test.CSV` (was
`CSVs/Flight Test.CSV` with a space).

`casper_sim_config.cfg.Plots_OutDir` / `Logs_OutDir` / `Data_OutDir` now
write to `integration/{plots,logs,data}/` (same content as before, the
trustgate report and 12-plot bundle still land in the integration dir).

The legacy MATLAB-driver byte-exact regression path
(`casper_phase0_run.m` / `run_phase0_trustgate.m` /
`build_casper_sim_phase0_legacy.m`) is unchanged in semantics; only the
truth-mat load path was rewritten from
`fullfile(fileparts(here), 'T01_truth_pipeline', 'truth_trajectory.mat')`
to `fullfile(simroot, 'truth', 'truth_trajectory.mat')`.

## .gitignore changes

Three rules were generalized from `Matlab Code/Simulink Development/build/**/...`
to `Matlab Code/Simulink Development/**/...`:
* `.mat` files (regeneratable caches)
* `.slxc` files (Simulink JIT cache)
* `slprj/` directories (Simulink JIT cache)
* `.png` deliverables (exception kept; new paths now match)

A new exception was added so the input CSV is tracked again:
```
!Matlab Code/Simulink Development/inputs/*.CSV
!Matlab Code/Simulink Development/inputs/*.csv
```

(The old `CSVs/` rule still ignores any future CSVs under that name.)

## Verification

All 10 tests requested in the manager dispatch were re-run after the reorg
via `matlab -batch ...`. All PASS:

| # | Test (run from)                                | Result        |
|---|------------------------------------------------|---------------|
| 1 | `cd Simulink Development; casper('StopTime', 5.0)` | PASS (top-level setup completes) |
| 2 | `cd integration; test_visual_model_compile`    | PASS (4/4)    |
| 3 | `cd sensors/imu; test_imu_quirks`              | PASS (11/11)  |
| 4 | `cd sensors/baro; test_baro_quirks`            | PASS (13/13)  |
| 5 | `cd sensors/mag; test_mag_quirks`              | PASS (11/11)  |
| 6 | `cd sensors/gps; test_gps_quirks`              | PASS (26/26)  |
| 7 | `cd sensors/imu; test_imu_block_visual_compile`| PASS          |
| 8 | `cd sensors/baro; test_baro_block_visual_compile`| PASS        |
| 9 | `cd sensors/mag; test_mag_block_visual_compile`| PASS (11/11)  |
|10 | `cd sensors/gps; test_gps_block_visual_compile`| PASS (5/5)    |

All git moves used PowerShell `Move-Item` followed by `git add -A` so git's
rename detection (`-M`) preserves history. 210 file renames detected in
`git status -M`; 0 unintentional deletes; no logic changes to tests or
models.
