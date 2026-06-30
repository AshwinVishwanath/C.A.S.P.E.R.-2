# SIMULINK_PATTERNS.md — Programmatic Construction Idioms

Sub-agents must build Simulink models programmatically (script-driven), not by hand in the GUI. This makes the build reproducible, diffable in git, and agent-executable. This file documents the patterns. No code in this file — patterns and recipes only. Sub-agents implement the patterns in their `build_*.m` scripts.

## 1. Why programmatic

A `.slx` file is a zip of XML. Hand-editing is fragile. Hand-building in the GUI gives no audit trail. A `build_*.m` script that constructs the model from scratch:
- Produces identical `.slx` every run (with a stable sort of block adds — Simulink's auto-layout can vary, so don't rely on visual identity, only on functional identity).
- Lets us regenerate after a MATLAB version change.
- Lets reviewers read the construction logic in MATLAB, not click through XML.

## 2. Top-level build pattern

Every Simulink-emitting task follows this shape for its `build_*.m` script:

1. Close any open instance of the target model (`bdclose` if it's loaded).
2. Delete the existing `.slx` on disk if regenerating.
3. Create a new system with `new_system`.
4. Set top-level model parameters (solver, fixed-step size, stop time, save format).
5. Add blocks one at a time. Use stable names (no anonymous `Subsystem1`).
6. Set per-block parameters with `set_param`.
7. Add lines with `add_line`. Use port names where possible (`'block/PortName'`).
8. Arrange visually (optional cosmetic step — `Simulink.BlockDiagram.arrangeSystem`).
9. Save the system with `save_system` to a known path.
10. Close the system. Done.

Step 5 must be deterministic. Use a fixed iteration order. Do not use `containers.Map` for block lists (unordered iteration).

## 3. Block construction APIs (cheat sheet)

These are the MATLAB function calls sub-agents will use. Do not memorize syntax — look it up in the MATLAB docs at build time.

| Operation | Function |
|---|---|
| Create new model | `new_system` |
| Open a library block path | `'simulink/Sources/Constant'`, `'aeroblks/Environment/.../COESA Atmosphere Model'`, etc. |
| Add a block | `add_block(source_path, dest_path)` |
| Set a block parameter | `set_param(block_path, 'Parameter', value)` |
| Add a connecting line | `add_line(model, src_port, dst_port)` or by block names with `'block_name/PortName'` |
| Add a subsystem | `add_block('built-in/Subsystem', path)` |
| Add an input/output port to a subsystem | `add_block('built-in/Inport', path)` / `Outport` |
| Add a MATLAB Function block | `add_block('simulink/User-Defined Functions/MATLAB Function', path)`, then set its `Script` parameter |
| Set a Rate Transition block | `add_block('simulink/Signal Attributes/Rate Transition', path)`, then `set_param(..., 'OutPortSampleTime', '...')` |
| Save the model | `save_system(model_handle, file_path)` |
| Close without saving | `bdclose(model_name)` |

For the full vocabulary, the canonical reference is the MATLAB documentation: `web('docid:simulink_doc.mw_p_simulink_programmatic_modeling')`.

## 4. Sample times and rate transitions

Sample times are first-class citizens in this build. Get them wrong and the model runs but produces nonsense.

### 4.1 Top-level solver
- Fixed-step `ode4` (Runge-Kutta 4)
- Fixed step size: `1e-4` seconds (10 kHz)
- Stop time: configurable via `casper_sim_config.m`, typical 549 s for full RasAero trajectory.

### 4.2 Per-block sample times
Set the `SampleTime` parameter on every signal source explicitly:
- Truth source: `-1` (inherit, runs at solver rate, 10 kHz)
- IMU output: `1/833`
- ADXL output: `1/800` post-launch (use a switched-rate subsystem variant)
- Baro output: `1/100` (nominal; firmware ~100 Hz)
- Mag output: `1/100`
- GPS output: `1/10`
- EKF predict: `1/416` (matches firmware EKF_DT = 0.0024)
- Radio TX event: `1/10`

### 4.3 Rate Transition blocks
Every cross-rate signal must go through an explicit Rate Transition block. Configuration:
- For sensor-to-estimator signals: `OutPortSampleTime` = explicit, `Integrity` = on (deterministic data transfer).
- Use deterministic data transfer mode for safety-critical paths.
- Do not rely on Simulink's automatic rate transition insertion.

## 5. MATLAB Function block guidelines

When using a MATLAB Function block (which most sensor models will), follow:

1. Specify input and output signal types and sizes in the function declaration (Edit Data dialog or programmatically).
2. The function script lives in the model itself (not as an external file) unless the same function is used by multiple blocks. For shared functions, place them on the MATLAB path and call them from inside the MATLAB Function block.
3. Inputs that are vectors must declare exact size: `accel_body` is `double(3,1)`, never `double(:,1)`.
4. Outputs same: `mag_uT_body` is `double(3,1)`.
5. Random number generation inside MATLAB Function blocks: use a `persistent` RandStream initialized on the first call with a seed derived from `Sim.Seed`. **Do not** call `rng(...)` inside the block — it has global side effects.
6. Hand-code linear algebra (3×3 matmul, etc.) — do not call `*` on small matrices when a written-out form is faster and more deterministic.

## 6. Sensor block construction template

A typical sensor block has this structure:

```
Subsystem "<sensor>_block"
├── Input ports (from truth bus)
│   - position_NED      (3×1)
│   - velocity_NED      (3×1)
│   - accel_NED         (3×1)
│   - attitude_quat_std (4×1)
│   - body_rates_std    (3×1)
│   - time_s            (scalar)
├── Internal blocks
│   ├── Rate Transition (down-sample truth to sensor rate)
│   ├── MATLAB Function: <sensor>_model
│   │   (truth in, ideal-sensor-output computation)
│   ├── MATLAB Function: <sensor>_noise
│   │   (add ARW, bias drift, quantization)
│   ├── Saturation block (model sensor range)
│   └── (optional) Rate Transition out
└── Output ports
    - <sensor>_measurement (size depends on sensor)
    - <sensor>_flags       (status word matching firmware)
```

Subsystems are saved as library blocks in `casper_sim_lib.slx` when they will be reused; otherwise as inline subsystems in the model.

## 7. Bus signals

For the truth bus, define a `Simulink.Bus` object in `casper_sim_config.m`:

- `TruthBus`:
  - `pos_NED` (3×1 double)
  - `vel_NED` (3×1 double)
  - `accel_NED` (3×1 double)
  - `quat_std` (4×1 double, scalar-first)
  - `omega_body_std` (3×1 double)
  - `time_s` (scalar double)
  - `mach` (scalar double)
  - `air_density_kgm3` (scalar double)
  - `air_temp_K` (scalar double)
  - `air_pressure_pa` (scalar double)

Bus objects let the truth signal flow as a single line into each sensor subsystem. Sub-agents must not deviate from this bus layout; if a new field is needed, escalate to manager.

## 8. Persistent state in MATLAB Function blocks

Sensor models that have memory (bias random walk, AR(1) noise, EKF state) use `persistent` variables. Pattern:

1. First call: persistent variables are empty. Initialize from parameters passed via constant block or workspace variable.
2. Subsequent calls: read previous values, update, store back.
3. To reset state between sim runs, use Simulink's "Initialize Function" callback on the subsystem.

**Reset on every run**: every block with persistent state must implement a reset path so two runs with the same seed produce identical outputs. Sub-agents must include a "reset test" in their unit test scripts (run twice in one MATLAB session, check outputs match byte-for-byte).

## 9. Parameter passing

Parameters reach blocks via the base workspace. The convention:

- Top-level: `casper_sim_config.m` populates a struct `Sim` and per-sensor structs `IMU`, `Baro`, `Mag`, `GPS`, `Estimator`.
- These are placed in the base workspace before `sim()` is called.
- Blocks reference them by name: `set_param(block, 'Gain', 'Mag.HardIron(1)')`, etc.
- Bus objects similarly: `Simulink.Bus.cellToObject({TruthBus_def}, 'TruthBus')` is called from `casper_sim_config.m`.

Sub-agents must not hard-code numerical values inside MATLAB Function block scripts. All values come from struct parameters, which are mapped through the Block Parameters dialog (programmatically: `set_param(block, 'ParameterArgumentNames', '...')`).

## 10. Save format & file system layout

- `.slx` files: save with `save_system(handle, '/full/path/to/file.slx')`. Always use absolute paths in `build_*.m` scripts.
- `.mat` files: use `-v7.3` format if they will contain > 2 GB; otherwise `-v7` is fine and more diff-friendly.
- Intermediate truth trajectory: `truth_trajectory.mat` in `T01_truth_pipeline/`, format `-v7`.
- Each task's outputs live in `Software/Sim/build/T0X_*/`. Do not pollute task directories with unrelated files.

## 11. Common Simulink pitfalls

### 11.1 Anonymous block names
Avoid. Always pass a third argument to `add_block` to give the block a stable name. Otherwise `add_block(src, 'mymodel/Subsystem')` collides with auto-generated names.

### 11.2 Line direction
`add_line(model, 'src_block/port', 'dst_block/port')`. The port can be a number ('1') or a port name (for buses, `'PortName'`). Lines are directional.

### 11.3 Signal naming
Always set `set_param(line_handle, 'Name', 'descriptive_name')` for the top-level signals between subsystems. Makes the model readable.

### 11.4 Sample time inheritance
A block with `SampleTime = -1` inherits from its driver. For Simulink to compile, every signal must have a resolvable sample time. If you see `Could not propagate sample time` errors, find the source — usually a constant block without an explicit `SampleTime`.

### 11.5 Algebraic loops
The estimator feeds attitude back into the IMU rotation. This creates an algebraic loop unless broken by a unit delay (Z⁻¹) or a memory block. Phase 0 breaks the loop because truth attitude (not estimated) feeds the sensor models — but verify there are no other loops in the estimator port.

### 11.6 Random seed scoping
`imuSensor`, `magnetometer`, `gpsSensor` objects: the seed is set on the object, not via Simulink workspace. Set them in the block's `Initialize Function` callback.

## 12. Validation patterns

Every block produced by a sub-agent must come with a `test_*.m` that:

1. Loads parameters from `casper_sim_config.m`.
2. Constructs the block-under-test (calls the relevant `build_*.m`).
3. Drives it with a deterministic input (e.g., constant pad pose for 1 s).
4. Captures the output (run via `sim()`, log via `'SaveOutput', 'on'`).
5. Asserts that the output matches expectations to a specified tolerance.
6. Prints PASS or FAIL with details.

Test scripts are invoked by the manager during verification (`MANAGER_PLAYBOOK.md` §4 step 3). They must run in < 30 seconds each.

## 13. Documentation

Every `build_*.m` and `test_*.m` script must have:

- H1 line: one-line summary.
- Synopsis: how to call (no arguments expected for `build_*` and `test_*`).
- Output description: what files / model handles it produces.
- Source firmware reference: which firmware file(s) it parallels, if applicable.
- A `STATUS.md` in the task's build directory that captures the test result and any deviations.

## 14. A note on hand-editing

A sub-agent may, in extremis, hand-edit the `.slx` via the GUI for diagnostic purposes (e.g., to confirm a hypothesis fast). Any change made this way **must** be reflected back in the `build_*.m` script before STATUS.md is written. The script is the source of truth; the `.slx` is a build artifact.

Re-running the `build_*.m` script must produce a functionally identical `.slx` to the GUI-edited version. If it doesn't, the script is wrong, not the GUI.
