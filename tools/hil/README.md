# HIL host-side framework

Drives a CASPER-2 firmware running with `-DHIL_MODE` from a real
flight trajectory. Generates raw-sensor packets (IMU, baro, mag, GPS,
high-G, pyro continuity, battery) at the rates the real drivers see,
streams them over USB CDC, and parses the firmware's COBS telemetry
that comes back so the operator can watch the FSM walk through
`PAD → BOOST → COAST → APOGEE → DROGUE → MAIN → LANDED` exactly as it
would on a real flight.

The first scenario, `openrocket_hpr`, is built straight from
`CSVs/HIL Sim Data.csv` — an OpenRocket export of an HPR flight to
40 km apogee.

## Quick start

```bash
# 1. Build firmware with HIL gating
cd Software && make hil
#    → flash build/FlightComputer/Casper2_Flight.hex via DFU

# 2. From the repo root, run a scenario
python -m tools.hil.runner --port COM3 --scenario openrocket_hpr

# Speed up for development; sensor cadences scale with speed
python -m tools.hil.runner --port COM3 --scenario openrocket_hpr --speed 5.0

# Truncate the flight to the first N seconds (great for FSM debugging)
python -m tools.hil.runner --port COM3 --scenario openrocket_hpr --duration 30

# Dry run — list packet counts without opening the port
python -m tools.hil.runner --scenario openrocket_hpr --dry-run
```

`pyserial` is required for live runs (`pip install pyserial`). Dry
runs need only the standard library.

## What the runner does

For every iteration of the scenario it produces a virtual time
`t_v` and a packet. The runner sleeps `t_v / speed` wall-seconds from
the start, then writes the COBS-framed bytes to the serial port.
Between writes it polls the port for incoming bytes, runs them
through the COBS decoder, and dispatches each frame to the right
parser (`FAST`, `GPS`, `EVENT`). Recognised frames are printed live —
state transitions show up as `FSM → BOOST` lines with timestamps,
events as `EVENT PYRO ch0 dur≈1000ms`, and a once-per-second
heartbeat keeps the operator in the loop without scrolling through
833 IMU packets per second.

At the end the runner prints a summary: packet counts, FSM history,
events grouped by type. That summary is the first signal of
"did the run pass?". For deeper validation, dump the QSPI flash via
USB MSC and decode it with `tools/casper_decode.py`.

## Architecture

```
tools/hil/
├── framing.py      COBS encode/decode + CRC-32 (matches firmware)
├── protocol.py     pack/parse for 0xD3/0xD4/0xD5 + FAST/GPS/EVENT
├── trajectory.py   loader for OpenRocket-style CSVs
├── sensor_models.py  body-frame projection, ISA atmosphere, noise
├── runner.py       paced TX + parallel RX + event log
└── scenarios/
    ├── __init__.py
    └── openrocket_hpr.py
```

A scenario is just a module that exposes:

```python
SCENARIO_NAME = "..."
@dataclass
class ScenarioArgs: ...
def build_packets(args) -> Iterator[(virtual_time_s, framed_bytes, kind)]:
    ...
```

The runner does not care what's inside `build_packets`. New scenarios
can be a 50-line file using `Trajectory` + `synthesise_*` helpers, or
something more elaborate that drives 6-DOF dynamics, drogue failures,
GPS dropouts, sensor faults, etc.

## Coordinate convention

The firmware uses **body Y = nose, up on pad** (see
`flight_loop.c`). On the pad a stationary rocket reads
`accel = (0, +9.81, 0) m/s²` because the accelerometer measures
specific force, which equals `kinematic - gravity_vector`. During
free-fall (motor off, in coast or apogee) it reads zero. During
boost it reads `(0, +g + thrust/mass, 0)`.

`sensor_models.synthesise_imu_baro_mag` enforces this convention:

| Phase                     | accel body-Y reading              |
| ------------------------- | --------------------------------- |
| On rail (thrust < 50 N)   | +g (clean, sensor-at-rest)        |
| Boost / coast / descent   | OpenRocket vert_acc + g           |

The on-rail kludge exists because OpenRocket reports the kinematic
acceleration as `-g` while the rocket is constrained — that's not
what an accelerometer would measure. Without the override, the
Mahony filter fails to acquire a stable gravity vector during the
30 s pad calibration.

## Pacing

Windows' `time.sleep` has 1–15 ms jitter depending on the system
timer. For 833 Hz IMU pacing the runner uses `time.perf_counter()`
plus a busy-spin tail of 200 µs. The firmware's EKF dt comes from
the packet `tick_ms` field (virtual time), not wall clock, so any
host-side jitter affects throughput but not numerical accuracy.

## Sanity checks before plugging in hardware

```bash
# Verify packet sizes match the firmware constants
python -c "from tools.hil.protocol import *; \
           print(len(pack_hil_raw(0,[0,0,0],[0,0,0],0,[0,0,0])), \
                 len(pack_hil_aux(0)), \
                 len(pack_hil_adxl(0,0,0,0)))"
# → 50 33 16

# Spot-check sensor synthesis at key flight phases
python -c "from tools.hil.trajectory import Trajectory; \
           from tools.hil.sensor_models import synthesise_imu_baro_mag, NoiseModel; \
           t = Trajectory('CSVs/HIL Sim Data.csv'); \
           print(synthesise_imu_baro_mag(t.state(94.2), NoiseModel()).accel_ms2)"
# → near (0, 0, 0) at apogee — free fall, sensor reads 0 g
```

## Limits / future work

- **Single trajectory, no scenarios yet.** The next batch (drogue
  failure, tilted launch, GPS dropout, two-stage) drops in as
  additional modules under `scenarios/`.
- **Lateral motion is dropped.** OpenRocket's lateral velocity column
  is small for vertical flights, but a 6-DOF scenario should compose
  the body-to-NED rotation properly.
- **No assertion DSL.** Pass/fail is "the operator sees the FSM walk
  the right path with the right events." Adding a declarative
  `expect_state(BOOST, at_t=(0.4, 1.5))` API is straightforward once
  we know which assertions matter most in practice.
- **No replay mode.** Reconstructing sensor packets from a real flight
  log would let us debug field bugs on the bench. Same scaffolding,
  different `Trajectory` source.
