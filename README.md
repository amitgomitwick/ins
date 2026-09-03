# INS — GPS-aided Inertial Navigation System for a UAV

A GPS-aided error-state Extended Kalman Filter (EKF) strapdown INS for a
UAV, in two implementations that share the same architecture and the same
validation approach:

- **[`include/`](include/) / [`src/`](src/) (C++17)** — standalone,
  dependency-free, meant to eventually run *inside* a flight controller
  (see [`docs/ardupilot_integration.md`](docs/ardupilot_integration.md) for
  wiring it into ArduPilot). This is the reference implementation the
  equations are derived against.
- **[`python/`](python/) (Python/numpy)** — a companion-computer port that
  runs on a Raspberry Pi/Jetson-class Linux board and reads real telemetry
  over MAVLink. Real flight controllers run hard-real-time C/C++, so Python
  can't be part of that loop directly — this is what "Python on real
  flight hardware" actually means in practice. See
  [`docs/python_companion_computer.md`](docs/python_companion_computer.md).

Everything below describes the shared architecture using the C++ paths;
the Python port mirrors it file-for-file (`python/ins_ekf/ekf.py` ==
`src/InsEkf.cpp`, etc.) — see `python/README.md` for its specifics.

It estimates, from IMU (accelerometer + gyro) and GPS (position + velocity)
alone:

- **Position** and **velocity**, NED frame
- **Attitude** (roll / pitch / yaw), as a quaternion
- **Gyro bias** and **accelerometer bias**

## Why this exists / how it works

Real IMUs drift: gyros integrate to a wandering attitude, accelerometers
integrate twice to a wildly wandering position, and both have unknown,
slowly-varying biases. GPS alone is noisy, low-rate (typically 5–10 Hz), and
gives no attitude at all. An INS fuses the two: the IMU supplies smooth,
high-rate motion between GPS fixes, and GPS periodically pulls the IMU's
drifting estimate back toward truth.

This implementation uses a 15-state **error-state EKF** (the standard
architecture behind most real flight-controller INS stacks, including
ArduPilot's own EKF3 and PX4's EKF2):

| Nominal state (integrated every IMU sample) | Error state (what the Kalman covariance tracks) |
|---|---|
| position `p` (NED, m) | `δp` (3) |
| velocity `v` (NED, m/s) | `δv` (3) |
| attitude quaternion `q` | `δθ` — small-angle attitude error (3) |
| gyro bias `bg` (rad/s) | `δbg` (3) |
| accel bias `ba` (m/s²) | `δba` (3) |

GPS position/velocity updates are applied as **sequential scalar
updates** (one axis at a time) rather than a block matrix inversion — since
each axis's measurement noise is independent, this is mathematically
equivalent to a joint update but needs no matrix inverse anywhere in the
filter, which is the standard trick for running an EKF like this on
flight-controller-class hardware.

Full derivation of the propagation/measurement equations:
[`docs/architecture.md`](docs/architecture.md).

## Layout

```
include/ins/       C++ public API: Math3 (Vector3/Matrix3/Quaternion),
                    ImuSample, GpsSample, InsEkf
src/InsEkf.cpp      The C++ filter implementation
examples/           FlightScenario.h (synthetic flight generator + sim
                    runner) and sim_flight.cpp (CLI that writes a CSV log)
tests/              Dependency-free sanity/regression tests (ctest-wired)
python/             Companion-computer port -- see python/README.md
docs/               Architecture derivation, ArduPilot integration guide,
                    Python/MAVLink companion-computer guide
```

## Build & run

```sh
mkdir build && cd build
cmake .. && cmake --build .
ctest --output-on-failure    # unit tests
./sim_flight flight_log.csv  # runs the synthetic flight, writes a CSV
```

`sim_flight` flies a synthetic 120-second UAV trajectory (accelerate away →
90 s coordinated turn → decelerate to a stop) through simulated MEMS-grade
IMU noise/bias and 5 Hz noisy GPS, including a 10-second GPS dropout mid-turn
to demonstrate the INS coasting on inertial data alone. It prints RMSE
against the (known, synthetic) ground truth:

```
Position RMSE : ~1 m
Velocity RMSE : ~0.3 m/s
Attitude RMSE : a few degrees (roll/pitch; see note below)
```

## A known, honest limitation: yaw without a magnetometer

This filter has **no magnetometer input**, by design (that wasn't in
scope — see `docs/architecture.md` if you want to add one). Roll and pitch
are reasonably well observed because gravity couples into the accelerometer
measurement. **Yaw has no such direct reference** — it's only corrected
indirectly, through how attitude errors leak into velocity errors during
acceleration/turning, which GPS velocity then corrects. That means yaw
convergence is slower and noisier than roll/pitch, especially before the
vehicle has maneuvered. This matches real GPS/IMU-only systems; adding a
compass (a single extra scalar-update measurement, same pattern as the GPS
updates) is the standard fix and a natural next step.

## Status

This is a validated first version: the propagation and update equations are
implemented and covered by the sanity tests in `tests/`, the full synthetic
flight converges with realistic sensor noise (see
`docs/architecture.md#validation`), and the sign/derivation walkthrough for
every equation is written down so it can be reviewed line-by-line. The
Python port additionally has an end-to-end test that streams the same
synthetic flight as *real* MAVLink UDP packets and decodes them with the
companion tool's actual message-handling code
(`python/tests/test_mavlink_integration.py`) — see
`docs/python_companion_computer.md` for exactly what that does and doesn't
prove. Neither implementation has run on real hardware or against real
ArduPilot/SITL yet — see
[`docs/ardupilot_integration.md`](docs/ardupilot_integration.md) (C++) and
[`docs/python_companion_computer.md`](docs/python_companion_computer.md)
(Python) for what's next.
