# INS — GPS-aided Inertial Navigation System for a UAV

A standalone, dependency-free C++17 implementation of a strapdown inertial
navigation system (INS) with GPS-aided error-state Extended Kalman Filter
(EKF) fusion, built to be dropped into an [ArduPilot](https://ardupilot.org/)
firmware tree as a custom estimator module. See
[`docs/ardupilot_integration.md`](docs/ardupilot_integration.md) for exactly
how to wire it in.

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
include/ins/       Public API: Math3 (Vector3/Matrix3/Quaternion), ImuSample,
                    GpsSample, InsEkf
src/InsEkf.cpp      The filter implementation
examples/           FlightScenario.h (synthetic flight generator + sim
                    runner) and sim_flight.cpp (CLI that writes a CSV log)
tests/              Dependency-free sanity/regression tests (ctest-wired)
docs/               Architecture derivation + ArduPilot integration guide
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
every equation is written down so it can be reviewed line-by-line. It has
**not** run on real hardware or against a real ArduPilot build yet — that
integration work is scoped out in
[`docs/ardupilot_integration.md`](docs/ardupilot_integration.md).
