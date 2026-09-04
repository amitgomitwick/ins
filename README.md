# INS — GPS+magnetometer-aided Inertial Navigation System for a UAV

A standalone, dependency-free C++17 implementation of a strapdown inertial
navigation system (INS) with a GPS- and magnetometer-aided error-state
Extended Kalman Filter (EKF), built to be dropped into an
[ArduPilot](https://ardupilot.org/) firmware tree as a custom estimator
module. See [`docs/ardupilot_integration.md`](docs/ardupilot_integration.md)
for exactly how to wire it in.

It estimates, from IMU (accelerometer + gyro), GPS (position + velocity),
and magnetometer (heading) input:

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

GPS position/velocity and magnetometer heading are all fused as
**sequential scalar updates** (one axis/quantity at a time) rather than a
block matrix inversion — since each measurement's noise is modeled as
independent, this is mathematically exact and needs no matrix inverse
anywhere in the filter, which is the standard trick for running an EKF like
this on flight-controller-class hardware.

Full derivation of the propagation/measurement equations:
[`docs/architecture.md`](docs/architecture.md).

## Layout

```
include/ins/       Public API: Math3 (Vector3/Matrix3/Quaternion), ImuSample,
                    GpsSample, MagSample, InsEkf
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
IMU noise/bias, 5 Hz noisy GPS (with a 10-second dropout mid-turn), and a
noisy magnetometer at a nonzero declination. It prints RMSE against the
(known, synthetic) ground truth — typical numbers across seeds:

```
Position RMSE   : ~0.7–2 m
Velocity RMSE   : ~0.3–0.6 m/s
Roll/pitch RMSE : ~1.3–2.5 deg
Yaw RMSE        : ~1.2–2.5 deg (magnetometer-aided)
```

## Yaw, and the magnetometer fix

GPS alone never observes attitude directly — it corrects attitude only
*indirectly*, through how attitude errors leak into velocity errors during
acceleration/turning. That works reasonably for roll/pitch (gravity couples
straight into the accelerometer) but leaves yaw weakly and slowly observed
without an independent heading reference (an earlier version of this filter
had exactly that gap — yaw RMSE around 8° in the same synthetic flight).

`InsEkf::fuseMag()` fixes this: it tilt-compensates a magnetometer reading
using the filter's current roll/pitch estimate, derives a heading, corrects
it for local magnetic declination, and fuses it as one more scalar update —
the same pattern as the GPS updates, just observing the yaw error state
directly instead of indirectly. See `docs/architecture.md` for the full
derivation and before/after validation numbers.

**This requires setting `InsEkfConfig::mag_declination_rad`** to your
location's actual magnetic declination (e.g. from a WMM/NOAA calculator —
the same value ArduPilot itself uses) — left at the default 0, yaw will be a
magnetic-north estimate offset from true north by however wrong that is.
Only the magnetometer's *direction* is used (not calibrated magnitude), but
hard-iron/soft-iron offsets still need to be calibrated out upstream, same
as any other compass consumer.

## GPS as a helper, not a source of truth

Every GPS (and magnetometer) update passes an innovation gate before it's
allowed to correct anything: the filter checks whether the measurement is
*plausible* given its current uncertainty, not just "close enough" by some
fixed distance, and rejects it if not (`InsEkfConfig::innovation_gate_sigma`).
A channel that's been rejected for too long (`gate_reset_timeout_s`) is
forced through anyway, so a channel that's *actually* drifted — like GPS
reacquiring after a real jamming outage — still resyncs rather than
staying permanently distrusted. Full derivation, and the honest limit of
this (it catches a sudden bad fix, not a patient spoof that drifts slowly
enough to never look implausible), in `docs/architecture.md`'s
"Measurement gating" section.

## Status

This is a validated implementation: the propagation and update equations
(GPS and magnetometer) are covered by the sanity tests in `tests/`, the full
synthetic flight converges with realistic sensor noise on every state
(see `docs/architecture.md#validation`), and the sign/derivation walkthrough
for every equation is written down so it can be reviewed line-by-line. It has
**not** run on real hardware or against a real ArduPilot build yet — that
integration work is scoped out in
[`docs/ardupilot_integration.md`](docs/ardupilot_integration.md).
