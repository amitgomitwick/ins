# Architecture & derivation

## Conventions

- **Frame**: NED (North-East-Down), local tangent plane, fixed origin at
  wherever `InsEkf::init()` is called.
- **Body frame**: X forward, Y right, Z down (ArduPilot/aerospace standard).
- **Attitude** `q`: unit quaternion, body → NED (`v_ned = q.rotate(v_body)`).
- **Gravity**: `g_ned = [0, 0, +g]` — NED Z is down, and gravity pulls
  down, so the gravity *acceleration* vector points in +Z.
- **Specific force**: an accelerometer measures specific force
  `f = a_true − g_ned` (not true acceleration) — this is why a stationary,
  level accelerometer reads `f_body = [0, 0, −g]`: it senses the normal
  force holding it up, which is the *opposite* of gravity. Rearranged:
  `a_true = R·f_body + g_ned`, which is exactly the mechanization equation
  used in `InsEkf::predict()`.

## Nominal-state strapdown mechanization

Every IMU sample, at rate `dt`:

```
gyro  = ω_meas − bg                     (bias-corrected body rate)
accel = f_meas − ba                     (bias-corrected specific force)

q ← q ⊗ exp(gyro · dt)                  (quaternion integration)
R  = q.toMatrix()                       (body → NED, post-update)

a_ned = R·accel + g_ned                 (true NED acceleration)
v ← v + a_ned · dt
p ← p + v_old · dt + ½·a_ned·dt²
```

Using the *post-update* attitude to rotate the specific force (rather than
a proper midpoint/trapezoidal average of old and new attitude) is a
first-order simplification. At typical IMU rates (100–400 Hz) the resulting
error is `O(dt²)` and is dominated by sensor noise; a production port could
upgrade this to a midpoint or RK4 integrator without changing the filter's
architecture.

## Error-state kinematics

Define the error state relative to the nominal state:

```
δp = p_true − p_nominal
δv = v_true − v_nominal
δθ : small-angle rotation vector such that q_true ≈ q_nominal ⊗ exp(δθ)   (body-frame error)
δbg = bg_true − bg_nominal
δba = ba_true − ba_nominal
```

Linearizing the mechanization equations around the nominal trajectory gives
the continuous-time error dynamics:

```
δṗ = δv
δv̇ = −R·[f]ₓ·δθ − R·δba − R·n_a
δθ̇ = −[ω]ₓ·δθ − δbg − n_g
δḃg = n_bg          (random walk)
δḃa = n_ba          (random walk)
```

where `[x]ₓ` is the skew-symmetric cross-product matrix
(`[x]ₓ·y = x × y`), and `n_a, n_g, n_bg, n_ba` are the noise terms whose
spectral densities are `InsEkfConfig::{accel,gyro}_noise_density` and
`{accel,gyro}_bias_instability`.

This is discretized to first order as `F_d = I + F_c·dt` (see
`InsEkf::predict()` for the exact block layout), and the covariance
propagates as the standard `P ← F_d·P·F_dᵀ + Q_d`, with `Q_d`'s diagonal
built from `noise_density² · dt` per block (the standard relation between a
noise power spectral density and the variance it injects into an integrated
state over one time step).

## Measurement update (GPS)

GPS position and velocity are fused as **six independent scalar updates**
(one per axis, position then velocity) rather than a single 6-dimensional
block update. Because each axis's measurement noise is modeled as
independent, this is mathematically exact — not an approximation — and it
means the filter never needs to invert anything larger than a 1×1 "matrix"
(a scalar division). For a measurement of state index `i`:

```
S = P[i][i] + R                 (innovation covariance, scalar)
K = P[:,i] / S                  (Kalman gain, 15×1)
δx += K · (z − x_nominal[i])    (accumulate correction)
P  -= K · P[i,:]                (covariance update)
```

All six scalar updates from one GPS fix accumulate into `δx` using the
*same* nominal state (correct, since the nominal state doesn't change until
injection) but *sequentially updated* `P` (also correct — this is the
standard sequential/scalar Kalman filter, valid whenever the measurement
noise covariance is diagonal). After all axes are processed, the
accumulated `δx` is injected into the nominal state once
(`InsEkf::injectErrorState()`) and reset to zero.

Note that GPS never observes attitude directly (`H` has no columns in the
`δθ` block). Attitude is corrected only through `P`'s cross-covariance
between `δθ` and `δv`, which the `−R·[f]ₓ·δθ` term in `F` builds up over
time whenever there's meaningful specific force (i.e. during acceleration
or turning). This is why attitude converges more slowly and noisily than
position/velocity, and why it converges *faster* during the simulated
flight's acceleration and turn phases than during straight, constant-speed
cruise. Roll and pitch have a second, independent correction path — gravity
couples directly into the accelerometer, so tilt errors show up in the
predicted specific force itself. **Yaw has no such shortcut through GPS
alone**, which is what the next section fixes.

## Measurement update (magnetometer)

`InsEkf::fuseMag()` gives yaw the direct reference GPS can't: it turns a
magnetometer reading into a heading measurement and fuses it as one more
scalar update on `δθ`'s third component (`kTheta0 + 2`), reusing the exact
same `fuseScalar()` machinery as the GPS updates above.

The derivation, in the filter's own conventions: since
`R_nb = Rz(yaw) · Ry(pitch) · Rx(roll)` (the ZYX composition
`Quaternion::fromEulerRad` builds), let `R_rp = Ry(pitch) · Rx(roll)` — the
"yaw-zeroed" rotation from body into a frame that's level but not yet
pointed at true north. Tilt-compensating the raw reading with the filter's
*current* roll/pitch estimate:

```
field_level = R_rp · field_body
```

Since `R_nb · field_body = field_ned` (the true NED field) whenever the
attitude used is exact, and `R_nb = Rz(yaw) · R_rp`:

```
field_level = Rz(yaw)⁻¹ · field_ned = Rz(−yaw) · field_ned
```

`field_ned`'s horizontal direction points at magnetic north, i.e. at angle
`declination` from true north (same angle convention as yaw itself — see
`InsEkfConfig::mag_declination_rad`). Rotating a vector at angle
`declination` by `Rz(−yaw)` puts it at angle `declination − yaw`, so:

```
yaw_measured = declination − atan2(field_level.y, field_level.x)
innovation   = wrap_to_pi(yaw_measured − yaw_nominal)
fuseScalar(kTheta0 + 2, innovation, mag_yaw_noise_rad²)
```

Only the field's *direction* enters this (the `atan2` cancels any overall
scale), which is why `InsEkf` never needs a calibrated field magnitude —
though hard-iron/soft-iron offsets still bias the *direction* and must be
calibrated out upstream, same as for any compass consumer. Tilt compensation
uses the filter's own roll/pitch estimate, not ground truth, so residual
roll/pitch error leaks into the derived heading — same real-world behavior
as any tilt-compensated compass, and the reason the validation numbers below
use the filter's actual (imperfect) roll/pitch, not the true attitude.

## Measurement gating

Every scalar update — GPS position/velocity and magnetometer heading alike
— passes through an innovation gate in `fuseScalar()` before it's allowed
to correct anything:

```
accept  iff  innovation² <= innovation_gate_sigma² · (P[i][i] + R)
```

i.e. the measurement has to be plausible *given how uncertain the filter
currently is*, not just "close enough" by some fixed distance. This is the
filter-level check that a measurement claiming to be healthy actually is —
independent of `GpsSample::position_valid`/`velocity_valid`, which is the
*external* check (set those from the receiver's own fix-quality report,
e.g. `AP_GPS::status()`). Both matter: a receiver can report a fix as good
when it isn't (spoofing), and a plausible-looking value can still disagree
badly with where the filter knows it is.

**Rejection can't be a one-way trap.** If a channel is rejected, the
filter's own dead-reckoned estimate for it keeps drifting — so the
*longer* a channel goes unaccepted, the *more* likely it is that the
filter's own estimate, not the measurement, has become the wrong one
(exactly the post-GPS-jamming reacquisition case). Each of the 15 possible
scalar channels tracks its own independent "last accepted" time; past
`gate_reset_timeout_s` of nothing being accepted, the next reading for
*that channel* is forced through regardless of the gate.

This has to be tracked **per channel, not per GPS fix**: an early version
of this gate used one shared clock for the whole `fuseGps()` call, and hit
exactly the failure mode that invites — one axis got stuck rejecting every
fix while the *other* axes kept passing normally and kept refreshing the
shared clock, so the stuck axis's own timeout never fired and it stayed
~65 m off for the rest of the flight. The fix, and what's implemented now,
is `last_accepted_time_[state_idx]` per scalar channel.

**A forced accept re-inflates that channel's covariance first.** Without
this, a channel that's been legitimately shrinking `P` for a while (normal
behavior — repeated accepted updates make the filter more confident) would
still only partially trust a forced correction (Kalman gain `K = P/(P+R)`
is small when `P` is small), converge only part of the way, and then need
*another* full `gate_reset_timeout_s` wait before the next partial step —
recovery from a long rejection streak could take many multiples of the
timeout instead of one. Re-inflating `P[i][i]` to at least
`measurement_noise_var * 100` right before a forced accept makes `K`
close to 1, so the resync actually happens in one step, same as a fresh
GPS fix would if the filter had just been re-initialized.

**The honest limit of this mechanism:** it catches a *sudden* implausible
jump — a glitch, or a spoofing attempt that doesn't bother being subtle.
It does not catch a *patient* spoof: a fake position that drifts slowly
enough that every individual innovation stays within a few sigma of `P`
never trips the gate at all, by construction — that's indistinguishable
from genuine sensor noise using this measurement alone. See
`docs/ardupilot_integration.md` for what a fuller defense would need.

## Validation

`examples/FlightScenario.h` generates a **kinematically self-consistent**
synthetic flight (not just an arbitrary curve with invented sensor values):
given an analytic bank-angle and speed profile, the true body rates and
specific force are *derived* from the coordinated-turn equations, so the
"truth" a real rigid body flying this path would actually produce. See the
comments there for the phase-by-phase derivation (accelerate → coordinated
turn with smooth bank-in/out → decelerate).

Sanity check: with sensor noise and bias driven to ~0 and GPS run tight and
fast, roll/pitch/yaw errors converge to well under 1° — confirming the
filter equations themselves are correct (no sign errors, no missing
coupling terms). The numbers below are with **realistic MEMS-grade sensor
noise**, default 5 Hz/1.5 m/0.2 m/s GPS with a 10 s dropout mid-turn, and a
noisy magnetometer (5° 1-sigma) at a nonzero (8°) declination, across 10
random seeds:

| | min | max | typical |
|---|---|---|---|
| Position RMSE | 0.7 m | 3.9 m | ~1.5 m |
| Velocity RMSE | 0.2 m/s | 1.2 m/s | ~0.5 m/s |
| Roll/pitch RMSE | 1.2° | 2.5° | ~1.8° |
| Yaw RMSE | 1.2° | 2.5° | ~1.8° |
| Gyro bias error (final) | — | 0.004 rad/s | well-estimated |
| Accel bias error (final) | — | 0.38 m/s² | forward-axis bias is weakly observable in mostly-level flight (expected — see below) |

**Before magnetometer fusion**, the same flight (GPS + IMU only) gave yaw
RMSE of 1.4°–6.5° with no independent heading correction — the fix above
roughly halves the typical case and, more importantly, removes the
long slow-convergence tail: yaw no longer depends on the vehicle having
maneuvered enough for GPS to indirectly reveal it. Position/velocity RMSE
moved into a similar-but-not-identical range across the same seeds; that's
sampling variation from a different random-draw sequence (the magnetometer
samples consume the same RNG stream), not a regression — the underlying GPS
fusion code is unchanged.

**With innovation gating enabled** (the default), the same 10-seed sweep
is statistically indistinguishable from the table above (e.g. position RMSE
0.7–3.9 m either way) — confirming the gate's covariance re-inflation on
forced-accept does what it's supposed to: normal GPS dropout recovery is
just as fast as with no gating at all, while `tests/test_ins_ekf.cpp`'s
`testGpsInnovationGating` separately confirms a sudden implausible fix is
actually rejected. Gating is a real cost only against a sustained,
gradually-diverging measurement — see the section above.

The accelerometer bias along the body's forward axis is only weakly
separable from a small, constant pitch error during mostly-level,
non-accelerating flight (both look like the same small horizontal specific
force) — this is a known, physically real observability limitation of
accelerometer-only bias estimation, not a filter bug; it's part of why
production INS implementations schedule maneuvers (or fuse other sensors)
to fully identify all biases.

## Possible next steps

- **Detecting a patient spoof.** Innovation gating (see above) catches a
  sudden jump; it can't catch a fake position that drifts slowly enough to
  never trip the gate. That needs a different kind of signal entirely —
  e.g. cross-checking GPS against IMU-only dead reckoning over a longer
  window, or consuming a receiver's own anti-spoofing/RAIM status (the
  Here4's u-blox F9P exposes jamming/spoofing indicator flags over UBX,
  separate from the DroneCAN fix message this filter consumes) — not a
  scalar-update pattern this architecture already has.
- Barometer fusion for a more robust vertical channel.
- Midpoint/RK4 attitude & velocity integration instead of first-order.
- Van Loan or closed-form discretization of `F`/`Q` instead of `I + F·dt`.
- Joseph-form covariance update for extra numerical robustness in the
  scalar updates.
