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
or turning). This is why attitude — and yaw especially — converges more
slowly and noisily than position/velocity, and why it converges *faster*
during the simulated flight's acceleration and turn phases than during
straight, constant-speed cruise. See the README for why yaw specifically is
weak without a magnetometer.

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
noise**, default 5 Hz/1.5 m/0.2 m/s GPS, and a 10 s GPS dropout mid-turn,
across 10 random seeds:

| | min | max | typical |
|---|---|---|---|
| Position RMSE | 0.7 m | 2.0 m | ~1.1 m |
| Velocity RMSE | 0.2 m/s | 0.6 m/s | ~0.35 m/s |
| Roll/pitch RMSE | 1.4° | 6.5° | ~3° |
| Gyro bias error (final) | — | 0.009 rad/s | well-estimated |
| Accel bias error (final) | — | 0.19 m/s² | forward-axis bias is weakly observable in mostly-level flight (expected — see below) |

The accelerometer bias along the body's forward axis is only weakly
separable from a small, constant pitch error during mostly-level,
non-accelerating flight (both look like the same small horizontal specific
force) — this is a known, physically real observability limitation of
accelerometer-only bias estimation, not a filter bug; it's part of why
production INS implementations schedule maneuvers (or fuse other sensors)
to fully identify all biases.

## Possible next steps

- Magnetometer (compass heading) fusion — directly observes yaw, the
  filter's weakest state. Same scalar-update pattern as GPS.
- Barometer fusion for a more robust vertical channel.
- Midpoint/RK4 attitude & velocity integration instead of first-order.
- Van Loan or closed-form discretization of `F`/`Q` instead of `I + F·dt`.
- Joseph-form covariance update for extra numerical robustness in the GPS
  scalar updates.
