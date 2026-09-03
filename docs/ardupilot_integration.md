# Integrating with ArduPilot

This library is standalone on purpose — it has zero ArduPilot dependencies,
so it can be built, tested, and reviewed on its own (as in this repo).
Wiring it into ArduPilot is a separate step, described here. The class
names, signatures and file paths below were checked against the current
[ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot) `master`
branch (`libraries/AP_Math`, `AP_AHRS`, `AP_InertialSensor`, `AP_GPS`,
`AP_NavEKF3`, `AP_Common/Location.h`), not guessed from memory.

**Recommended path: run it as a shadow estimator first, not a replacement.**
ArduPilot's `AP_AHRS::EKFType` (`libraries/AP_AHRS/AP_AHRS.h`) is a fixed
enum selecting between DCM/EKF2/EKF3/SIM — there's no plugin registry to
drop a new estimator into. Making this filter fly the vehicle means
implementing the ~18-method `AP_AHRS_Backend` interface and wiring a new
`EKFType` value through `AP_AHRS`, which is real core-flight-code surgery
you should only do after the estimator has proven itself on logged data.
The two integration levels below reflect that.

## Level 1 — Shadow estimator (start here)

Run `InsEkf` alongside the stock EKF3, fed the same sensors, logging its
own output for comparison. Zero risk to the actual flight-critical
estimator; this is how you'd validate the filter against real IMU/GPS
noise and real vehicle dynamics before trusting it with anything.

**1. Add the library.** Copy `include/ins/` and `src/InsEkf.cpp` into a new
`libraries/AP_INS_Custom/` directory (matching ArduPilot's per-library
layout), and add it to the build (`wscript`/`SConscript` depending on the
build system in use at your checkout).

**2. Feed it IMU samples**, e.g. from a scheduler task alongside the
existing `EKF3` update:

```cpp
// ArduPilot's AP_InertialSensor exposes exactly these accessors
// (libraries/AP_InertialSensor/AP_InertialSensor.h):
const Vector3f &gyro  = ins.get_gyro();   // rad/s, body frame
const Vector3f &accel = ins.get_accel();  // m/s^2, body frame (specific force)

ins::ImuSample sample;
sample.timestamp_s = AP_HAL::micros64() * 1.0e-6;
sample.gyro_rad_s  = ins::Vector3(gyro.x, gyro.y, gyro.z);
sample.accel_m_s2  = ins::Vector3(accel.x, accel.y, accel.z);

if (!my_ekf.isInitialized()) {
    my_ekf.init(sample);
} else {
    my_ekf.predict(sample);
}
```

ArduPilot's `Vector3f`/`Matrix3f`/`Quaternion` (`libraries/AP_Math/`) are
essentially the same shape as this library's `ins::Vector3`/`Matrix3`/
`Quaternion` (same scalar-first `q1,q2,q3,q4` quaternion layout used here
as `w,x,y,z`) — the conversions above are the only glue needed; there's no
deeper adaptation required.

**3. Feed it GPS fixes**, converting the receiver's lat/lon/alt to the
local NED frame with ArduPilot's own helper rather than writing your own
conversion:

```cpp
// libraries/AP_GPS/AP_GPS.h
const Location &loc = gps.location();
const Vector3f  &vel_ned = gps.velocity();  // already NED, m/s

Vector3f pos_ned_m;
if (loc.get_vector_from_origin_NED_m(pos_ned_m)) {   // false until EKF origin is set
    ins::GpsSample g;
    g.timestamp_s = AP_HAL::micros64() * 1.0e-6;
    g.position_ned_m   = ins::Vector3(pos_ned_m.x, pos_ned_m.y, pos_ned_m.z);
    g.velocity_ned_m_s = ins::Vector3(vel_ned.x, vel_ned.y, vel_ned.z);
    g.velocity_valid   = gps.have_vertical_velocity() || true;  // 3D or 2D fix
    my_ekf.fuseGps(g);
}
```

`get_vector_from_origin_NED_m()` needs an EKF origin to already be set
(the stock EKF sets this on first good GPS fix) — the shadow estimator can
just piggyback on that rather than establishing its own origin.

**4. Log it.** Add a new dataflash log message (see any existing
`AP::logger().WriteStreaming(...)` call in `AP_NavEKF3` for the pattern)
recording `my_ekf.state()`'s position/velocity/attitude/biases alongside
the stock EKF3's output. Compare them across real flights — this is the
validation step this repo's synthetic simulation can't give you.

## Level 2 — Full AP_AHRS backend (only after Level 1 checks out)

To have this filter actually fly the vehicle, it needs to implement
`AP_AHRS_Backend` (`libraries/AP_AHRS/AP_AHRS_Backend.h`) — the same
interface `AP_NavEKF3` implements — and be wired into `AP_AHRS::EKFType`
and `AP_AHRS::backend_for_type()`
(`libraries/AP_AHRS/AP_AHRS.h`/`.cpp`) as a new selectable type. This is a
meaningfully larger, safety-critical change (it touches the class every
other subsystem — control loops, failsafes, logging — gets its state
from), so treat it as its own scoped task once Level 1 has real flight
data behind it, and expect it to need review against ArduPilot's own
contribution guidelines.

## What this library does *not* yet give you

- **No magnetometer fusion** — see `docs/architecture.md`'s note on yaw
  observability. ArduPilot's compass driver output
  (`AP_Compass::get_field()`) would feed a new scalar-update measurement,
  same pattern as the GPS updates in `InsEkf::fuseGps()`.
- **No barometer fusion** for the vertical channel.
- **No sensor health/failure handling** (stuck sensors, GPS glitch
  rejection, innovation gating) — `AP_NavEKF3` has extensive logic here
  that a flight-worthy backend would need equivalents of.
- **No airspeed/rangefinder/optical-flow fusion** — EKF3 supports these;
  this filter only fuses IMU + GPS by design (see the project's original
  scope).

None of these are architectural blockers — each is another scalar-update
measurement source or a guard condition on top of the same 15-state filter
— but they're real work, not a checkbox.
