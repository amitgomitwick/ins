# Python companion-computer deployment

The [`python/`](../python/) directory is a companion-computer port of this
repo's INS: it runs as a normal Python process on a Raspberry Pi/Jetson-
class Linux board wired to the flight controller (USB/UART serial, or a
network link for SITL), consuming real telemetry over MAVLink. It is a
**shadow estimator** by default (Level 1 in
[`ardupilot_integration.md`](ardupilot_integration.md)) — it reads, it
never commands the vehicle, unless you explicitly opt into
`--publish-vision`.

## Why Python can't just replace the C++ integration

A flight controller's estimator runs in a hard-real-time loop (ArduPilot's
EKF3 runs on every scheduler tick, sub-millisecond jitter budget). CPython
can't meet that — no real-time guarantee, GC pauses, and it isn't running
on the flight controller's MCU in the first place. "Embedded Python for
real flight hardware" means a companion computer talking over MAVLink, not
code inside the autopilot binary. That's the entire reason this is a
separate implementation from `include/ins/`, not a rewrite of it.

## Message choice and units (verified against ArduPilot's MAVLink XML)

These were checked against `pymavlink`'s bundled `common.xml`
(`pymavlink/dialects/v20/common.xml`) field-by-field, not assumed from
memory — MAVLink field units are easy to get subtly wrong:

| Message | Field | Unit | Used for |
|---|---|---|---|
| `HIGHRES_IMU` | `xacc/yacc/zacc` | **m/s²** (float) | accelerometer — already SI, already "specific force" |
| `HIGHRES_IMU` | `xgyro/ygyro/zgyro` | **rad/s** (float) | gyro — already SI |
| `SCALED_IMU2` (fallback) | `xacc/yacc/zacc` | **mG** (int16) | ×0.001×9.80665 → m/s² |
| `SCALED_IMU2` (fallback) | `xgyro/ygyro/zgyro` | **mrad/s** (int16) | ×0.001 → rad/s |
| `GLOBAL_POSITION_INT` | `lat/lon` | degE7 | → local NED via flat-Earth projection |
| `GLOBAL_POSITION_INT` | `alt` | mm (MSL) | → NED down |
| `GLOBAL_POSITION_INT` | `vx/vy/vz` | **cm/s, NED** (int16) | already north/east/down (field docs: "positive north" / "positive east" / "positive down") — no axis remapping needed |

**`RAW_IMU` is deliberately not used.** Its `xacc`/`xgyro` fields carry no
declared units in the MAVLink spec (board/sensor-specific raw counts) —
using it would mean guessing a scale factor per board, which is exactly
the kind of silent-wrong-answer bug this project has tried to avoid
throughout. `HIGHRES_IMU` is what ArduPilot streams as calibrated SI
values, and is what `mavlink_ins.py` uses by default.

## Running it

### Against ArduPilot SITL (no hardware needed)

```sh
# in an ArduPilot checkout, once (https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html):
Tools/autotest/sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550

# in this repo, in another terminal:
cd python && source .venv/bin/activate
python companion/mavlink_ins.py --connection udp:127.0.0.1:14550
```

`--out` adds an extra MAVLink output endpoint alongside SITL's normal
ground-station link — this is a real, documented `sim_vehicle.py` flag
(`Tools/autotest/sim_vehicle.py --out`), confirmed against the current
ArduPilot source. **This combination has not been run in this session** —
building SITL (submodules, `install-prereqs`, a `waf` build) was out of
scope for what could be validated here. What *has* been validated,
end-to-end, is the exact same code path against synthetic MAVLink traffic:
`tests/test_mavlink_integration.py` streams the validated synthetic flight
as real `HIGHRES_IMU`/`GLOBAL_POSITION_INT` packets over a real UDP socket
and decodes them with `mavlink_ins.py`'s own functions. That proves the
message choice, units, and NED conversion are right; it does not prove
SITL's actual output stream matches byte-for-byte (it should — ArduPilot
is the reference implementation these units were checked against — but
"should" isn't "verified," so treat the SITL run above as the next real
checkpoint, not a formality.

### Against real hardware

```sh
python companion/mavlink_ins.py --connection /dev/serial0 --baud 921600
```

Whatever serial port and baud your companion computer's link to the flight
controller uses (a Pixhawk `TELEM2` port at 921600 or 57600 is typical).
Confirm `SERIALx_PROTOCOL` is set to MAVLink on that port in your
ArduPilot parameters first.

## `--publish-vision`: making the estimate actually count

By default `mavlink_ins.py` only prints/logs — flip `--publish-vision` and
it also sends `VISION_POSITION_ESTIMATE` back to the autopilot. **That
message is ignored by ArduPilot unless you explicitly configure an
EK3 source set to use it** — it doesn't silently take over. To actually
feed it into the flight controller's own EKF3 (only do this deliberately,
on the bench, with props off, and only after validating the estimate looks
sane in shadow mode first):

```
EK3_SRC1_POSXY = 6   # ExternalNav
EK3_SRC1_VELXY = 6   # ExternalNav
EK3_SRC1_POSZ  = 6   # ExternalNav (or leave as Baro if you'd rather keep barometric altitude)
```

This is real-vehicle configuration, not something this repo sets for you —
get the shadow-mode comparison right first.

## What this does and doesn't prove

- **Proven**: the filter math (`tests/test_ekf.py`, same synthetic-flight
  validation as the C++ version — see `docs/architecture.md`); the MAVLink
  message decode/unit-conversion/NED-projection pipeline, over a real
  socket with real pymavlink encode/decode
  (`tests/test_mavlink_integration.py`).
- **Not proven**: behavior against real ArduPilot SITL or real flight
  hardware — neither was run in this session. Run the SITL command above
  as the next step before trusting this near a real vehicle, and treat
  `--publish-vision` as a separate, deliberate decision after that.
