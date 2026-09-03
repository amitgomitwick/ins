# INS (Python) — companion-computer GPS-aided EKF for a UAV

A Python port of this repo's [C++ INS](../README.md), built for the way
Python actually runs on real flight hardware: **on a companion computer**
(Raspberry Pi / Jetson-class Linux board) talking to the flight controller
over MAVLink — not inside the flight controller itself. Real flight
controllers (Pixhawk/ArduPilot) run hard-real-time C/C++; Python can't meet
that deadline, so it isn't part of the flight-critical loop here.

```
ins_ekf/            The filter itself: math3.py, ekf.py (InsEkf) -- pure
                     numpy, no MAVLink/hardware dependency, same math as
                     the C++ version (see ../docs/architecture.md)
examples/            flight_scenario.py (synthetic flight generator) and
                     sim_flight.py (CLI, writes a CSV) -- offline validation
companion/           mavlink_ins.py: the real companion-computer app, reads
                     live MAVLink telemetry and runs the EKF
                     mock_mavlink_flight.py: replays the synthetic flight as
                     real MAVLink UDP messages, for testing without hardware
tests/               test_ekf.py (filter math, mirrors the C++ tests) and
                     test_mavlink_integration.py (real MAVLink wire format,
                     end-to-end)
```

## Setup

```sh
cd python
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Validate the filter (no hardware needed)

```sh
python tests/test_ekf.py                 # filter math, ~10s
python tests/test_mavlink_integration.py  # real MAVLink wire format, ~15s
python examples/sim_flight.py flight_log.csv   # same synthetic flight as the C++ version
```

`test_mavlink_integration.py` is the one worth calling out: it runs
`mock_mavlink_flight.py` (streams the validated synthetic flight as real
`HIGHRES_IMU`/`GLOBAL_POSITION_INT` MAVLink packets over a real UDP socket)
against `mavlink_ins.py`'s actual decode functions, so it's checking the
message choice, field units, and lat/lon→NED conversion — not just the
filter math test_ekf.py already covers. See
[`../docs/python_companion_computer.md`](../docs/python_companion_computer.md)
for what it does and doesn't prove about real-hardware readiness.

## Run it against real hardware or SITL

```sh
python companion/mavlink_ins.py --connection udp:127.0.0.1:14550     # SITL / companion computer's usual GCS port
python companion/mavlink_ins.py --connection /dev/serial0 --baud 921600  # direct serial to the flight controller
```

By default this **only reads** telemetry and prints/logs its own estimate —
it never commands the vehicle. See
[`../docs/python_companion_computer.md`](../docs/python_companion_computer.md)
before using `--publish-vision`, which changes that.

## Relationship to the C++ implementation

Same architecture, same equations (`InsEkf` here mirrors
`include/ins/InsEkf.h`/`src/InsEkf.cpp` line-for-line where the languages
allow), same synthetic-flight validation, same honest limitation: no
magnetometer, so yaw is only weakly observed. The difference is entirely in
deployment — this version is meant to run as a companion-computer process
reading real telemetry today; the C++ version is meant to eventually run
inside ArduPilot itself (see `../docs/ardupilot_integration.md`), which
Python architecturally can't do for the flight-critical loop.
