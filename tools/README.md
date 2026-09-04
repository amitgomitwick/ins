# tools/

One-off utilities that don't belong in the `InsEkf` library itself.

## `parse_dataflash_log.py`

Extracts IMU/GPS/magnetometer/EKF3 data from an ArduPilot dataflash
(`.bin`) log into plain CSVs that `examples/replay_log.cpp` can read —
so you can run **real flight-controller sensor data** through `InsEkf`
after a test drive/flight, not just the synthetic simulation.

This is **not** a reintroduction of a companion-computer runtime (that was
removed from this repo on request). It's a narrow, one-time conversion
step: ArduPilot's binary log format is a solved problem in `pymavlink`
(the `DFReader` it uses under the hood), not worth re-solving in C++ for
an offline replay tool. `InsEkf` stays pure C++; this script only ever
produces CSV files, never runs the filter itself.

```sh
pip install pymavlink
python3 tools/parse_dataflash_log.py path/to/flight.bin --out-dir replay_data/
```

Writes `imu.csv`, `gps.csv`, `mag.csv`, `ekf3.csv` (EKF3's own estimate,
for comparison only) into `--out-dir`, and prints row counts plus
`COMPASS_DEC` if it found one logged — pass that straight to
`replay_log --declination-deg`.

**Honesty note:** the field names/units used here (`GyrX/AccX` in
rad/s·m/s², `Spd`+`GCrs` needing trig for NED velocity, `MagX` in Gauss,
`XKF1`'s per-core fields) were checked against ArduPilot's actual
`LogStructure.h` source on GitHub, not assumed from memory — but this
script has not yet been run against a real `.bin` log in this session. If
message counts come back at 0, or field access throws, check your log's
actual message types first (`mavlogdump.py --types IMU,GPS,MAG,XKF1
flight.bin | head`, from pymavlink's own bundled tools) before assuming
the script is wrong — dataflash message sets vary by `LOG_BITMASK` and
firmware version.

See `examples/replay_log.cpp` for what happens to these CSVs next.
