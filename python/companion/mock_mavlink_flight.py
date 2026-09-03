#!/usr/bin/env python3
"""Replays the same validated synthetic flight from examples/flight_scenario.py
as REAL MAVLink messages over UDP -- standing in for a flight controller (or
ArduPilot SITL) so mavlink_ins.py's MAVLink decode/EKF pipeline can be
exercised end-to-end without real hardware.

Test fixture first (see tests/test_mavlink_integration.py), but also usable
standalone if you want to point mavlink_ins.py (or a real GCS) at a
synthetic flight:

    python companion/mock_mavlink_flight.py --port 14550 &
    python companion/mavlink_ins.py --connection udp:127.0.0.1:14550
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "examples"))

import numpy as np
from pymavlink import mavutil

from flight_scenario import FlightScenario, ScenarioParams

EARTH_RADIUS_M = 6371000.0

# Arbitrary local-flight origin -- only used to produce plausible lat/lon
# numbers to encode/decode through GLOBAL_POSITION_INT; not a real location.
ORIGIN_LAT_DEG = 47.0
ORIGIN_LON_DEG = 8.0
ORIGIN_ALT_M = 500.0


def run_mock_flight(port: int, scenario_params: ScenarioParams | None = None, seed: int = 42,
                     imu_rate_hz: float = 50.0, host: str = "127.0.0.1", verbose: bool = True) -> None:
    """Connects OUT to host:port (udpout) -- the same role a real flight
    controller/SITL plays, sending telemetry to a fixed companion-computer
    address -- and streams one full synthetic flight as HIGHRES_IMU +
    GLOBAL_POSITION_INT messages. Returns once the whole flight has been
    sent (there's no real-time pacing: message timestamp fields carry
    synthetic time, so the receiving EKF doesn't need wall-clock-paced
    delivery -- see mavlink_ins.py's imu_sample_from_* helpers)."""
    sp = scenario_params or ScenarioParams()
    scenario = FlightScenario(sp)
    rng = np.random.default_rng(seed)

    master = mavutil.mavlink_connection(f"udpout:{host}:{port}")
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                               mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)

    imu_dt = 1.0 / imu_rate_hz
    gps_dt = 1.0 / sp.gps_rate_hz
    # Matches InsEkfConfig defaults -- see docs/architecture.md for why
    # sample_std = noise_density * sqrt(rate).
    gyro_sample_std = 0.01 * np.sqrt(imu_rate_hz)
    accel_sample_std = 0.05 * np.sqrt(imu_rate_hz)

    cos_lat0 = np.cos(np.radians(ORIGIN_LAT_DEG))
    total_t = sp.total_duration
    n_steps = int(total_t / imu_dt)
    next_gps_t = gps_dt
    n_imu_sent = n_gps_sent = 0

    for i in range(n_steps + 1):
        t = i * imu_dt
        truth = scenario.evaluate(t)

        gyro = truth.body_rate + sp.true_gyro_bias + rng.normal(0, gyro_sample_std, 3)
        accel = truth.specific_force + sp.true_accel_bias + rng.normal(0, accel_sample_std, 3)
        master.mav.highres_imu_send(
            int(t * 1e6), float(accel[0]), float(accel[1]), float(accel[2]),
            float(gyro[0]), float(gyro[1]), float(gyro[2]),
            0.0, 0.0, 0.0,          # mag -- unused by this filter
            1013.25, 0.0, 0.0, 20.0,  # abs_pressure, diff_pressure, pressure_alt, temperature
            0, 0)                    # fields_updated, id
        n_imu_sent += 1

        if t >= next_gps_t:
            next_gps_t += gps_dt
            pos_noise = rng.normal(0, 1.5, 3)
            vel_noise = rng.normal(0, 0.2, 3)
            n_true, e_true, d_true = truth.position_ned + pos_noise
            v_ned = truth.velocity_ned + vel_noise

            lat = ORIGIN_LAT_DEG + np.degrees(n_true / EARTH_RADIUS_M)
            lon = ORIGIN_LON_DEG + np.degrees(e_true / (EARTH_RADIUS_M * cos_lat0))
            alt = ORIGIN_ALT_M - d_true

            master.mav.global_position_int_send(
                int(t * 1000), int(lat * 1e7), int(lon * 1e7), int(alt * 1000), 0,
                int(np.clip(v_ned[0] * 100, -32768, 32767)),
                int(np.clip(v_ned[1] * 100, -32768, 32767)),
                int(np.clip(v_ned[2] * 100, -32768, 32767)),
                65535)  # hdg unknown
            n_gps_sent += 1

    if verbose:
        print(f"mock flight: sent {n_imu_sent} IMU + {n_gps_sent} GPS messages to udp {host}:{port}")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--imu-rate-hz", type=float, default=50.0)
    args = ap.parse_args()
    run_mock_flight(args.port, seed=args.seed, imu_rate_hz=args.imu_rate_hz, host=args.host)
