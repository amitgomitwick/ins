#!/usr/bin/env python3
"""End-to-end integration test: streams the validated synthetic flight as
*real* MAVLink UDP messages (mock_mavlink_flight.py) and decodes them with
the exact same helpers companion/mavlink_ins.py uses
(imu_sample_from_highres, gps_sample_from_global_position_int, LlaToNed),
feeding InsEkf and checking the estimate against the independently-known
true final state.

This is what tests/test_ekf.py can't check: that the MAVLink message
choice, field units, and lat/lon-to-NED conversion are actually right --
by sending real HIGHRES_IMU/GLOBAL_POSITION_INT packets over a real UDP
socket and decoding them with pymavlink, not by passing Python objects
around in-process.
"""
from __future__ import annotations

import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "examples"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "companion"))

import numpy as np
from pymavlink import mavutil

from ins_ekf import InsEkf, InsEkfConfig
from flight_scenario import FlightScenario, ScenarioParams
from mavlink_ins import LlaToNed, gps_sample_from_global_position_int, imu_sample_from_highres
from mock_mavlink_flight import run_mock_flight

_failures = 0


def expect_true(cond: bool, what: str) -> None:
    global _failures
    print(("PASS: " if cond else "FAIL: ") + what)
    if not cond:
        _failures += 1


def find_free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def main() -> int:
    port = find_free_udp_port()
    sp = ScenarioParams()
    imu_rate_hz = 50.0  # reduced from the 200 Hz default sim only to keep this test fast

    # Bind the receiver BEFORE the sender starts, so no early UDP packets
    # (including the initial heartbeat mavlink_ins.py's wait_heartbeat()
    # needs) are dropped for lack of a listener.
    rx = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")

    sender = threading.Thread(
        target=run_mock_flight,
        kwargs=dict(port=port, scenario_params=sp, seed=123, imu_rate_hz=imu_rate_hz, verbose=False),
        daemon=True,
    )
    sender.start()

    rx.wait_heartbeat(timeout=10)
    expect_true(True, "received heartbeat over real MAVLink UDP")

    ekf = InsEkf(InsEkfConfig())
    projector = LlaToNed()
    n_imu = n_gps = 0
    expected_imu = int(sp.total_duration * imu_rate_hz) + 1
    expected_gps = int(sp.total_duration * sp.gps_rate_hz)

    deadline = time.time() + 60.0
    while time.time() < deadline:
        msg = rx.recv_match(type=["HIGHRES_IMU", "GLOBAL_POSITION_INT"], blocking=True, timeout=1.0)
        if msg is None:
            if not sender.is_alive() and n_imu >= expected_imu - 2:
                break
            continue

        if msg.get_type() == "HIGHRES_IMU":
            sample = imu_sample_from_highres(msg)
            if not ekf.initialized:
                ekf.init(sample)
            else:
                ekf.predict(sample)
            n_imu += 1
        else:
            if ekf.initialized:
                ekf.fuse_gps(gps_sample_from_global_position_int(msg, projector))
                n_gps += 1

        if n_imu >= expected_imu and not sender.is_alive():
            break

    sender.join(timeout=5)

    print(f"received {n_imu}/{expected_imu} IMU and {n_gps}/{expected_gps} GPS messages over real MAVLink UDP")
    expect_true(n_imu >= expected_imu * 0.98, "received (almost) all IMU messages over the wire")
    expect_true(n_gps >= expected_gps * 0.90, "received (almost) all GPS messages over the wire")

    # Compare the final estimate against the independently-computed true
    # final state (same FlightScenario, evaluated directly -- not the
    # noisy stream the EKF saw).
    truth_final = FlightScenario(sp).evaluate(sp.total_duration)
    est = ekf.state()
    pos_err = float(np.linalg.norm(est.position_ned_m - truth_final.position_ned))
    vel_err = float(np.linalg.norm(est.velocity_ned_m_s - truth_final.velocity_ned))
    tr, tp, _ty = truth_final.attitude.to_euler_rad()
    er, ep, _ey = est.attitude.to_euler_rad()
    att_err_deg = float(np.degrees(np.hypot(tr - er, tp - ep)))

    print(f"final: pos_err={pos_err:.2f}m vel_err={vel_err:.2f}m/s roll/pitch_err={att_err_deg:.2f}deg")

    # Looser than test_ekf.py's RMSE-over-time bounds: this is a single
    # final-instant sample (not an average), and the GLOBAL_POSITION_INT
    # int16 cm/s velocity quantization adds a little extra noise on top of
    # what the pure-Python test already covers.
    expect_true(pos_err < 15.0, "final position within bounds of true trajectory")
    expect_true(vel_err < 3.0, "final velocity within bounds of true trajectory")
    expect_true(att_err_deg < 10.0, "final roll/pitch within bounds of true trajectory")

    if _failures:
        print(f"\n{_failures} test(s) FAILED")
        return 1
    print("\nAll tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
