#!/usr/bin/env python3
"""Dependency-free sanity tests for InsEkf (no test framework, just
asserts -> nonzero exit on failure) -- Python port of tests/test_ins_ekf.cpp.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "examples"))

import numpy as np

from ins_ekf import DEG_TO_RAD, GpsSample, ImuSample, InsEkf, InsEkfConfig, Quaternion
from flight_scenario import ScenarioParams, run_simulation

_failures = 0


def expect_true(cond: bool, what: str) -> None:
    global _failures
    if not cond:
        print(f"FAIL: {what}")
        _failures += 1
    else:
        print(f"PASS: {what}")


def test_stationary_level() -> None:
    cfg = InsEkfConfig()
    ekf = InsEkf(cfg)

    s = ImuSample(0.0, np.zeros(3), np.array([0.0, 0.0, -cfg.gravity_m_s2]))
    ekf.init(s)

    for i in range(1, 1001):
        s = ImuSample(i * 0.005, np.zeros(3), np.array([0.0, 0.0, -cfg.gravity_m_s2]))
        ekf.predict(s)

    st = ekf.state()
    roll, pitch, _yaw = st.euler_deg()

    expect_true(float(np.linalg.norm(st.position_ned_m)) < 0.01, "stationary: position stays near zero")
    expect_true(float(np.linalg.norm(st.velocity_ned_m_s)) < 0.01, "stationary: velocity stays near zero")
    expect_true(abs(roll) < 0.1 and abs(pitch) < 0.1, "stationary: attitude stays level")


def test_leveling_init() -> None:
    cfg = InsEkfConfig()
    ekf = InsEkf(cfg)

    true_roll = 10.0 * DEG_TO_RAD
    true_pitch = -5.0 * DEG_TO_RAD
    q_true = Quaternion.from_euler_rad(true_roll, true_pitch, 30.0 * DEG_TO_RAD)
    g_ned = np.array([0.0, 0.0, cfg.gravity_m_s2])
    f_ned = -g_ned
    f_body = q_true.to_matrix().T @ f_ned

    s = ImuSample(0.0, np.zeros(3), f_body)
    ekf.init(s)

    roll, pitch, yaw = ekf.state().euler_deg()
    expect_true(abs(roll - 10.0) < 0.5, "leveling init: recovers roll")
    expect_true(abs(pitch - (-5.0)) < 0.5, "leveling init: recovers pitch")
    expect_true(abs(yaw) < 0.5, "leveling init: yaw defaults to 0 (unobservable)")


def test_full_flight_converges() -> None:
    scenario_params = ScenarioParams()
    filter_config = InsEkfConfig()

    result = run_simulation(scenario_params, filter_config, seed=1, keep_log=False)

    print(f"  pos_rmse={result.pos_rmse_m:.2f}m vel_rmse={result.vel_rmse_m_s:.2f}m/s att_rmse={result.att_rmse_deg:.2f}deg")

    # Bounds are set from the empirical spread across many seeds (numpy's
    # RNG differs from the C++ port's, so these aren't the same numbers as
    # tests/test_ins_ekf.cpp, just the same style of margin). Roll/pitch
    # RMSE of several degrees, and forward-axis accel bias not fully
    # converging, are both expected here -- see docs/architecture.md.
    expect_true(result.pos_rmse_m < 6.0, "full flight: position RMSE bounded")
    expect_true(result.vel_rmse_m_s < 1.2, "full flight: velocity RMSE bounded")
    expect_true(result.att_rmse_deg < 8.0, "full flight: roll/pitch RMSE bounded")
    expect_true(float(np.linalg.norm(result.final_gyro_bias_error)) < 0.01, "full flight: gyro bias estimated")
    expect_true(float(np.linalg.norm(result.final_accel_bias_error)) < 0.5, "full flight: accel bias estimated")


def main() -> int:
    test_stationary_level()
    test_leveling_init()
    test_full_flight_converges()

    if _failures:
        print(f"\n{_failures} test(s) FAILED")
        return 1
    print("\nAll tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
