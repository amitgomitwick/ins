#!/usr/bin/env python3
"""Runs the synthetic UAV flight through InsEkf and writes a CSV of
true-vs-estimated state for inspection/plotting -- Python port of
``examples/sim_flight.cpp``.

Usage: python sim_flight.py [output.csv]
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ins_ekf import InsEkfConfig
from flight_scenario import ScenarioParams, run_simulation


def main() -> int:
    out_path = sys.argv[1] if len(sys.argv) > 1 else "flight_log.csv"

    scenario_params = ScenarioParams()
    filter_config = InsEkfConfig()

    result = run_simulation(scenario_params, filter_config, seed=42, keep_log=True)

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "t", "true_n", "true_e", "true_d", "est_n", "est_e", "est_d",
            "true_vn", "true_ve", "true_vd", "est_vn", "est_ve", "est_vd",
            "true_roll", "true_pitch", "true_yaw", "est_roll", "est_pitch", "est_yaw",
            "gps_fused", "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
            "accel_bias_x", "accel_bias_y", "accel_bias_z",
        ])
        for r in result.log:
            w.writerow([
                r.t, *r.true_p, *r.est_p, *r.true_v, *r.est_v,
                *r.true_euler_deg, *r.est_euler_deg,
                int(r.gps_fused), *r.est_gyro_bias, *r.est_accel_bias,
            ])

    print(f"Wrote {len(result.log)} rows to {out_path}\n")
    print(f"Position RMSE : {result.pos_rmse_m:.3f} m")
    print(f"Velocity RMSE : {result.vel_rmse_m_s:.3f} m/s")
    print(f"Attitude RMSE : {result.att_rmse_deg:.3f} deg")
    print(f"Final gyro bias error  : [{result.final_gyro_bias_error[0]:.4f}, "
          f"{result.final_gyro_bias_error[1]:.4f}, {result.final_gyro_bias_error[2]:.4f}] rad/s")
    print(f"Final accel bias error : [{result.final_accel_bias_error[0]:.4f}, "
          f"{result.final_accel_bias_error[1]:.4f}, {result.final_accel_bias_error[2]:.4f}] m/s^2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
