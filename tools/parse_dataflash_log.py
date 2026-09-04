#!/usr/bin/env python3
"""Extract IMU/GPS/magnetometer samples from an ArduPilot dataflash (.bin)
log into plain CSVs that examples/replay_log.cpp can read.

This is a ONE-TIME LOG-CONVERSION UTILITY, not a companion-computer
runtime -- InsEkf itself stays pure C++ (see tools/README.md). It exists
purely because parsing ArduPilot's binary log format is a solved problem
in pymavlink (DFReader) and not worth re-solving in C++ for a one-off
offline replay tool.

Field names/units below were checked against ArduPilot's actual log
message definitions on GitHub (libraries/AP_InertialSensor/LogStructure.h,
AP_GPS/LogStructure.h, AP_Compass/LogStructure.h, AP_NavEKF3/LogStructure.h),
not assumed from memory:

  IMU:  GyrX/Y/Z already rad/s, AccX/Y/Z already m/s^2 (mult="0", i.e. the
        log's own FMTU scaling is 1x -- and pymavlink's DFReader applies
        that scaling for you, so msg.GyrX etc. come back already in these
        units, no manual conversion needed here).
  GPS:  Spd = ground speed (m/s) magnitude, GCrs = ground course (deg,
        clockwise from true north -- same convention this project's yaw
        uses), VZ = vertical speed (m/s). Lat/Lng/Alt already in
        degrees/degrees/metres (DFReader applies the log's own int32
        degrees*1e7 / cm scaling for you). NED velocity is NOT logged
        directly -- it's reconstructed here via trig from Spd+GCrs+VZ.
        VZ's sign (positive-down, matching this project's NED convention)
        mirrors AP_GPS's internal state.velocity.z, which is what MAVLink's
        GLOBAL_POSITION_INT.vz documents as "positive down" -- carried over
        from that verified finding, not independently re-checked against
        this specific dataflash field. Sanity-check it against a known
        climb/descent in your own log before trusting it.
  MAG:  MagX/Y/Z in Gauss. Only the *direction* matters for InsEkf::fuseMag
        (it's atan2 of the horizontal components), so the unit and any
        residual hard/soft-iron offset not fully calibrated out just adds
        noise to that direction, same caveat as any compass consumer.
  XKF1: EKF3's own primary-core (C==0) attitude/velocity/position estimate,
        pulled in purely as a comparison baseline -- not used by InsEkf.

Usage:
    python3 tools/parse_dataflash_log.py flight.bin --out-dir replay_data/
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("This needs pymavlink: pip install pymavlink", file=sys.stderr)
    sys.exit(1)

MIN_FIX_TYPE_3D = 3  # AP_GPS_FixType: 0=NO_GPS,1=NO_FIX,2=FIX_2D,3=FIX_3D,...


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", help="ArduPilot dataflash .bin log")
    ap.add_argument("--out-dir", default=".", help="directory to write imu.csv/gps.csv/mag.csv/ekf3.csv into")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    mlog = mavutil.mavlink_connection(args.logfile)

    imu_f = open(out_dir / "imu.csv", "w", newline="")
    gps_f = open(out_dir / "gps.csv", "w", newline="")
    mag_f = open(out_dir / "mag.csv", "w", newline="")
    ekf_f = open(out_dir / "ekf3.csv", "w", newline="")
    imu_w = csv.writer(imu_f)
    gps_w = csv.writer(gps_f)
    mag_w = csv.writer(mag_f)
    ekf_w = csv.writer(ekf_f)
    imu_w.writerow(["t", "gx", "gy", "gz", "ax", "ay", "az"])
    gps_w.writerow(["t", "lat_deg", "lon_deg", "alt_m", "vn", "ve", "vd", "status", "nsats"])
    mag_w.writerow(["t", "mx", "my", "mz"])
    ekf_w.writerow(["t", "roll_deg", "pitch_deg", "yaw_deg", "vn", "ve", "vd", "pn", "pe", "pd"])

    counts = {"IMU": 0, "GPS": 0, "GPS_used": 0, "MAG": 0, "XKF1": 0, "PARM": 0}
    compass_dec = None
    first_imu_instance = None
    first_gps_instance = None
    first_mag_instance = None

    while True:
        msg = mlog.recv_match(type=["IMU", "GPS", "MAG", "XKF1", "PARM"], blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        t = msg.TimeUS * 1.0e-6

        if mtype == "PARM":
            if msg.Name == "COMPASS_DEC":
                compass_dec = msg.Value
            counts["PARM"] += 1

        elif mtype == "IMU":
            # Multiple IMUs may be logged (instance field `I`); stick to the
            # first instance seen so predict() gets one consistent stream.
            if first_imu_instance is None:
                first_imu_instance = msg.I
            if msg.I != first_imu_instance:
                continue
            imu_w.writerow([f"{t:.6f}", msg.GyrX, msg.GyrY, msg.GyrZ, msg.AccX, msg.AccY, msg.AccZ])
            counts["IMU"] += 1

        elif mtype == "GPS":
            if first_gps_instance is None:
                first_gps_instance = msg.I
            if msg.I != first_gps_instance:
                continue
            counts["GPS"] += 1
            if msg.Status < MIN_FIX_TYPE_3D:
                continue  # not a usable fix -- the "external health" check
            counts["GPS_used"] += 1
            course_rad = math.radians(msg.GCrs)
            vn = msg.Spd * math.cos(course_rad)
            ve = msg.Spd * math.sin(course_rad)
            vd = msg.VZ
            gps_w.writerow([f"{t:.6f}", msg.Lat, msg.Lng, msg.Alt, f"{vn:.4f}", f"{ve:.4f}", f"{vd:.4f}",
                             msg.Status, msg.NSats])

        elif mtype == "MAG":
            if first_mag_instance is None:
                first_mag_instance = msg.I
            if msg.I != first_mag_instance:
                continue
            mag_w.writerow([f"{t:.6f}", msg.MagX, msg.MagY, msg.MagZ])
            counts["MAG"] += 1

        elif mtype == "XKF1":
            if msg.C != 0:
                continue  # primary EKF3 core only
            ekf_w.writerow([f"{t:.6f}", msg.Roll, msg.Pitch, msg.Yaw, msg.VN, msg.VE, msg.VD,
                             msg.PN, msg.PE, msg.PD])
            counts["XKF1"] += 1

    for f in (imu_f, gps_f, mag_f, ekf_f):
        f.close()

    print(f"Wrote {out_dir}/imu.csv  ({counts['IMU']} rows, IMU instance {first_imu_instance})")
    print(f"Wrote {out_dir}/gps.csv  ({counts['GPS_used']}/{counts['GPS']} fixes were 3D+ and kept, "
          f"GPS instance {first_gps_instance})")
    print(f"Wrote {out_dir}/mag.csv  ({counts['MAG']} rows, MAG instance {first_mag_instance})")
    print(f"Wrote {out_dir}/ekf3.csv ({counts['XKF1']} rows, EKF3 core 0 -- for comparison only)")
    if compass_dec is not None:
        print(f"\nCOMPASS_DEC found in log: {compass_dec:.6f} rad ({math.degrees(compass_dec):.2f} deg)")
        print(f"Pass this to replay_log: --declination-deg {math.degrees(compass_dec):.4f}")
    else:
        print("\nCOMPASS_DEC not found in this log's PARM messages -- pass --declination-deg "
              "yourself (from a WMM/NOAA calculator for your location) or yaw will be offset.")

    if counts["IMU"] == 0 or counts["GPS_used"] == 0:
        print("\nWARNING: little/no usable IMU or GPS data found. Check LOG_BITMASK included "
              "these message types, and that the log actually covers an armed period "
              "(logging is usually armed-only unless LOG_DISARMED=1).", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
