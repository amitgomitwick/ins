#!/usr/bin/env python3
"""Companion-computer INS: runs InsEkf against a *real* MAVLink telemetry
stream from a flight controller (real hardware, or SITL) over pymavlink.

This is the "Level 1 shadow estimator" from docs/ardupilot_integration.md,
implemented for a Python companion computer instead of C++ inside
ArduPilot: it never commands the vehicle, it only reads IMU/GPS and prints/
logs its own independent estimate alongside the autopilot's. That's a
deliberate, safety-first default -- see --publish-vision below before
changing it.

IMU source: HIGHRES_IMU (message id 105) gives calibrated accel in m/s^2
and gyro in rad/s directly -- no unit conversion needed, and it's exactly
this filter's "specific force" convention. RAW_IMU is NOT used: MAVLink
leaves its units sensor/board-specific (see the field table in
docs/python_companion_computer.md), so it can't be trusted without a
device-specific scale factor. SCALED_IMU2 (mG, mrad/s) is supported as a
fallback via --imu-source.

GPS source: GLOBAL_POSITION_INT gives lat/lon/alt (WGS84) and vx/vy/vz
already in the NED convention this filter uses (confirmed against
pymavlink's bundled common.xml: "vx: Ground X Speed (Latitude, positive
north)", vy positive east, vz positive down). Lat/lon is converted to a
local NED plane with a flat-Earth (equirectangular) approximation around
the first fix, same approach ArduPilot's own Location code uses for local
offsets over short ranges.

Usage:
    python companion/mavlink_ins.py --connection udp:127.0.0.1:14550
    python companion/mavlink_ins.py --connection /dev/serial0 --baud 921600
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from pymavlink import mavutil

from ins_ekf import GpsSample, ImuSample, InsEkf, InsEkfConfig

EARTH_RADIUS_M = 6371000.0

# MAVLink message ids for MAV_CMD_SET_MESSAGE_INTERVAL.
MSG_ID_HIGHRES_IMU = 105
MSG_ID_SCALED_IMU2 = 116
MSG_ID_GLOBAL_POSITION_INT = 33


class LlaToNed:
    """Flat-Earth (equirectangular) projection around the first fix it
    sees. Adequate for local-area flight (the same approximation
    ArduPilot's own Location offset math uses); not a substitute for a
    real geodesy library over long distances."""

    def __init__(self) -> None:
        self.origin_lat_rad: float | None = None
        self.origin_lon_rad: float = 0.0
        self.origin_alt_m: float = 0.0
        self._cos_lat0 = 1.0

    def set_origin_if_needed(self, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        if self.origin_lat_rad is not None:
            return
        self.origin_lat_rad = np.radians(lat_deg)
        self.origin_lon_rad = np.radians(lon_deg)
        self.origin_alt_m = alt_m
        self._cos_lat0 = np.cos(self.origin_lat_rad)

    def to_ned(self, lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
        lat_rad, lon_rad = np.radians(lat_deg), np.radians(lon_deg)
        north = (lat_rad - self.origin_lat_rad) * EARTH_RADIUS_M
        east = (lon_rad - self.origin_lon_rad) * EARTH_RADIUS_M * self._cos_lat0
        down = -(alt_m - self.origin_alt_m)
        return np.array([north, east, down])


def request_message_interval(master, message_id: int, rate_hz: float) -> None:
    interval_us = 1.0e6 / rate_hz if rate_hz > 0 else -1
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, interval_us, 0, 0, 0, 0, 0,
    )


def imu_sample_from_highres(msg) -> ImuSample:
    # time_usec is microseconds since the *autopilot's* boot, not wall
    # clock -- using it (instead of the companion computer's receive
    # time) keeps dt free of network/serial jitter, which matters a lot
    # for a filter that integrates rate/acceleration over dt.
    return ImuSample(
        timestamp_s=msg.time_usec * 1.0e-6,
        gyro_rad_s=np.array([msg.xgyro, msg.ygyro, msg.zgyro]),
        accel_m_s2=np.array([msg.xacc, msg.yacc, msg.zacc]),
    )


def imu_sample_from_scaled_imu2(msg) -> ImuSample:
    return ImuSample(
        timestamp_s=msg.time_boot_ms * 1.0e-3,
        gyro_rad_s=np.array([msg.xgyro, msg.ygyro, msg.zgyro]) * 1.0e-3,
        accel_m_s2=np.array([msg.xacc, msg.yacc, msg.zacc]) * 1.0e-3 * 9.80665,
    )


def gps_sample_from_global_position_int(msg, projector: "LlaToNed") -> GpsSample:
    lat_deg, lon_deg, alt_m = msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3
    projector.set_origin_if_needed(lat_deg, lon_deg, alt_m)
    pos_ned = projector.to_ned(lat_deg, lon_deg, alt_m)
    vel_ned = np.array([msg.vx, msg.vy, msg.vz]) * 1.0e-2  # cm/s -> m/s
    return GpsSample(timestamp_s=msg.time_boot_ms * 1e-3, position_ned_m=pos_ned, velocity_ned_m_s=vel_ned)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connection", default="udp:127.0.0.1:14550",
                     help="pymavlink connection string: udp:HOST:PORT, tcp:HOST:PORT, or a serial device path")
    ap.add_argument("--baud", type=int, default=921600, help="baud rate for a serial connection")
    ap.add_argument("--imu-source", choices=["highres", "scaled2"], default="highres")
    ap.add_argument("--imu-rate-hz", type=float, default=50.0)
    ap.add_argument("--gps-rate-hz", type=float, default=5.0)
    ap.add_argument("--print-interval-s", type=float, default=1.0, help="how often to print the estimate")
    ap.add_argument("--log", type=str, default=None, help="optional CSV path to log every estimate")
    ap.add_argument(
        "--publish-vision", action="store_true",
        help="ADVANCED / RISKY: also send this filter's estimate back to the autopilot as "
             "VISION_POSITION_ESTIMATE. Only affects the vehicle if EK3_SRC is explicitly "
             "configured to use it (see docs/python_companion_computer.md) -- off by default "
             "so this tool is a passive shadow estimator until you opt in.")
    args = ap.parse_args()

    print(f"Connecting to {args.connection} ...")
    master = mavutil.mavlink_connection(args.connection, baud=args.baud)
    master.wait_heartbeat()
    print(f"Heartbeat from system {master.target_system} component {master.target_component}")

    request_message_interval(master, MSG_ID_GLOBAL_POSITION_INT, args.gps_rate_hz)
    if args.imu_source == "highres":
        request_message_interval(master, MSG_ID_HIGHRES_IMU, args.imu_rate_hz)
    else:
        request_message_interval(master, MSG_ID_SCALED_IMU2, args.imu_rate_hz)

    ekf = InsEkf(InsEkfConfig())
    projector = LlaToNed()

    log_writer = None
    log_file = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t", "n", "e", "d", "vn", "ve", "vd", "roll_deg", "pitch_deg", "yaw_deg",
                              "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
                              "accel_bias_x", "accel_bias_y", "accel_bias_z"])

    last_print = 0.0
    n_imu = n_gps = 0
    imu_msg_name = "HIGHRES_IMU" if args.imu_source == "highres" else "SCALED_IMU2"
    print(f"Streaming {imu_msg_name} @ {args.imu_rate_hz} Hz, GLOBAL_POSITION_INT @ {args.gps_rate_hz} Hz. Ctrl+C to stop.")

    try:
        while True:
            msg = master.recv_match(
                type=[imu_msg_name, "GLOBAL_POSITION_INT"], blocking=True, timeout=5.0)
            if msg is None:
                print("No MAVLink messages received in 5s -- link may be down.")
                continue

            now = time.time()  # wall clock: only for print throttling and the vision timestamp below

            if msg.get_type() == imu_msg_name:
                # Sample timestamps come from the message's own onboard
                # clock field (not `now`/wall time) so predict()'s dt is
                # free of network/serial receive jitter -- see the
                # imu_sample_from_* helpers.
                sample = (imu_sample_from_highres(msg) if args.imu_source == "highres"
                          else imu_sample_from_scaled_imu2(msg))
                if not ekf.initialized:
                    ekf.init(sample)
                    print("EKF initialized from first IMU sample.")
                else:
                    ekf.predict(sample)
                n_imu += 1

            elif msg.get_type() == "GLOBAL_POSITION_INT":
                if ekf.initialized:
                    # fuse_gps() doesn't use the sample's timestamp in its
                    # math (it's an event-driven correction applied to
                    # whatever the current predicted state is), so this
                    # field is informational only -- no clock-domain
                    # assumption needed here.
                    ekf.fuse_gps(gps_sample_from_global_position_int(msg, projector))
                    n_gps += 1

                if args.publish_vision and ekf.initialized:
                    st = ekf.state()
                    roll, pitch, yaw = st.attitude.to_euler_rad()
                    # ArduPilot auto-detects Unix-epoch vs. boot-relative
                    # timestamps by magnitude (same rule as HIGHRES_IMU);
                    # sending wall-clock epoch time here is the common
                    # pattern for external vision/nav companion setups.
                    # Verify against your specific autopilot's logs before
                    # relying on this -- see docs/python_companion_computer.md.
                    master.mav.vision_position_estimate_send(
                        int(now * 1e6), float(st.position_ned_m[0]), float(st.position_ned_m[1]),
                        float(st.position_ned_m[2]), roll, pitch, yaw)

            if ekf.initialized and now - last_print >= args.print_interval_s:
                last_print = now
                st = ekf.state()
                roll, pitch, yaw = st.euler_deg()
                print(f"t={now:.1f}  pos(NED)=[{st.position_ned_m[0]:7.1f} {st.position_ned_m[1]:7.1f} "
                      f"{st.position_ned_m[2]:6.1f}]m  vel=[{st.velocity_ned_m_s[0]:5.1f} "
                      f"{st.velocity_ned_m_s[1]:5.1f} {st.velocity_ned_m_s[2]:5.1f}]m/s  "
                      f"rpy=[{roll:5.1f} {pitch:5.1f} {yaw:6.1f}]deg  "
                      f"(imu={n_imu} gps={n_gps})")
                if log_writer:
                    log_writer.writerow([now, *st.position_ned_m, *st.velocity_ned_m_s, roll, pitch, yaw,
                                          *st.gyro_bias_rad_s, *st.accel_bias_m_s2])
                    log_file.flush()

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if log_file:
            log_file.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
