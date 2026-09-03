"""GPS-aided strapdown Inertial Navigation System for a UAV -- Python port.

Same 15-state error-state EKF as ``src/InsEkf.cpp`` / ``include/ins/InsEkf.h``
in the C++ implementation; see ``docs/architecture.md`` at the repo root for
the full derivation (sign conventions, F/Q block layout, the sequential
scalar GPS update). This file mirrors that implementation line-for-line
where the languages allow, so the two can be cross-checked directly.

This module has no MAVLink/hardware dependencies -- it is pure numpy, usable
standalone (as in ``examples/sim_flight.py``) or driven by real IMU/GPS data
from ``companion/mavlink_ins.py``.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .math3 import Quaternion, skew_symmetric

NUM_STATES = 15
P0, V0, THETA0, BG0, BA0 = 0, 3, 6, 9, 12


@dataclass
class ImuSample:
    timestamp_s: float
    gyro_rad_s: np.ndarray   # body frame, rad/s
    accel_m_s2: np.ndarray   # body frame, specific force, m/s^2


@dataclass
class GpsSample:
    timestamp_s: float
    position_ned_m: np.ndarray
    velocity_ned_m_s: np.ndarray
    position_valid: bool = True
    velocity_valid: bool = True


@dataclass
class InsEkfConfig:
    gravity_m_s2: float = 9.80665

    # Continuous-time noise spectral densities (typical low-cost MEMS IMU).
    gyro_noise_density: float = 0.01          # rad/s / sqrt(Hz)
    accel_noise_density: float = 0.05         # m/s^2 / sqrt(Hz)
    gyro_bias_instability: float = 2.0e-4     # rad/s / sqrt(Hz)  (bias random walk)
    accel_bias_instability: float = 2.0e-3    # m/s^2 / sqrt(Hz) (bias random walk)

    # GPS measurement noise (1-sigma).
    gps_pos_noise_m: float = 1.5
    gps_vel_noise_m_s: float = 0.2

    # Initial covariance (1-sigma) used by init().
    init_pos_std_m: float = 5.0
    init_vel_std_m_s: float = 1.0
    init_att_std_rad: float = 5.0 * np.pi / 180.0
    init_yaw_std_rad: float = 15.0 * np.pi / 180.0
    init_gyro_bias_std: float = 0.05
    init_accel_bias_std: float = 0.3


@dataclass
class InsState:
    timestamp_s: float
    position_ned_m: np.ndarray
    velocity_ned_m_s: np.ndarray
    attitude: Quaternion
    gyro_bias_rad_s: np.ndarray
    accel_bias_m_s2: np.ndarray

    def euler_deg(self) -> tuple[float, float, float]:
        r, p, y = self.attitude.to_euler_rad()
        return r * 180.0 / np.pi, p * 180.0 / np.pi, y * 180.0 / np.pi


class InsEkf:
    def __init__(self, config: InsEkfConfig | None = None):
        self.config = config or InsEkfConfig()
        self.initialized = False
        self._last_time_s = 0.0

        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._q = Quaternion()
        self._bg = np.zeros(3)
        self._ba = np.zeros(3)

        self._P = np.zeros((NUM_STATES, NUM_STATES))
        self._delta_x = np.zeros(NUM_STATES)

    # ---------------------------------------------------------------- init
    def init(self, first_sample: ImuSample, initial_position_ned_m: np.ndarray | None = None) -> None:
        """Initialize from a single stationary, roughly-level IMU sample:
        roll/pitch come from the measured gravity direction (minimal
        rotation aligning it to NED [0,0,-1]); yaw is unobservable from one
        accelerometer sample and is set to 0 with large uncertainty."""
        cfg = self.config
        b = first_sample.accel_m_s2 / (np.linalg.norm(first_sample.accel_m_s2) + 1e-15)
        n = np.array([0.0, 0.0, -1.0])
        axis = np.cross(b, n)
        s = float(np.linalg.norm(axis))
        c = float(np.dot(b, n))
        if s < 1e-9:
            q_align = Quaternion(1, 0, 0, 0) if c > 0.0 else Quaternion.from_rotation_vector(np.array([np.pi, 0, 0]))
        else:
            q_align = Quaternion.from_rotation_vector(axis * (np.arctan2(s, c) / s))
        roll, pitch, _yaw_unused = q_align.to_euler_rad()
        self._q = Quaternion.from_euler_rad(roll, pitch, 0.0)

        self._p = np.array(initial_position_ned_m, dtype=float) if initial_position_ned_m is not None else np.zeros(3)
        self._v = np.zeros(3)
        self._bg = np.zeros(3)
        self._ba = np.zeros(3)
        self._last_time_s = first_sample.timestamp_s

        self._P = np.zeros((NUM_STATES, NUM_STATES))
        for i in range(3):
            self._P[P0 + i, P0 + i] = cfg.init_pos_std_m ** 2
            self._P[V0 + i, V0 + i] = cfg.init_vel_std_m_s ** 2
            self._P[BG0 + i, BG0 + i] = cfg.init_gyro_bias_std ** 2
            self._P[BA0 + i, BA0 + i] = cfg.init_accel_bias_std ** 2
        self._P[THETA0 + 0, THETA0 + 0] = cfg.init_att_std_rad ** 2
        self._P[THETA0 + 1, THETA0 + 1] = cfg.init_att_std_rad ** 2
        self._P[THETA0 + 2, THETA0 + 2] = cfg.init_yaw_std_rad ** 2

        self._delta_x = np.zeros(NUM_STATES)
        self.initialized = True

    # ------------------------------------------------------------- predict
    def predict(self, sample: ImuSample) -> None:
        dt = sample.timestamp_s - self._last_time_s
        if not self.initialized or dt <= 0.0:
            self._last_time_s = sample.timestamp_s
            return
        self._last_time_s = sample.timestamp_s
        cfg = self.config

        gyro = sample.gyro_rad_s - self._bg
        accel = sample.accel_m_s2 - self._ba

        # --- nominal state propagation (strapdown mechanization) ---
        dq = Quaternion.from_rotation_vector(gyro * dt)
        q_new = (self._q * dq).normalized()
        R_new = q_new.to_matrix()  # body -> NED, post-update attitude

        # a_true = R*f_body + g_ned ; g_ned = [0,0,+g] (down is +Z in NED).
        g_ned = np.array([0.0, 0.0, cfg.gravity_m_s2])
        accel_ned = R_new @ accel + g_ned

        v_old = self._v
        self._v = self._v + accel_ned * dt
        self._p = self._p + v_old * dt + accel_ned * (0.5 * dt * dt)
        self._q = q_new

        # --- error-state covariance propagation ---
        Fd = np.eye(NUM_STATES)
        I3 = np.eye(3)
        Fd[P0:P0 + 3, V0:V0 + 3] = I3 * dt
        Fd[V0:V0 + 3, THETA0:THETA0 + 3] = -(R_new @ skew_symmetric(accel)) * dt
        Fd[V0:V0 + 3, BA0:BA0 + 3] = -R_new * dt
        Fd[THETA0:THETA0 + 3, THETA0:THETA0 + 3] = I3 - skew_symmetric(gyro) * dt
        Fd[THETA0:THETA0 + 3, BG0:BG0 + 3] = -I3 * dt

        self._P = Fd @ self._P @ Fd.T

        q_v = cfg.accel_noise_density ** 2 * dt
        q_theta = cfg.gyro_noise_density ** 2 * dt
        q_bg = cfg.gyro_bias_instability ** 2 * dt
        q_ba = cfg.accel_bias_instability ** 2 * dt
        for i in range(3):
            self._P[V0 + i, V0 + i] += q_v
            self._P[THETA0 + i, THETA0 + i] += q_theta
            self._P[BG0 + i, BG0 + i] += q_bg
            self._P[BA0 + i, BA0 + i] += q_ba

    # ------------------------------------------------------------- update
    def _fuse_scalar(self, state_idx: int, innovation: float, measurement_noise_var: float) -> None:
        S = self._P[state_idx, state_idx] + measurement_noise_var
        if S < 1e-15:
            return
        K = self._P[:, state_idx] / S
        self._delta_x += K * innovation
        Hrow = self._P[state_idx, :].copy()  # H is a unit row vector
        self._P -= np.outer(K, Hrow)

    def _inject_error_state(self) -> None:
        self._p = self._p + self._delta_x[P0:P0 + 3]
        self._v = self._v + self._delta_x[V0:V0 + 3]
        dtheta = self._delta_x[THETA0:THETA0 + 3]
        self._q = (self._q * Quaternion.from_rotation_vector(dtheta)).normalized()
        self._bg = self._bg + self._delta_x[BG0:BG0 + 3]
        self._ba = self._ba + self._delta_x[BA0:BA0 + 3]
        self._delta_x = np.zeros(NUM_STATES)

    def fuse_gps(self, gps: GpsSample) -> None:
        """GPS position/velocity update via six independent scalar fusions
        (one per axis) rather than a block matrix inversion -- exact when
        measurement noise is diagonal, and needs no matrix inverse anywhere
        in the filter. See docs/architecture.md."""
        if not self.initialized:
            return
        self._delta_x = np.zeros(NUM_STATES)
        cfg = self.config

        if gps.position_valid:
            for axis in range(3):
                innovation = gps.position_ned_m[axis] - self._p[axis]
                self._fuse_scalar(P0 + axis, innovation, cfg.gps_pos_noise_m ** 2)
        if gps.velocity_valid:
            for axis in range(3):
                innovation = gps.velocity_ned_m_s[axis] - self._v[axis]
                self._fuse_scalar(V0 + axis, innovation, cfg.gps_vel_noise_m_s ** 2)

        self._inject_error_state()

    # -------------------------------------------------------------- state
    def state(self) -> InsState:
        return InsState(
            timestamp_s=self._last_time_s,
            position_ned_m=self._p.copy(),
            velocity_ned_m_s=self._v.copy(),
            attitude=self._q,
            gyro_bias_rad_s=self._bg.copy(),
            accel_bias_m_s2=self._ba.copy(),
        )

    @property
    def covariance(self) -> np.ndarray:
        return self._P
