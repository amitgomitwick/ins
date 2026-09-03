"""Synthetic UAV flight used to exercise and validate InsEkf -- Python port
of ``examples/FlightScenario.h``. Same trajectory, same derivation: a
straight accelerate-away leg, a coordinated turn, and a straight
decelerate-in leg, all at constant altitude, built so attitude/body-rate/
specific-force are kinematically self-consistent. See docs/architecture.md
at the repo root for the physics.

Example/test-only code: not part of the ins_ekf package.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from ins_ekf import GpsSample, ImuSample, InsEkf, InsEkfConfig, Quaternion, RAD_TO_DEG


@dataclass
class ScenarioParams:
    accel_phase_s: float = 10.0
    turn_phase_s: float = 90.0
    decel_phase_s: float = 20.0
    roll_ramp_s: float = 3.0
    cruise_speed_m_s: float = 15.0
    turn_radius_m: float = 200.0
    gravity_m_s2: float = 9.80665

    imu_rate_hz: float = 200.0
    gps_rate_hz: float = 5.0
    gps_dropout_start_s: float = 40.0
    gps_dropout_end_s: float = 50.0

    true_gyro_bias: np.ndarray = field(default_factory=lambda: np.array([0.020, -0.015, 0.010]))
    true_accel_bias: np.ndarray = field(default_factory=lambda: np.array([0.15, -0.10, 0.08]))

    @property
    def total_duration(self) -> float:
        return self.accel_phase_s + self.turn_phase_s + self.decel_phase_s


@dataclass
class TrueState:
    t: float
    position_ned: np.ndarray
    velocity_ned: np.ndarray
    attitude: Quaternion
    body_rate: np.ndarray
    specific_force: np.ndarray


def ramp_value(t: float, t0: float, duration: float) -> float:
    u = np.clip((t - t0) / duration, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def ramp_deriv(t: float, t0: float, duration: float) -> float:
    u = (t - t0) / duration
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return (6.0 * u - 6.0 * u * u) / duration


class FlightScenario:
    def __init__(self, params: ScenarioParams):
        self.p = params
        self._build_yaw_table()

    def _build_yaw_table(self) -> None:
        dt = 0.001
        T2 = self.p.turn_phase_s
        n = int(T2 / dt) + 2
        self._yaw_dt = dt
        table = np.zeros(n)
        Tr = self.p.roll_ramp_s
        target_phi = np.arctan2(self.p.cruise_speed_m_s ** 2 / self.p.turn_radius_m, self.p.gravity_m_s2)
        yaw = 0.0
        for i in range(1, n):
            u = i * dt
            roll = target_phi * (ramp_value(u, 0.0, Tr) - ramp_value(u, T2 - Tr, Tr))
            yaw_dot = self.p.gravity_m_s2 * np.tan(roll) / self.p.cruise_speed_m_s
            yaw += yaw_dot * dt
            table[i] = yaw
        self._yaw_table = table

        # position-integration cache (see integrate_position)
        self._pos_cache_t = 0.0
        self._pos_cache = np.zeros(3)

    def _lookup_yaw(self, u: float) -> float:
        if u <= 0.0:
            return float(self._yaw_table[0])
        idx_f = u / self._yaw_dt
        idx = int(idx_f)
        if idx + 1 >= len(self._yaw_table):
            return float(self._yaw_table[-1])
        frac = idx_f - idx
        return float(self._yaw_table[idx] * (1.0 - frac) + self._yaw_table[idx + 1] * frac)

    def _speed_yaw(self, t: float) -> tuple[float, float]:
        T1, T2, v0 = self.p.accel_phase_s, self.p.turn_phase_s, self.p.cruise_speed_m_s
        if t < T1:
            return v0 * ramp_value(t, 0.0, T1), 0.0
        elif t < T1 + T2:
            return v0, self._lookup_yaw(t - T1)
        else:
            u = t - T1 - T2
            return v0 * (1.0 - ramp_value(u, 0.0, self.p.decel_phase_s)), float(self._yaw_table[-1])

    def _integrate_position(self, t_query: float) -> np.ndarray:
        dt = 0.005
        while self._pos_cache_t < t_query - 1e-9:
            speed, yaw = self._speed_yaw(self._pos_cache_t)
            v = np.array([speed * np.cos(yaw), speed * np.sin(yaw), 0.0])
            self._pos_cache = self._pos_cache + v * dt
            self._pos_cache_t += dt
        return self._pos_cache

    def evaluate(self, t: float) -> TrueState:
        T1, T2, v0 = self.p.accel_phase_s, self.p.turn_phase_s, self.p.cruise_speed_m_s

        if t < T1:
            speed = v0 * ramp_value(t, 0.0, T1)
            speed_dot = v0 * ramp_deriv(t, 0.0, T1)
            roll, roll_dot, yaw, yaw_dot = 0.0, 0.0, 0.0, 0.0
        elif t < T1 + T2:
            u = t - T1
            Tr = self.p.roll_ramp_s
            target_phi = np.arctan2(v0 ** 2 / self.p.turn_radius_m, self.p.gravity_m_s2)
            roll = target_phi * (ramp_value(u, 0.0, Tr) - ramp_value(u, T2 - Tr, Tr))
            roll_dot = target_phi * (ramp_deriv(u, 0.0, Tr) - ramp_deriv(u, T2 - Tr, Tr))
            speed, speed_dot = v0, 0.0
            yaw_dot = self.p.gravity_m_s2 * np.tan(roll) / v0
            yaw = self._lookup_yaw(u)
        else:
            u = t - T1 - T2
            T3 = self.p.decel_phase_s
            speed = v0 * (1.0 - ramp_value(u, 0.0, T3))
            speed_dot = -v0 * ramp_deriv(u, 0.0, T3)
            roll, roll_dot, yaw_dot = 0.0, 0.0, 0.0
            yaw = float(self._yaw_table[-1])

        cy, sy = np.cos(yaw), np.sin(yaw)
        velocity_ned = np.array([speed * cy, speed * sy, 0.0])
        a_ned = np.array([speed_dot * cy - speed * yaw_dot * sy, speed_dot * sy + speed * yaw_dot * cy, 0.0])
        attitude = Quaternion.from_euler_rad(roll, 0.0, yaw)
        g_ned = np.array([0.0, 0.0, self.p.gravity_m_s2])
        f_ned = a_ned - g_ned
        specific_force = attitude.to_matrix().T @ f_ned  # NED -> body

        body_rate = np.array([roll_dot, yaw_dot * np.sin(roll), yaw_dot * np.cos(roll)])
        position_ned = self._integrate_position(t)

        return TrueState(t, position_ned, velocity_ned, attitude, body_rate, specific_force)


@dataclass
class LogRow:
    t: float
    true_p: np.ndarray
    est_p: np.ndarray
    true_v: np.ndarray
    est_v: np.ndarray
    true_euler_deg: tuple[float, float, float]
    est_euler_deg: tuple[float, float, float]
    gps_fused: bool
    est_gyro_bias: np.ndarray
    est_accel_bias: np.ndarray


@dataclass
class SimulationResult:
    pos_rmse_m: float = 0.0
    vel_rmse_m_s: float = 0.0
    att_rmse_deg: float = 0.0
    final_gyro_bias_error: np.ndarray = field(default_factory=lambda: np.zeros(3))
    final_accel_bias_error: np.ndarray = field(default_factory=lambda: np.zeros(3))
    log: list = field(default_factory=list)


def run_simulation(scenario_params: ScenarioParams, filter_config: InsEkfConfig, seed: int, keep_log: bool) -> SimulationResult:
    scenario = FlightScenario(scenario_params)
    rng = np.random.default_rng(seed)

    imu_dt = 1.0 / scenario_params.imu_rate_hz
    gps_dt = 1.0 / scenario_params.gps_rate_hz
    gyro_sample_std = filter_config.gyro_noise_density * np.sqrt(scenario_params.imu_rate_hz)
    accel_sample_std = filter_config.accel_noise_density * np.sqrt(scenario_params.imu_rate_hz)

    ekf = InsEkf(filter_config)
    result = SimulationResult()

    next_gps_t = gps_dt
    sum_pos_sq = sum_vel_sq = sum_att_sq = 0.0
    n_samples = 0
    total_t = scenario_params.total_duration
    n_steps = int(total_t / imu_dt)

    for i in range(n_steps + 1):
        t = i * imu_dt
        truth = scenario.evaluate(t)

        imu = ImuSample(
            timestamp_s=t,
            gyro_rad_s=truth.body_rate + scenario_params.true_gyro_bias + rng.normal(0, gyro_sample_std, 3),
            accel_m_s2=truth.specific_force + scenario_params.true_accel_bias + rng.normal(0, accel_sample_std, 3),
        )

        gps_fused = False
        if i == 0:
            ekf.init(imu, np.zeros(3))
        else:
            ekf.predict(imu)
            if t >= next_gps_t:
                next_gps_t += gps_dt
                in_dropout = scenario_params.gps_dropout_start_s <= t <= scenario_params.gps_dropout_end_s
                if not in_dropout:
                    gps = GpsSample(
                        timestamp_s=t,
                        position_ned_m=truth.position_ned + rng.normal(0, filter_config.gps_pos_noise_m, 3),
                        velocity_ned_m_s=truth.velocity_ned + rng.normal(0, filter_config.gps_vel_noise_m_s, 3),
                    )
                    ekf.fuse_gps(gps)
                    gps_fused = True

        est = ekf.state()
        pos_err = float(np.linalg.norm(est.position_ned_m - truth.position_ned))
        vel_err = float(np.linalg.norm(est.velocity_ned_m_s - truth.velocity_ned))
        tr, tp, ty = truth.attitude.to_euler_rad()
        er, ep, ey = est.attitude.to_euler_rad()
        att_err_deg = float(np.hypot(tr - er, tp - ep) * RAD_TO_DEG)

        if t > 5.0:
            sum_pos_sq += pos_err ** 2
            sum_vel_sq += vel_err ** 2
            sum_att_sq += att_err_deg ** 2
            n_samples += 1

        if keep_log:
            result.log.append(LogRow(
                t=t, true_p=truth.position_ned, est_p=est.position_ned_m,
                true_v=truth.velocity_ned, est_v=est.velocity_ned_m_s,
                true_euler_deg=(tr * RAD_TO_DEG, tp * RAD_TO_DEG, ty * RAD_TO_DEG),
                est_euler_deg=(er * RAD_TO_DEG, ep * RAD_TO_DEG, ey * RAD_TO_DEG),
                gps_fused=gps_fused,
                est_gyro_bias=est.gyro_bias_rad_s, est_accel_bias=est.accel_bias_m_s2,
            ))

        if i == n_steps:
            result.final_gyro_bias_error = est.gyro_bias_rad_s - scenario_params.true_gyro_bias
            result.final_accel_bias_error = est.accel_bias_m_s2 - scenario_params.true_accel_bias

    n_samples = max(1, n_samples)
    result.pos_rmse_m = float(np.sqrt(sum_pos_sq / n_samples))
    result.vel_rmse_m_s = float(np.sqrt(sum_vel_sq / n_samples))
    result.att_rmse_deg = float(np.sqrt(sum_att_sq / n_samples))
    return result
