// Synthetic UAV flight used to exercise and validate InsEkf: a straight
// accelerate-away leg, a coordinated turn, and a straight decelerate-in
// leg, all at constant altitude. The trajectory is built so that attitude,
// body rates and specific force are all kinematically self-consistent
// (i.e. this is a real, flyable rigid-body motion, not just a curve with
// invented sensor values) -- see docs/architecture.md for the derivation.
//
// Example/test-only code: not part of the ins_ekf library.
#pragma once

#include <cmath>
#include <random>
#include <vector>

#include "ins/InsEkf.h"

namespace ins_sim {

using ins::Vector3;
using ins::Quaternion;

struct ScenarioParams {
    double accel_phase_s = 10.0;    // ramp 0 -> cruise speed
    double turn_phase_s = 90.0;     // coordinated turn at cruise speed
    double decel_phase_s = 20.0;    // ramp cruise speed -> 0
    double roll_ramp_s = 3.0;       // bank in/out time at turn entry/exit
    double cruise_speed_m_s = 15.0;
    double turn_radius_m = 200.0;
    double gravity_m_s2 = 9.80665;

    double imu_rate_hz = 200.0;
    double gps_rate_hz = 5.0;
    double gps_dropout_start_s = 40.0;  // simulated GPS outage mid-turn,
    double gps_dropout_end_s = 50.0;    // to show the INS coasting on IMU alone.
    double mag_rate_hz = 25.0;          // magnetometer keeps running through the GPS dropout

    // "True" sensor imperfections the filter must estimate/reject.
    Vector3 true_gyro_bias{0.020, -0.015, 0.010};      // rad/s
    Vector3 true_accel_bias{0.15, -0.10, 0.08};        // m/s^2

    // A realistic nonzero declination, so the sim genuinely exercises the
    // declination-compensation term in InsEkf::fuseMag rather than trivially
    // passing with it at zero.
    double true_mag_declination_rad = 8.0 * ins::kDegToRad;

    double totalDuration() const { return accel_phase_s + turn_phase_s + decel_phase_s; }
};

struct TrueState {
    double t = 0.0;
    Vector3 position_ned;
    Vector3 velocity_ned;
    Quaternion attitude;
    Vector3 body_rate;       // rad/s
    Vector3 specific_force;  // body frame, m/s^2
};

// Cubic smoothstep ramp of `t` over [t0, t0+duration], zero slope at both
// ends (C1 continuous) so kinematics stay smooth with no rate spikes.
inline double rampValue(double t, double t0, double duration) {
    double u = (t - t0) / duration;
    u = std::max(0.0, std::min(1.0, u));
    return u * u * (3.0 - 2.0 * u);
}
inline double rampDeriv(double t, double t0, double duration) {
    double u = (t - t0) / duration;
    if (u <= 0.0 || u >= 1.0) return 0.0;
    return (6.0 * u - 6.0 * u * u) / duration;
}

class FlightScenario {
public:
    explicit FlightScenario(const ScenarioParams& params) : p_(params) {
        buildYawTable();
    }

    TrueState evaluate(double t) const {
        const double T1 = p_.accel_phase_s;
        const double T2 = p_.turn_phase_s;
        const double v0 = p_.cruise_speed_m_s;

        double speed, speed_dot, roll, roll_dot, yaw, yaw_dot;

        if (t < T1) {
            speed = v0 * rampValue(t, 0.0, T1);
            speed_dot = v0 * rampDeriv(t, 0.0, T1);
            roll = 0.0;
            roll_dot = 0.0;
            yaw = 0.0;
            yaw_dot = 0.0;
        } else if (t < T1 + T2) {
            const double u = t - T1;
            const double Tr = p_.roll_ramp_s;
            const double target_phi = std::atan2(v0 * v0 / p_.turn_radius_m, p_.gravity_m_s2);
            roll = target_phi * (rampValue(u, 0.0, Tr) - rampValue(u, T2 - Tr, Tr));
            roll_dot = target_phi * (rampDeriv(u, 0.0, Tr) - rampDeriv(u, T2 - Tr, Tr));
            speed = v0;
            speed_dot = 0.0;
            yaw_dot = p_.gravity_m_s2 * std::tan(roll) / v0;
            yaw = lookupYaw(u);
        } else {
            const double u = t - T1 - T2;
            const double T3 = p_.decel_phase_s;
            speed = v0 * (1.0 - rampValue(u, 0.0, T3));
            speed_dot = -v0 * rampDeriv(u, 0.0, T3);
            roll = 0.0;
            roll_dot = 0.0;
            yaw_dot = 0.0;
            yaw = yaw_table_.back();
        }

        TrueState s;
        s.t = t;
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        s.velocity_ned = Vector3(speed * cy, speed * sy, 0.0);
        // a_ned = d/dt [speed*cos(yaw), speed*sin(yaw), 0]; by construction
        // speed_dot and yaw_dot are never simultaneously nonzero.
        const Vector3 a_ned(speed_dot * cy - speed * yaw_dot * sy,
                             speed_dot * sy + speed * yaw_dot * cy, 0.0);
        s.attitude = Quaternion::fromEulerRad(roll, 0.0, yaw);
        const Vector3 g_ned(0.0, 0.0, p_.gravity_m_s2);
        const Vector3 f_ned = a_ned - g_ned;
        s.specific_force = s.attitude.toMatrix().transposed() * f_ned;  // NED -> body

        s.body_rate = Vector3(roll_dot, yaw_dot * std::sin(roll), yaw_dot * std::cos(roll));

        s.position_ned = integratePosition(t);
        return s;
    }

    double totalDuration() const { return p_.totalDuration(); }
    const ScenarioParams& params() const { return p_; }

private:
    void buildYawTable() {
        const double dt = 0.001;
        const double T2 = p_.turn_phase_s;
        const int n = static_cast<int>(T2 / dt) + 2;
        yaw_table_.resize(n);
        yaw_table_[0] = 0.0;
        yaw_table_dt_ = dt;
        double yaw = 0.0;
        for (int i = 1; i < n; i++) {
            const double u = i * dt;
            // roll(u) replicated here to derive yaw_dot(u); kept in sync
            // with evaluate()'s turn-phase branch.
            const double Tr = p_.roll_ramp_s;
            const double target_phi =
                std::atan2(p_.cruise_speed_m_s * p_.cruise_speed_m_s / p_.turn_radius_m, p_.gravity_m_s2);
            const double roll = target_phi * (rampValue(u, 0.0, Tr) - rampValue(u, T2 - Tr, Tr));
            const double yaw_dot = p_.gravity_m_s2 * std::tan(roll) / p_.cruise_speed_m_s;
            yaw += yaw_dot * dt;
            yaw_table_[i] = yaw;
        }
    }

    double lookupYaw(double u) const {
        if (u <= 0.0) return yaw_table_.front();
        const double idx_f = u / yaw_table_dt_;
        const int idx = static_cast<int>(idx_f);
        if (idx + 1 >= static_cast<int>(yaw_table_.size())) return yaw_table_.back();
        const double frac = idx_f - idx;
        return yaw_table_[idx] * (1.0 - frac) + yaw_table_[idx + 1] * frac;
    }

    // Position is only needed for truth logging/RMSE, not for sensor
    // synthesis, so a coarse-but-accurate numerical integration of
    // velocity_ned(t) (cached, monotonically extended) is sufficient here.
    Vector3 integratePosition(double t_query) const {
        const double dt = 0.005;
        while (pos_cache_t_ < t_query - 1e-9) {
            const TrueState s_now = evaluateVelocityOnly(pos_cache_t_);
            pos_cache_ += s_now.velocity_ned * dt;
            pos_cache_t_ += dt;
        }
        return pos_cache_;
    }

    TrueState evaluateVelocityOnly(double t) const {
        // Avoids infinite recursion with integratePosition(); duplicates
        // the speed/yaw calc from evaluate() but skips position.
        TrueState s;
        const double T1 = p_.accel_phase_s, T2 = p_.turn_phase_s, v0 = p_.cruise_speed_m_s;
        double speed, yaw;
        if (t < T1) {
            speed = v0 * rampValue(t, 0.0, T1);
            yaw = 0.0;
        } else if (t < T1 + T2) {
            speed = v0;
            yaw = lookupYaw(t - T1);
        } else {
            const double u = t - T1 - T2;
            speed = v0 * (1.0 - rampValue(u, 0.0, p_.decel_phase_s));
            yaw = yaw_table_.back();
        }
        s.velocity_ned = Vector3(speed * std::cos(yaw), speed * std::sin(yaw), 0.0);
        return s;
    }

    ScenarioParams p_;
    std::vector<double> yaw_table_;
    double yaw_table_dt_ = 0.001;
    mutable double pos_cache_t_ = 0.0;
    mutable Vector3 pos_cache_{};
};

struct LogRow {
    double t;
    Vector3 true_p, est_p;
    Vector3 true_v, est_v;
    double true_roll_deg, true_pitch_deg, true_yaw_deg;
    double est_roll_deg, est_pitch_deg, est_yaw_deg;
    bool gps_fused;
    Vector3 est_gyro_bias, est_accel_bias;
};

struct SimulationResult {
    double pos_rmse_m = 0.0;
    double vel_rmse_m_s = 0.0;
    double att_rmse_deg = 0.0;   // roll/pitch only
    double yaw_rmse_deg = 0.0;
    Vector3 final_gyro_bias_error;
    Vector3 final_accel_bias_error;
    std::vector<LogRow> log;
};

inline SimulationResult runSimulation(const ScenarioParams& scenario_params,
                                       const ins::InsEkfConfig& filter_config, unsigned seed,
                                       bool keep_log) {
    FlightScenario scenario(scenario_params);
    std::mt19937 rng(seed);
    std::normal_distribution<double> nrm(0.0, 1.0);

    const double imu_dt = 1.0 / scenario_params.imu_rate_hz;
    const double gps_dt = 1.0 / scenario_params.gps_rate_hz;
    const double gyro_sample_std = filter_config.gyro_noise_density * std::sqrt(scenario_params.imu_rate_hz);
    const double accel_sample_std = filter_config.accel_noise_density * std::sqrt(scenario_params.imu_rate_hz);

    ins::InsEkf ekf(filter_config);
    SimulationResult result;

    const double mag_dt = 1.0 / scenario_params.mag_rate_hz;
    double next_gps_t = gps_dt;
    double next_mag_t = mag_dt;
    double sum_pos_sq = 0.0, sum_vel_sq = 0.0, sum_att_sq = 0.0, sum_yaw_sq = 0.0;
    int n_samples = 0;
    const double total_t = scenario.totalDuration();
    const int n_steps = static_cast<int>(total_t / imu_dt);

    for (int i = 0; i <= n_steps; i++) {
        const double t = i * imu_dt;
        const TrueState truth = scenario.evaluate(t);

        ins::ImuSample imu;
        imu.timestamp_s = t;
        imu.gyro_rad_s = truth.body_rate + scenario_params.true_gyro_bias +
                         Vector3(nrm(rng), nrm(rng), nrm(rng)) * gyro_sample_std;
        imu.accel_m_s2 = truth.specific_force + scenario_params.true_accel_bias +
                          Vector3(nrm(rng), nrm(rng), nrm(rng)) * accel_sample_std;

        bool gps_fused = false;
        if (i == 0) {
            ekf.init(imu, Vector3(0, 0, 0));
        } else {
            ekf.predict(imu);

            if (t >= next_gps_t) {
                next_gps_t += gps_dt;
                const bool in_dropout =
                    t >= scenario_params.gps_dropout_start_s && t <= scenario_params.gps_dropout_end_s;
                if (!in_dropout) {
                    ins::GpsSample gps;
                    gps.timestamp_s = t;
                    gps.position_ned_m =
                        truth.position_ned + Vector3(nrm(rng), nrm(rng), nrm(rng)) * filter_config.gps_pos_noise_m;
                    gps.velocity_ned_m_s = truth.velocity_ned +
                                            Vector3(nrm(rng), nrm(rng), nrm(rng)) * filter_config.gps_vel_noise_m_s;
                    ekf.fuseGps(gps);
                    gps_fused = true;
                }
            }

            if (t >= next_mag_t) {
                next_mag_t += mag_dt;
                // Perturb the *assumed magnetic north direction* by the
                // configured noise (rather than the raw body-frame vector)
                // so the injected noise matches the filter's assumed R,
                // same "well-tuned filter" pattern used for GPS above.
                // Runs through the GPS dropout too -- magnetometer heading
                // doesn't depend on GPS being available.
                const double heading_noise = nrm(rng) * filter_config.mag_yaw_noise_rad;
                const double decl = scenario_params.true_mag_declination_rad + heading_noise;
                const Vector3 mag_ned(0.6 * std::cos(decl), 0.6 * std::sin(decl), 0.8);
                ins::MagSample mag;
                mag.timestamp_s = t;
                mag.field_body = truth.attitude.toMatrix().transposed() * mag_ned;
                ekf.fuseMag(mag);
            }
        }

        const ins::InsState est = ekf.state();
        const double pos_err = (est.position_ned_m - truth.position_ned).length();
        const double vel_err = (est.velocity_ned_m_s - truth.velocity_ned).length();
        double tr, tp, ty, er, ep, ey;
        truth.attitude.toEulerRad(tr, tp, ty);
        est.attitude.toEulerRad(er, ep, ey);
        const double att_err_deg =
            std::sqrt((tr - er) * (tr - er) + (tp - ep) * (tp - ep)) * ins::kRadToDeg;
        double yaw_err = ty - ey;
        while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
        const double yaw_err_deg = yaw_err * ins::kRadToDeg;

        if (t > 5.0) {  // skip initial convergence transient for RMSE stats
            sum_pos_sq += pos_err * pos_err;
            sum_vel_sq += vel_err * vel_err;
            sum_att_sq += att_err_deg * att_err_deg;
            sum_yaw_sq += yaw_err_deg * yaw_err_deg;
            n_samples++;
        }

        if (keep_log) {
            LogRow row;
            row.t = t;
            row.true_p = truth.position_ned;
            row.est_p = est.position_ned_m;
            row.true_v = truth.velocity_ned;
            row.est_v = est.velocity_ned_m_s;
            row.true_roll_deg = tr * ins::kRadToDeg;
            row.true_pitch_deg = tp * ins::kRadToDeg;
            row.true_yaw_deg = ty * ins::kRadToDeg;
            row.est_roll_deg = er * ins::kRadToDeg;
            row.est_pitch_deg = ep * ins::kRadToDeg;
            row.est_yaw_deg = ey * ins::kRadToDeg;
            row.gps_fused = gps_fused;
            row.est_gyro_bias = est.gyro_bias_rad_s;
            row.est_accel_bias = est.accel_bias_m_s2;
            result.log.push_back(row);
        }

        if (i == n_steps) {
            result.final_gyro_bias_error = est.gyro_bias_rad_s - scenario_params.true_gyro_bias;
            result.final_accel_bias_error = est.accel_bias_m_s2 - scenario_params.true_accel_bias;
        }
    }

    result.pos_rmse_m = std::sqrt(sum_pos_sq / std::max(1, n_samples));
    result.vel_rmse_m_s = std::sqrt(sum_vel_sq / std::max(1, n_samples));
    result.att_rmse_deg = std::sqrt(sum_att_sq / std::max(1, n_samples));
    result.yaw_rmse_deg = std::sqrt(sum_yaw_sq / std::max(1, n_samples));
    return result;
}

}  // namespace ins_sim
