#include "ins/InsEkf.h"

#include <algorithm>
#include <cmath>

namespace ins {

namespace {

Covariance zeroCov() {
    Covariance c{};
    for (auto& row : c) row.fill(0.0);
    return c;
}

Covariance identityCov() {
    Covariance c = zeroCov();
    for (int i = 0; i < kNumStates; i++) c[i][i] = 1.0;
    return c;
}

// out = a * b  (dense NxN multiply; N=15 is small enough that this need not
// exploit F's block-sparsity for a project of this scope).
Covariance matMul(const Covariance& a, const Covariance& b) {
    Covariance out = zeroCov();
    for (int i = 0; i < kNumStates; i++) {
        for (int k = 0; k < kNumStates; k++) {
            const double aik = a[i][k];
            if (aik == 0.0) continue;
            for (int j = 0; j < kNumStates; j++) out[i][j] += aik * b[k][j];
        }
    }
    return out;
}

Covariance transposeCov(const Covariance& a) {
    Covariance out = zeroCov();
    for (int i = 0; i < kNumStates; i++)
        for (int j = 0; j < kNumStates; j++) out[i][j] = a[j][i];
    return out;
}

void setBlock3(Covariance& m, int row0, int col0, const Matrix3& b) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) m[row0 + i][col0 + j] = b.m[i][j];
}

}  // namespace

InsEkf::InsEkf(const InsEkfConfig& config) : config_(config) {}

void InsEkf::init(const ImuSample& first_sample, const Vector3& initial_position_ned_m) {
    // Align measured specific force to the known NED "gravity reaction"
    // direction [0,0,-1] using the minimal (shortest-arc) rotation, then
    // keep only its roll/pitch content -- yaw is unobservable from a
    // single accelerometer sample and is initialized to 0 with large
    // uncertainty instead.
    const Vector3 b = first_sample.accel_m_s2.normalized();
    const Vector3 n(0.0, 0.0, -1.0);
    Quaternion q_align;
    const Vector3 axis = b.cross(n);
    const double s = axis.length();
    const double c = b.dot(n);
    if (s < 1e-9) {
        q_align = (c > 0.0) ? Quaternion(1, 0, 0, 0) : Quaternion::fromRotationVector(Vector3(M_PI, 0, 0));
    } else {
        q_align = Quaternion::fromRotationVector(axis * (std::atan2(s, c) / s));
    }
    double roll, pitch, yaw_unused;
    q_align.toEulerRad(roll, pitch, yaw_unused);
    q_ = Quaternion::fromEulerRad(roll, pitch, 0.0);

    p_ = initial_position_ned_m;
    v_ = Vector3();
    bg_ = Vector3();
    ba_ = Vector3();
    last_time_s_ = first_sample.timestamp_s;

    P_ = zeroCov();
    for (int i = 0; i < 3; i++) {
        P_[kP0 + i][kP0 + i] = config_.init_pos_std_m * config_.init_pos_std_m;
        P_[kV0 + i][kV0 + i] = config_.init_vel_std_m_s * config_.init_vel_std_m_s;
        P_[kBg0 + i][kBg0 + i] = config_.init_gyro_bias_std * config_.init_gyro_bias_std;
        P_[kBa0 + i][kBa0 + i] = config_.init_accel_bias_std * config_.init_accel_bias_std;
    }
    P_[kTheta0 + 0][kTheta0 + 0] = config_.init_att_std_rad * config_.init_att_std_rad;
    P_[kTheta0 + 1][kTheta0 + 1] = config_.init_att_std_rad * config_.init_att_std_rad;
    P_[kTheta0 + 2][kTheta0 + 2] = config_.init_yaw_std_rad * config_.init_yaw_std_rad;

    delta_x_.fill(0.0);
    initialized_ = true;
}

void InsEkf::predict(const ImuSample& sample) {
    const double dt = sample.timestamp_s - last_time_s_;
    if (!initialized_ || dt <= 0.0) {
        last_time_s_ = sample.timestamp_s;
        return;
    }
    last_time_s_ = sample.timestamp_s;

    const Vector3 gyro = sample.gyro_rad_s - bg_;
    const Vector3 accel = sample.accel_m_s2 - ba_;

    // --- Nominal state propagation (strapdown mechanization) ---
    const Quaternion dq = Quaternion::fromRotationVector(gyro * dt);
    Quaternion q_new = q_ * dq;
    q_new.normalize();
    const Matrix3 R_new = q_new.toMatrix();  // body -> NED, post-update attitude

    // Accelerometer measures specific force f_body; true NED acceleration
    // is a = R*f_body + g_ned, with g_ned = [0,0,+g] (gravity pulls down,
    // NED z is down). See docs/architecture.md for the sign derivation.
    const Vector3 g_ned(0.0, 0.0, config_.gravity_m_s2);
    const Vector3 accel_ned = R_new * accel + g_ned;

    const Vector3 v_old = v_;
    v_ = v_ + accel_ned * dt;
    p_ = p_ + v_old * dt + accel_ned * (0.5 * dt * dt);
    q_ = q_new;

    // --- Error-state covariance propagation ---
    // Discrete-time linearized error-state transition F_d = I + Fc*dt,
    // built directly in block form (see the derivation in
    // docs/architecture.md):
    //   delta_p_dot     = delta_v
    //   delta_v_dot     = -R*skew(f)*delta_theta - R*delta_ba
    //   delta_theta_dot = -skew(w)*delta_theta - delta_bg
    // (delta_bg, delta_ba have no deterministic dynamics; random-walk noise
    // only, added to Q below.)
    Covariance Fd = identityCov();
    const Matrix3 I3;  // default-constructed = identity
    setBlock3(Fd, kP0, kV0, I3 * dt);
    setBlock3(Fd, kV0, kTheta0, (R_new * skewSymmetric(accel)) * (-dt));
    setBlock3(Fd, kV0, kBa0, R_new * (-dt));
    setBlock3(Fd, kTheta0, kTheta0, I3 + skewSymmetric(gyro) * (-dt));
    setBlock3(Fd, kTheta0, kBg0, I3 * (-dt));

    Covariance FdT = transposeCov(Fd);
    P_ = matMul(matMul(Fd, P_), FdT);

    const double q_v = config_.accel_noise_density * config_.accel_noise_density * dt;
    const double q_theta = config_.gyro_noise_density * config_.gyro_noise_density * dt;
    const double q_bg = config_.gyro_bias_instability * config_.gyro_bias_instability * dt;
    const double q_ba = config_.accel_bias_instability * config_.accel_bias_instability * dt;
    for (int i = 0; i < 3; i++) {
        P_[kV0 + i][kV0 + i] += q_v;
        P_[kTheta0 + i][kTheta0 + i] += q_theta;
        P_[kBg0 + i][kBg0 + i] += q_bg;
        P_[kBa0 + i][kBa0 + i] += q_ba;
    }
}

void InsEkf::fuseScalar(int state_idx, double innovation, double measurement_noise_var) {
    const double S = P_[state_idx][state_idx] + measurement_noise_var;
    if (S < 1e-15) return;

    std::array<double, kNumStates> K{};
    for (int i = 0; i < kNumStates; i++) K[i] = P_[i][state_idx] / S;

    for (int i = 0; i < kNumStates; i++) delta_x_[i] += K[i] * innovation;

    const std::array<double, kNumStates> Hrow = P_[state_idx];  // H is a unit row vector
    for (int i = 0; i < kNumStates; i++)
        for (int j = 0; j < kNumStates; j++) P_[i][j] -= K[i] * Hrow[j];
}

void InsEkf::injectErrorState() {
    p_ += Vector3(delta_x_[kP0], delta_x_[kP0 + 1], delta_x_[kP0 + 2]);
    v_ += Vector3(delta_x_[kV0], delta_x_[kV0 + 1], delta_x_[kV0 + 2]);

    const Vector3 dtheta(delta_x_[kTheta0], delta_x_[kTheta0 + 1], delta_x_[kTheta0 + 2]);
    q_ = q_ * Quaternion::fromRotationVector(dtheta);
    q_.normalize();

    bg_ += Vector3(delta_x_[kBg0], delta_x_[kBg0 + 1], delta_x_[kBg0 + 2]);
    ba_ += Vector3(delta_x_[kBa0], delta_x_[kBa0 + 1], delta_x_[kBa0 + 2]);

    delta_x_.fill(0.0);
}

void InsEkf::fuseGps(const GpsSample& gps) {
    if (!initialized_) return;
    delta_x_.fill(0.0);

    if (gps.position_valid) {
        for (int axis = 0; axis < 3; axis++) {
            const double innovation = gps.position_ned_m[axis] - p_[axis];
            fuseScalar(kP0 + axis, innovation, config_.gps_pos_noise_m * config_.gps_pos_noise_m);
        }
    }
    if (gps.velocity_valid) {
        for (int axis = 0; axis < 3; axis++) {
            const double innovation = gps.velocity_ned_m_s[axis] - v_[axis];
            fuseScalar(kV0 + axis, innovation, config_.gps_vel_noise_m_s * config_.gps_vel_noise_m_s);
        }
    }

    injectErrorState();
}

InsState InsEkf::state() const {
    InsState s;
    s.timestamp_s = last_time_s_;
    s.position_ned_m = p_;
    s.velocity_ned_m_s = v_;
    s.attitude = q_;
    s.gyro_bias_rad_s = bg_;
    s.accel_bias_m_s2 = ba_;
    return s;
}

}  // namespace ins
