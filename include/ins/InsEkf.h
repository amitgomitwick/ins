// GPS-aided strapdown Inertial Navigation System for a UAV.
//
// Architecture: 15-state error-state (indirect) Extended Kalman Filter.
//   Nominal state (integrated every IMU sample, not part of the Kalman
//   covariance directly):
//     p   - position, NED, meters
//     v   - velocity, NED, m/s
//     q   - attitude quaternion, body -> NED
//     bg  - gyro bias, body frame, rad/s
//     ba  - accelerometer bias, body frame, m/s^2
//
//   Error state (15x1, what the covariance P actually tracks):
//     [delta_p(3), delta_v(3), delta_theta(3), delta_bg(3), delta_ba(3)]
//   where delta_theta is a small-angle rotation-vector attitude error.
//
// GPS position/velocity are fused with sequential scalar updates (one axis
// at a time) rather than a block matrix inversion, since the measurement
// noise on each axis is independent. That keeps every matrix operation on
// this 15-state filter to O(N) or O(N^2) vector/row ops -- no matrix
// inverse anywhere -- which is the standard trick for running an EKF like
// this on a flight controller.
//
// This is intentionally standalone (no ArduPilot headers) so it can be
// built, tested and understood on its own. See
// docs/ardupilot_integration.md for how to wire it into ArduPilot as a
// custom estimator backend.
#pragma once

#include <array>

#include "ins/GpsSample.h"
#include "ins/ImuSample.h"
#include "ins/MagSample.h"
#include "ins/Math3.h"

namespace ins {

constexpr int kNumStates = 15;
using StateVector = std::array<double, kNumStates>;
using Covariance = std::array<std::array<double, kNumStates>, kNumStates>;

// State-vector index layout (see error-state list above).
enum StateIndex {
    kP0 = 0,   // position error x,y,z -> 0,1,2
    kV0 = 3,   // velocity error x,y,z -> 3,4,5
    kTheta0 = 6,  // attitude error x,y,z -> 6,7,8
    kBg0 = 9,     // gyro bias error x,y,z -> 9,10,11
    kBa0 = 12,    // accel bias error x,y,z -> 12,13,14
};

struct InsEkfConfig {
    double gravity_m_s2 = 9.80665;

    // Continuous-time noise spectral densities (typical low-cost MEMS IMU).
    double gyro_noise_density = 0.01;        // rad/s / sqrt(Hz)
    double accel_noise_density = 0.05;       // m/s^2 / sqrt(Hz)
    double gyro_bias_instability = 2.0e-4;   // rad/s / sqrt(Hz)  (bias random walk)
    double accel_bias_instability = 2.0e-3;  // m/s^2 / sqrt(Hz) (bias random walk)

    // GPS measurement noise (1-sigma).
    double gps_pos_noise_m = 1.5;
    double gps_vel_noise_m_s = 0.2;

    // Magnetometer heading measurement noise (1-sigma, radians) and local
    // magnetic declination (radians, positive = magnetic north is east of
    // true north) -- set this to your location's actual declination
    // (e.g. from the NOAA/WMM calculator ArduPilot itself uses) or yaw
    // will be a magnetic-north estimate offset from true north by however
    // wrong this value is. Defaults to 0 (i.e. "assume no declination").
    double mag_yaw_noise_rad = 5.0 * kDegToRad;
    double mag_declination_rad = 0.0;

    // Initial covariance (1-sigma) used by init().
    double init_pos_std_m = 5.0;
    double init_vel_std_m_s = 1.0;
    double init_att_std_rad = 5.0 * kDegToRad;
    double init_yaw_std_rad = 15.0 * kDegToRad;
    double init_gyro_bias_std = 0.05;
    double init_accel_bias_std = 0.3;
};

// Estimated state, exposed as a plain struct for logging/consumers.
struct InsState {
    double timestamp_s = 0.0;
    Vector3 position_ned_m;
    Vector3 velocity_ned_m_s;
    Quaternion attitude;
    Vector3 gyro_bias_rad_s;
    Vector3 accel_bias_m_s2;

    void eulerDeg(double& roll, double& pitch, double& yaw) const {
        double r, p, y;
        attitude.toEulerRad(r, p, y);
        roll = r * kRadToDeg;
        pitch = p * kRadToDeg;
        yaw = y * kRadToDeg;
    }
};

class InsEkf {
public:
    explicit InsEkf(const InsEkfConfig& config = InsEkfConfig());

    // Initialize the filter from a single IMU sample, assuming the vehicle
    // is stationary and roughly level: roll/pitch are computed from the
    // measured gravity direction, yaw is set to 0 (true north) since a
    // single accelerometer reading cannot observe heading. Position and
    // velocity start at the given origin (typically {0,0,0} and zero).
    void init(const ImuSample& first_sample, const Vector3& initial_position_ned_m = Vector3());

    bool isInitialized() const { return initialized_; }

    // Strapdown mechanization + covariance propagation. Call at IMU rate.
    void predict(const ImuSample& sample);

    // GPS position/velocity update (sequential scalar fusion). Call
    // whenever a new fix arrives; safe to call with only one of
    // position_valid/velocity_valid set.
    void fuseGps(const GpsSample& gps);

    // Magnetometer heading update: tilt-compensates the raw reading using
    // the filter's current roll/pitch estimate, derives a heading, and
    // fuses it as a single scalar update on the yaw error state -- the
    // direct observation GPS alone can't provide (see docs/architecture.md).
    // Call at the magnetometer's own sample rate (it's typically slower
    // than the IMU).
    void fuseMag(const MagSample& mag);

    InsState state() const;

    const Covariance& covariance() const { return P_; }

private:
    void fuseScalar(int state_idx, double innovation, double measurement_noise_var);
    void injectErrorState();

    InsEkfConfig config_;
    bool initialized_ = false;
    double last_time_s_ = 0.0;

    // Nominal state.
    Vector3 p_;
    Vector3 v_;
    Quaternion q_;
    Vector3 bg_;
    Vector3 ba_;

    // Error-state covariance.
    Covariance P_{};

    // Accumulated (not-yet-injected) error-state correction from the most
    // recent fuseGps() call; scratch space, not part of the public API.
    StateVector delta_x_{};
};

}  // namespace ins
