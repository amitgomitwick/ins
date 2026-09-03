// Minimal, dependency-free sanity tests for InsEkf (no test framework, just
// asserts -> nonzero exit on failure, wired into `ctest`).
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "FlightScenario.h"
#include "ins/InsEkf.h"

namespace {

int g_failures = 0;

void expectTrue(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        g_failures++;
    } else {
        std::printf("PASS: %s\n", what);
    }
}

// A perfectly level, stationary, noise-free IMU should leave the filter's
// position/velocity essentially unchanged and attitude level.
void testStationaryLevel() {
    ins::InsEkfConfig cfg;
    ins::InsEkf ekf(cfg);

    ins::ImuSample s;
    s.timestamp_s = 0.0;
    s.gyro_rad_s = ins::Vector3(0, 0, 0);
    s.accel_m_s2 = ins::Vector3(0, 0, -cfg.gravity_m_s2);
    ekf.init(s);

    for (int i = 1; i <= 1000; i++) {
        s.timestamp_s = i * 0.005;  // 200 Hz for 5 s
        ekf.predict(s);
    }

    const ins::InsState st = ekf.state();
    double roll, pitch, yaw;
    st.eulerDeg(roll, pitch, yaw);

    expectTrue(st.position_ned_m.length() < 0.01, "stationary: position stays near zero");
    expectTrue(st.velocity_ned_m_s.length() < 0.01, "stationary: velocity stays near zero");
    expectTrue(std::fabs(roll) < 0.1 && std::fabs(pitch) < 0.1, "stationary: attitude stays level");
}

// Initialization from a tilted-but-stationary IMU sample should recover the
// correct roll/pitch (yaw is unobservable and expected to stay at 0).
void testLevelingInit() {
    ins::InsEkfConfig cfg;
    ins::InsEkf ekf(cfg);

    const double true_roll = 10.0 * ins::kDegToRad;
    const double true_pitch = -5.0 * ins::kDegToRad;
    const ins::Quaternion q_true = ins::Quaternion::fromEulerRad(true_roll, true_pitch, 30.0 * ins::kDegToRad);
    const ins::Vector3 g_ned(0, 0, cfg.gravity_m_s2);
    const ins::Vector3 f_ned = -g_ned;  // stationary
    const ins::Vector3 f_body = q_true.toMatrix().transposed() * f_ned;

    ins::ImuSample s;
    s.timestamp_s = 0.0;
    s.gyro_rad_s = ins::Vector3(0, 0, 0);
    s.accel_m_s2 = f_body;
    ekf.init(s);

    double roll, pitch, yaw;
    ekf.state().eulerDeg(roll, pitch, yaw);
    expectTrue(std::fabs(roll - 10.0) < 0.5, "leveling init: recovers roll");
    expectTrue(std::fabs(pitch - (-5.0)) < 0.5, "leveling init: recovers pitch");
    expectTrue(std::fabs(yaw) < 0.5, "leveling init: yaw defaults to 0 (unobservable)");
}

// End-to-end: the full synthetic flight (with sensor noise, bias, and a
// mid-turn GPS dropout) should still converge to bounded position/velocity
// error and recover the true sensor biases reasonably well.
void testFullFlightConverges() {
    ins_sim::ScenarioParams scenario_params;
    ins::InsEkfConfig filter_config;

    const auto result = ins_sim::runSimulation(scenario_params, filter_config, /*seed=*/1, /*keep_log=*/false);

    std::printf("  pos_rmse=%.2fm vel_rmse=%.2fm/s att_rmse=%.2fdeg\n", result.pos_rmse_m, result.vel_rmse_m_s,
                result.att_rmse_deg);

    // Bounds are set from the empirical spread across many seeds (see
    // docs/architecture.md's validation section), not tight theoretical
    // limits. Roll/pitch RMSE of several degrees is expected here: with a
    // 5 Hz, meter-level GPS and no magnetometer, attitude is only
    // corrected indirectly (through velocity/position innovations), so it
    // converges slower and noisier than position/velocity do.
    expectTrue(result.pos_rmse_m < 6.0, "full flight: position RMSE bounded");
    expectTrue(result.vel_rmse_m_s < 1.0, "full flight: velocity RMSE bounded");
    expectTrue(result.att_rmse_deg < 8.0, "full flight: roll/pitch RMSE bounded");
    expectTrue(result.final_gyro_bias_error.length() < 0.01, "full flight: gyro bias estimated");
    expectTrue(result.final_accel_bias_error.length() < 0.2, "full flight: accel bias estimated");
}

}  // namespace

int main() {
    testStationaryLevel();
    testLevelingInit();
    testFullFlightConverges();

    if (g_failures > 0) {
        std::fprintf(stderr, "\n%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("\nAll tests passed\n");
    return 0;
}
