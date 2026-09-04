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

// Magnetometer fusion in isolation: starting from a wrong yaw estimate,
// repeated fuseMag() calls with a fixed (noise-free) reading at a known
// declination should converge the estimate to the true heading. This is
// the fix for the yaw-drift limitation noted in earlier versions of this
// file/README -- see docs/architecture.md for the derivation.
void testMagYawFusionConverges() {
    ins::InsEkfConfig cfg;
    const double declination = 8.0 * ins::kDegToRad;
    cfg.mag_declination_rad = declination;
    ins::InsEkf ekf(cfg);

    ins::ImuSample s;
    s.timestamp_s = 0.0;
    s.gyro_rad_s = ins::Vector3(0, 0, 0);
    s.accel_m_s2 = ins::Vector3(0, 0, -cfg.gravity_m_s2);  // level, so init() yaw = 0
    ekf.init(s);

    const double true_yaw = 30.0 * ins::kDegToRad;
    const ins::Quaternion q_true = ins::Quaternion::fromEulerRad(0.0, 0.0, true_yaw);
    const ins::Vector3 mag_ned(std::cos(declination), std::sin(declination), 0.8);
    ins::MagSample mag;
    mag.timestamp_s = 0.0;
    mag.field_body = q_true.toMatrix().transposed() * mag_ned;

    for (int i = 0; i < 8; i++) ekf.fuseMag(mag);

    double roll, pitch, yaw;
    ekf.state().eulerDeg(roll, pitch, yaw);
    expectTrue(std::fabs(yaw - 30.0) < 1.0, "mag fusion: converges to true yaw given correct declination");
}

// Innovation gating: a single wildly implausible GPS fix (a stand-in for a
// spoofed or glitched one) should be rejected outright, not blindly
// trusted -- but if that same implausible-looking fix keeps arriving
// consistently, the filter should eventually resync to it rather than
// staying locked out forever (the post-jamming-reacquisition case). This
// also demonstrates the real limitation discussed for spoofing: a
// *sustained*, consistent fake position eventually gets through, by
// design -- gating catches a sudden jump, not a patient attacker.
void testGpsInnovationGating() {
    ins::InsEkfConfig cfg;
    ins::InsEkf ekf(cfg);

    ins::ImuSample level;
    level.gyro_rad_s = ins::Vector3(0, 0, 0);
    level.accel_m_s2 = ins::Vector3(0, 0, -cfg.gravity_m_s2);
    level.timestamp_s = 0.0;
    ekf.init(level);

    // A few steps + a couple of genuine GPS fixes to let P shrink to a
    // realistic in-flight size (a huge just-initialized P would make
    // almost anything pass the gate, which would defeat the point of this
    // test).
    double t = 0.0;
    for (int i = 0; i < 200; i++) {
        t += 0.005;
        level.timestamp_s = t;
        ekf.predict(level);
        if (i % 40 == 0) {
            ins::GpsSample gps;
            gps.timestamp_s = t;
            gps.position_ned_m = ins::Vector3(0, 0, 0);
            gps.velocity_ned_m_s = ins::Vector3(0, 0, 0);
            ekf.fuseGps(gps);
        }
    }

    // A fix 500m away on every axis is not a plausible next position for a
    // filter whose uncertainty is now on the order of meters.
    // fuseGps()'s return value is "any axis accepted", so all three axes
    // (and velocity) are offset here -- otherwise a correctly-matching
    // axis trivially passing its own gate would mask whether the *other*,
    // genuinely implausible axes were rejected.
    ins::GpsSample spoof;
    spoof.timestamp_s = t + 0.2;
    spoof.position_ned_m = ins::Vector3(500.0, 500.0, 500.0);
    spoof.velocity_ned_m_s = ins::Vector3(50.0, 50.0, 50.0);
    const bool first_accepted = ekf.fuseGps(spoof);

    expectTrue(!first_accepted, "gating: a single wild GPS jump is rejected");
    expectTrue(ekf.state().position_ned_m.length() < 5.0, "gating: rejected fix doesn't move the estimate");

    // The same "implausible" fix keeps arriving, consistently, past the
    // reset timeout -- e.g. GPS actually did reacquire after the filter's
    // own dead-reckoning drifted, or (the honest downside) a sustained
    // spoof. A forced-accept is a normal (if belated) Kalman update, not a
    // snap-to-measurement -- with P still small at that point, one forced
    // update only partially closes a 500m gap, and since it shrinks P
    // further, the *next* fix may need to wait out another reset timeout
    // before it, too, is forced through. Full convergence can take several
    // such cycles, so this runs long enough (~40s of persistent "spoofing")
    // to cover a few of them, rather than stopping at the first accept.
    double spoof_t = spoof.timestamp_s;
    bool ever_accepted = false;
    for (int i = 0; i < 200; i++) {
        spoof_t += 0.2;
        spoof.timestamp_s = spoof_t;
        if (ekf.fuseGps(spoof)) ever_accepted = true;
    }

    expectTrue(ever_accepted, "gating: a persistent fix eventually resyncs rather than locking out forever");
    expectTrue(std::fabs(ekf.state().position_ned_m.x - 500.0) < 20.0,
               "gating: after sustained resync the estimate converges to the persistent fix");
}

// End-to-end: the full synthetic flight (with sensor noise, bias, a
// mid-turn GPS dropout, and magnetometer aiding) should converge to bounded
// error on every state, including yaw.
void testFullFlightConverges() {
    ins_sim::ScenarioParams scenario_params;
    ins::InsEkfConfig filter_config;
    filter_config.mag_declination_rad = scenario_params.true_mag_declination_rad;

    const auto result = ins_sim::runSimulation(scenario_params, filter_config, /*seed=*/1, /*keep_log=*/false);

    std::printf("  pos_rmse=%.2fm vel_rmse=%.2fm/s roll_pitch_rmse=%.2fdeg yaw_rmse=%.2fdeg\n", result.pos_rmse_m,
                result.vel_rmse_m_s, result.att_rmse_deg, result.yaw_rmse_deg);

    // Bounds are set from the empirical spread across many seeds (see
    // docs/architecture.md's validation section), not tight theoretical
    // limits.
    expectTrue(result.pos_rmse_m < 6.0, "full flight: position RMSE bounded");
    expectTrue(result.vel_rmse_m_s < 1.5, "full flight: velocity RMSE bounded");
    expectTrue(result.att_rmse_deg < 4.0, "full flight: roll/pitch RMSE bounded");
    expectTrue(result.yaw_rmse_deg < 4.0, "full flight: yaw RMSE bounded (magnetometer-aided)");
    expectTrue(result.final_gyro_bias_error.length() < 0.01, "full flight: gyro bias estimated");
    expectTrue(result.final_accel_bias_error.length() < 0.4, "full flight: accel bias estimated");
}

}  // namespace

int main() {
    testStationaryLevel();
    testLevelingInit();
    testMagYawFusionConverges();
    testGpsInnovationGating();
    testFullFlightConverges();

    if (g_failures > 0) {
        std::fprintf(stderr, "\n%d test(s) FAILED\n", g_failures);
        return 1;
    }
    std::printf("\nAll tests passed\n");
    return 0;
}
