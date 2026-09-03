// Runs the synthetic UAV flight through InsEkf and writes a CSV of
// true-vs-estimated state for inspection/plotting.
//
// Usage: sim_flight [output.csv]
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "FlightScenario.h"

int main(int argc, char** argv) {
    const std::string out_path = argc > 1 ? argv[1] : "flight_log.csv";

    ins_sim::ScenarioParams scenario_params;
    ins::InsEkfConfig filter_config;

    const auto result = ins_sim::runSimulation(scenario_params, filter_config, /*seed=*/42, /*keep_log=*/true);

    std::ofstream f(out_path);
    if (!f) {
        std::cerr << "Failed to open " << out_path << " for writing\n";
        return 1;
    }
    f << "t,true_n,true_e,true_d,est_n,est_e,est_d,"
         "true_vn,true_ve,true_vd,est_vn,est_ve,est_vd,"
         "true_roll,true_pitch,true_yaw,est_roll,est_pitch,est_yaw,"
         "gps_fused,gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z\n";
    for (const auto& r : result.log) {
        f << r.t << ',' << r.true_p.x << ',' << r.true_p.y << ',' << r.true_p.z << ',' << r.est_p.x << ','
          << r.est_p.y << ',' << r.est_p.z << ',' << r.true_v.x << ',' << r.true_v.y << ',' << r.true_v.z << ','
          << r.est_v.x << ',' << r.est_v.y << ',' << r.est_v.z << ',' << r.true_roll_deg << ',' << r.true_pitch_deg
          << ',' << r.true_yaw_deg << ',' << r.est_roll_deg << ',' << r.est_pitch_deg << ',' << r.est_yaw_deg << ','
          << (r.gps_fused ? 1 : 0) << ',' << r.est_gyro_bias.x << ',' << r.est_gyro_bias.y << ','
          << r.est_gyro_bias.z << ',' << r.est_accel_bias.x << ',' << r.est_accel_bias.y << ',' << r.est_accel_bias.z
          << '\n';
    }

    std::printf("Wrote %zu rows to %s\n\n", result.log.size(), out_path.c_str());
    std::printf("Position RMSE : %.3f m\n", result.pos_rmse_m);
    std::printf("Velocity RMSE : %.3f m/s\n", result.vel_rmse_m_s);
    std::printf("Attitude RMSE : %.3f deg\n", result.att_rmse_deg);
    std::printf("Final gyro bias error  : [%.4f, %.4f, %.4f] rad/s\n", result.final_gyro_bias_error.x,
                result.final_gyro_bias_error.y, result.final_gyro_bias_error.z);
    std::printf("Final accel bias error : [%.4f, %.4f, %.4f] m/s^2\n", result.final_accel_bias_error.x,
                result.final_accel_bias_error.y, result.final_accel_bias_error.z);
    return 0;
}
