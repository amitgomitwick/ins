// Replays REAL flight-controller IMU/GPS/magnetometer data (extracted from
// an ArduPilot dataflash log by tools/parse_dataflash_log.py) through
// InsEkf, and -- if the log had EKF3's own output too -- compares against
// it. This is the real-hardware counterpart to examples/sim_flight.cpp's
// synthetic validation: same filter, same architecture, now fed actual
// sensor data instead of a generated trajectory.
//
// Usage:
//   replay_log --imu imu.csv [--gps gps.csv] [--mag mag.csv] [--ekf3 ekf3.csv]
//              [--declination-deg D] [--out replay_output.csv]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ins/InsEkf.h"

namespace {

struct ImuRow {
    double t, gx, gy, gz, ax, ay, az;
};
struct GpsRow {
    double t, lat_deg, lon_deg, alt_m, vn, ve, vd;
};
struct MagRow {
    double t, mx, my, mz;
};
struct Ekf3Row {
    double t, roll_deg, pitch_deg, yaw_deg, vn, ve, vd, pn, pe, pd;
};

std::vector<std::vector<std::string>> readCsv(const std::string& path) {
    std::vector<std::vector<std::string>> rows;
    std::ifstream f(path);
    if (!f) return rows;
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) fields.push_back(cell);
        if (!fields.empty()) rows.push_back(fields);
    }
    return rows;
}

std::vector<ImuRow> loadImu(const std::string& path) {
    std::vector<ImuRow> out;
    for (auto& f : readCsv(path))
        out.push_back({std::stod(f[0]), std::stod(f[1]), std::stod(f[2]), std::stod(f[3]), std::stod(f[4]),
                        std::stod(f[5]), std::stod(f[6])});
    return out;
}

std::vector<GpsRow> loadGps(const std::string& path) {
    std::vector<GpsRow> out;
    for (auto& f : readCsv(path))
        out.push_back({std::stod(f[0]), std::stod(f[1]), std::stod(f[2]), std::stod(f[3]), std::stod(f[4]),
                        std::stod(f[5]), std::stod(f[6])});
    return out;
}

std::vector<MagRow> loadMag(const std::string& path) {
    std::vector<MagRow> out;
    for (auto& f : readCsv(path)) out.push_back({std::stod(f[0]), std::stod(f[1]), std::stod(f[2]), std::stod(f[3])});
    return out;
}

std::vector<Ekf3Row> loadEkf3(const std::string& path) {
    std::vector<Ekf3Row> out;
    for (auto& f : readCsv(path))
        out.push_back({std::stod(f[0]), std::stod(f[1]), std::stod(f[2]), std::stod(f[3]), std::stod(f[4]),
                        std::stod(f[5]), std::stod(f[6]), std::stod(f[7]), std::stod(f[8]), std::stod(f[9])});
    return out;
}

constexpr double kEarthRadiusM = 6371000.0;

// Flat-Earth (equirectangular) projection around a fixed origin -- same
// approximation ArduPilot's own Location offset math uses for local-area
// distances, adequate for a single test drive/flight, not for long range.
struct LlaToNed {
    double origin_lat_rad = 0, origin_lon_rad = 0, origin_alt_m = 0, cos_lat0 = 1;
    bool set = false;

    void setOriginIfNeeded(double lat_deg, double lon_deg, double alt_m) {
        if (set) return;
        origin_lat_rad = lat_deg * ins::kDegToRad;
        origin_lon_rad = lon_deg * ins::kDegToRad;
        origin_alt_m = alt_m;
        cos_lat0 = std::cos(origin_lat_rad);
        set = true;
    }
    ins::Vector3 toNed(double lat_deg, double lon_deg, double alt_m) const {
        const double lat_rad = lat_deg * ins::kDegToRad, lon_rad = lon_deg * ins::kDegToRad;
        return ins::Vector3((lat_rad - origin_lat_rad) * kEarthRadiusM,
                             (lon_rad - origin_lon_rad) * kEarthRadiusM * cos_lat0, -(alt_m - origin_alt_m));
    }
};

std::string arg(int argc, char** argv, const std::string& name, const std::string& def = "") {
    for (int i = 1; i < argc - 1; i++)
        if (name == argv[i]) return argv[i + 1];
    return def;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string imu_path = arg(argc, argv, "--imu");
    const std::string gps_path = arg(argc, argv, "--gps");
    const std::string mag_path = arg(argc, argv, "--mag");
    const std::string ekf3_path = arg(argc, argv, "--ekf3");
    const std::string out_path = arg(argc, argv, "--out", "replay_output.csv");
    const double declination_deg = std::stod(arg(argc, argv, "--declination-deg", "0"));

    if (imu_path.empty()) {
        std::fprintf(stderr, "Usage: replay_log --imu imu.csv [--gps gps.csv] [--mag mag.csv] "
                              "[--ekf3 ekf3.csv] [--declination-deg D] [--out out.csv]\n");
        return 1;
    }

    const auto imu_rows = loadImu(imu_path);
    const auto gps_rows = gps_path.empty() ? std::vector<GpsRow>{} : loadGps(gps_path);
    const auto mag_rows = mag_path.empty() ? std::vector<MagRow>{} : loadMag(mag_path);
    const auto ekf3_rows = ekf3_path.empty() ? std::vector<Ekf3Row>{} : loadEkf3(ekf3_path);

    if (imu_rows.empty()) {
        std::fprintf(stderr, "No IMU rows in %s\n", imu_path.c_str());
        return 1;
    }
    std::printf("Loaded %zu IMU, %zu GPS, %zu MAG, %zu EKF3-reference rows.\n", imu_rows.size(), gps_rows.size(),
                mag_rows.size(), ekf3_rows.size());

    ins::InsEkfConfig cfg;
    cfg.mag_declination_rad = declination_deg * ins::kDegToRad;
    ins::InsEkf ekf(cfg);

    LlaToNed projector;

    std::ofstream out(out_path);
    out << "t,pn,pe,pd,vn,ve,vd,roll_deg,pitch_deg,yaw_deg,gps_fused\n";

    size_t gps_i = 0, mag_i = 0;
    int gps_attempted = 0, gps_accepted = 0, mag_attempted = 0, mag_accepted = 0;

    for (size_t i = 0; i < imu_rows.size(); i++) {
        const ImuRow& r = imu_rows[i];
        ins::ImuSample s;
        s.timestamp_s = r.t;
        s.gyro_rad_s = ins::Vector3(r.gx, r.gy, r.gz);
        s.accel_m_s2 = ins::Vector3(r.ax, r.ay, r.az);

        if (i == 0) {
            ekf.init(s);
        } else {
            ekf.predict(s);
        }

        bool gps_fused_this_step = false;
        // Fuse any GPS/MAG rows whose timestamp has now been reached --
        // both logs are chronological, so a simple advancing index works.
        while (gps_i < gps_rows.size() && gps_rows[gps_i].t <= r.t) {
            const GpsRow& g = gps_rows[gps_i++];
            if (!ekf.isInitialized()) continue;
            projector.setOriginIfNeeded(g.lat_deg, g.lon_deg, g.alt_m);
            ins::GpsSample gps;
            gps.timestamp_s = g.t;
            gps.position_ned_m = projector.toNed(g.lat_deg, g.lon_deg, g.alt_m);
            gps.velocity_ned_m_s = ins::Vector3(g.vn, g.ve, g.vd);
            gps_attempted++;
            if (ekf.fuseGps(gps)) {
                gps_accepted++;
                gps_fused_this_step = true;
            }
        }
        while (mag_i < mag_rows.size() && mag_rows[mag_i].t <= r.t) {
            const MagRow& m = mag_rows[mag_i++];
            if (!ekf.isInitialized()) continue;
            ins::MagSample mag;
            mag.timestamp_s = m.t;
            mag.field_body = ins::Vector3(m.mx, m.my, m.mz);
            mag_attempted++;
            if (ekf.fuseMag(mag)) mag_accepted++;
        }

        const ins::InsState st = ekf.state();
        double roll, pitch, yaw;
        st.eulerDeg(roll, pitch, yaw);
        out << r.t << ',' << st.position_ned_m.x << ',' << st.position_ned_m.y << ',' << st.position_ned_m.z << ','
            << st.velocity_ned_m_s.x << ',' << st.velocity_ned_m_s.y << ',' << st.velocity_ned_m_s.z << ',' << roll
            << ',' << pitch << ',' << yaw << ',' << (gps_fused_this_step ? 1 : 0) << '\n';
    }

    out.close();  // must be flushed/closed before re-reading it back below

    std::printf("\nGPS: %d/%d fixes accepted by the innovation gate (%d rejected).\n", gps_accepted, gps_attempted,
                gps_attempted - gps_accepted);
    std::printf("Magnetometer: %d/%d readings accepted.\n", mag_accepted, mag_attempted);
    std::printf("Wrote %s (InsEkf's own trajectory estimate over the whole log).\n", out_path.c_str());

    if (!ekf3_rows.empty()) {
        // Coarse nearest-timestamp comparison against EKF3's own estimate --
        // not a rigorous interpolation, just a sanity-check magnitude.
        double sum_pos_sq = 0, sum_att_sq = 0;
        int n = 0;
        std::ifstream replay(out_path);
        std::string header;
        std::getline(replay, header);
        std::string line;
        size_t ek = 0;
        while (std::getline(replay, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> f;
            while (std::getline(ss, cell, ',')) f.push_back(std::stod(cell));
            const double t = f[0];
            while (ek + 1 < ekf3_rows.size() && ekf3_rows[ek + 1].t <= t) ek++;
            if (std::fabs(ekf3_rows[ek].t - t) > 0.5) continue;  // no close-enough reference sample
            const Ekf3Row& e = ekf3_rows[ek];
            const double dpn = f[1] - e.pn, dpe = f[2] - e.pe, dpd = f[3] - e.pd;
            const double droll = f[7] - e.roll_deg, dpitch = f[8] - e.pitch_deg;
            sum_pos_sq += dpn * dpn + dpe * dpe + dpd * dpd;
            sum_att_sq += droll * droll + dpitch * dpitch;
            n++;
        }
        if (n > 0) {
            std::printf("\nVs. EKF3's own estimate (%d matched samples, nearest-timestamp, not interpolated):\n", n);
            std::printf("  position RMS difference : %.2f m\n", std::sqrt(sum_pos_sq / n));
            std::printf("  roll/pitch RMS difference: %.2f deg\n", std::sqrt(sum_att_sq / n));
            std::printf("  (position will differ somewhat by construction -- EKF3's origin and this replay's "
                        "projection origin aren't guaranteed identical; look at the *shape* of agreement over "
                        "time, not the absolute number.)\n");
        }
    }

    return 0;
}
