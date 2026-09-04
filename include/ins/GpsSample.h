#pragma once

#include "ins/Math3.h"

namespace ins {

// One GPS fix, already resolved to a local NED tangent-plane frame (meters
// relative to a fixed local origin). Converting a real receiver's
// lat/lon/alt to this frame is a one-line call to ArduPilot's
// Location::get_vector_from_origin_NEU() when this is ported in — see
// docs/ardupilot_integration.md.
struct GpsSample {
    double timestamp_s = 0.0;
    Vector3 position_ned_m;
    Vector3 velocity_ned_m_s;

    // The *external* half of "is this GPS healthy" -- set these from the
    // receiver's own fix-quality report (e.g. ArduPilot's AP_GPS::status()
    // >= GPS_OK_FIX_3D, an HDOP/satellite-count threshold) before calling
    // InsEkf::fuseGps(), not left at the default. A fix the receiver itself
    // reports as marginal should never reach the filter at all; the
    // filter's own innovation gate (InsEkfConfig::innovation_gate_sigma) is
    // the second, independent check, for a fix that *claims* to be healthy
    // but is statistically implausible anyway (e.g. spoofing).
    bool position_valid = true;
    bool velocity_valid = true;
};

}  // namespace ins
