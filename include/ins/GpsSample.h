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
    bool position_valid = true;
    bool velocity_valid = true;
};

}  // namespace ins
