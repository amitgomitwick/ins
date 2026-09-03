#pragma once

#include "ins/Math3.h"

namespace ins {

// One magnetometer measurement, body frame, arbitrary consistent units
// (only direction is used -- see InsEkf::fuseMag -- so raw sensor counts
// are fine as long as hard-iron/soft-iron offsets have been calibrated
// out upstream, same as any other compass consumer).
struct MagSample {
    double timestamp_s = 0.0;
    Vector3 field_body;
};

}  // namespace ins
