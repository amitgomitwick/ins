#pragma once

#include "ins/Math3.h"

namespace ins {

// One IMU measurement. Rates/specific-force are in the vehicle body frame
// (X forward, Y right, Z down), matching ArduPilot's AP_InertialSensor
// convention so a real driver's output can be fed in directly.
struct ImuSample {
    double timestamp_s = 0.0;
    Vector3 gyro_rad_s;    // body-frame angular rate
    Vector3 accel_m_s2;    // body-frame specific force (accelerometer output)
};

}  // namespace ins
