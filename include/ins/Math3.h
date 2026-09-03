// Minimal, dependency-free 3D math types for embedded strapdown INS use.
//
// Deliberately mirrors the API shape of ArduPilot's AP_Math Vector3<T> /
// Matrix3<T> / Quaternion so that porting InsEkf into an ArduPilot library
// later is close to a find-and-replace (see docs/ardupilot_integration.md).
#pragma once

#include <cmath>
#include <cstdint>

namespace ins {

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3 operator-() const { return {-x, -y, -z}; }
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    double dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vector3 cross(const Vector3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double lengthSquared() const { return dot(*this); }
    double length() const { return std::sqrt(lengthSquared()); }
    Vector3 normalized() const {
        const double n = length();
        return n > 1e-12 ? (*this) * (1.0 / n) : Vector3{0, 0, 0};
    }
};

inline Vector3 operator*(double s, const Vector3& v) { return v * s; }

// Row-major 3x3 matrix.
struct Matrix3 {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // identity by default

    static Matrix3 zero() {
        Matrix3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = 0.0;
        return r;
    }

    Vector3 operator*(const Vector3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Matrix3 operator*(const Matrix3& o) const {
        Matrix3 r = zero();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++) r.m[i][j] += m[i][k] * o.m[k][j];
        return r;
    }

    Matrix3 transposed() const {
        Matrix3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[j][i];
        return r;
    }

    Matrix3 operator+(const Matrix3& o) const {
        Matrix3 r = zero();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[i][j] + o.m[i][j];
        return r;
    }

    Matrix3 operator*(double s) const {
        Matrix3 r = zero();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[i][j] * s;
        return r;
    }
};

// Cross-product ("skew-symmetric") matrix such that skew(a) * b == a.cross(b).
inline Matrix3 skewSymmetric(const Vector3& a) {
    Matrix3 r = Matrix3::zero();
    r.m[0][1] = -a.z; r.m[0][2] = a.y;
    r.m[1][0] = a.z;  r.m[1][2] = -a.x;
    r.m[2][0] = -a.y; r.m[2][1] = a.x;
    return r;
}

// Hamilton quaternion, scalar-first, representing a body->NED rotation
// (i.e. Vector3_ned = q.rotate(Vector3_body)).
struct Quaternion {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    Quaternion() = default;
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    void normalize() {
        const double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n > 1e-12) {
            w /= n; x /= n; y /= n; z /= n;
        } else {
            w = 1.0; x = y = z = 0.0;
        }
    }

    Quaternion operator*(const Quaternion& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    // Rotation vector (axis * angle, radians) -> unit quaternion. Exact for
    // any magnitude (not just small-angle), used both for injecting EKF
    // attitude-error corrections and for integrating gyro delta-angles.
    static Quaternion fromRotationVector(const Vector3& rv) {
        const double angle = rv.length();
        if (angle < 1e-8) {
            // Small-angle approximation avoids division by ~0.
            return Quaternion(1.0, 0.5 * rv.x, 0.5 * rv.y, 0.5 * rv.z);
        }
        const double half = 0.5 * angle;
        const double s = std::sin(half) / angle;
        return Quaternion(std::cos(half), rv.x * s, rv.y * s, rv.z * s);
    }

    Matrix3 toMatrix() const {
        Matrix3 r;
        const double ww = w * w, xx = x * x, yy = y * y, zz = z * z;
        r.m[0][0] = ww + xx - yy - zz;
        r.m[0][1] = 2 * (x * y - w * z);
        r.m[0][2] = 2 * (x * z + w * y);
        r.m[1][0] = 2 * (x * y + w * z);
        r.m[1][1] = ww - xx + yy - zz;
        r.m[1][2] = 2 * (y * z - w * x);
        r.m[2][0] = 2 * (x * z - w * y);
        r.m[2][1] = 2 * (y * z + w * x);
        r.m[2][2] = ww - xx - yy + zz;
        return r;
    }

    Vector3 rotate(const Vector3& v) const { return toMatrix() * v; }

    // roll, pitch, yaw in radians (ZYX / aerospace convention), body->NED.
    void toEulerRad(double& roll, double& pitch, double& yaw) const {
        const Matrix3 R = toMatrix();
        pitch = std::asin(std::max(-1.0, std::min(1.0, -R.m[2][0])));
        if (std::fabs(R.m[2][0]) < 0.999999) {
            roll = std::atan2(R.m[2][1], R.m[2][2]);
            yaw = std::atan2(R.m[1][0], R.m[0][0]);
        } else {
            // Gimbal lock fallback.
            roll = std::atan2(-R.m[1][2], R.m[1][1]);
            yaw = 0.0;
        }
    }

    static Quaternion fromEulerRad(double roll, double pitch, double yaw) {
        const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
        const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        q.normalize();
        return q;
    }
};

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

}  // namespace ins
