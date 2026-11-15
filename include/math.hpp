#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace sa {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kGravity = 9.80665f;

struct Vec3
{
    float x;
    float y;
    float z;
};

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

struct Pose2D
{
    float x;
    float y;
    float yaw_deg;
};

// Device and world coordinates are right-handed: +X right, +Y forward, +Z up.
// Positive yaw rotates the forward axis toward the left.
inline Vec3 makeVec3(float x = 0.0f, float y = 0.0f, float z = 0.0f)
{
    return Vec3{x, y, z};
}

inline Quaternion makeQuaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f)
{
    return Quaternion{w, x, y, z};
}

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(const Vec3& a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, const Vec3& a) { return a * s; }
inline Vec3 operator/(const Vec3& a, float s) { return {a.x / s, a.y / s, a.z / s}; }
inline Vec3& operator+=(Vec3& a, const Vec3& b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline Vec3& operator-=(Vec3& a, const Vec3& b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
inline Vec3& operator*=(Vec3& a, float s) { a.x *= s; a.y *= s; a.z *= s; return a; }

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float normSquared(const Vec3& v) { return dot(v, v); }
inline float norm(const Vec3& v) { return std::sqrt(normSquared(v)); }
inline Vec3 normalized(const Vec3& v)
{
    const float n = norm(v);
    return n > 1.0e-8f ? v / n : Vec3{0.0f, 0.0f, 0.0f};
}

inline float clampf(float value, float lo, float hi) { return std::max(lo, std::min(hi, value)); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * clampf(t, 0.0f, 1.0f); }

inline float wrapRad(float angle)
{
    while (angle > kPi) angle -= kTwoPi;
    while (angle < -kPi) angle += kTwoPi;
    return angle;
}

inline float wrapDeg(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

inline float degToRad(float degrees) { return degrees * kDegToRad; }
inline float radToDeg(float radians) { return radians * kRadToDeg; }

inline Vec3 rotateZ(const Vec3& v, float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);
    return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

inline Vec3 poseApply(const Pose2D& pose, const Vec3& local)
{
    const Vec3 rotated = rotateZ(local, degToRad(pose.yaw_deg));
    return {rotated.x + pose.x, rotated.y + pose.y, rotated.z};
}

inline Vec3 poseApplyVector(const Pose2D& pose, const Vec3& local)
{
    return rotateZ(local, degToRad(pose.yaw_deg));
}

inline Vec3 poseInverseApply(const Pose2D& pose, const Vec3& world)
{
    const Vec3 delta{world.x - pose.x, world.y - pose.y, world.z};
    return rotateZ(delta, -degToRad(pose.yaw_deg));
}

inline Quaternion quatMultiply(const Quaternion& a, const Quaternion& b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quaternion quatConjugate(const Quaternion& q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Quaternion quatNormalize(const Quaternion& q)
{
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1.0e-8f) return {1.0f, 0.0f, 0.0f, 0.0f};
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Vec3 quatRotate(const Quaternion& q_in, const Vec3& v)
{
    const Quaternion q = quatNormalize(q_in);
    const Quaternion p{0.0f, v.x, v.y, v.z};
    const Quaternion r = quatMultiply(quatMultiply(q, p), quatConjugate(q));
    return {r.x, r.y, r.z};
}

inline Quaternion quatFromEulerDeg(float roll_deg, float pitch_deg, float yaw_deg)
{
    const float cr = std::cos(degToRad(roll_deg) * 0.5f);
    const float sr = std::sin(degToRad(roll_deg) * 0.5f);
    const float cp = std::cos(degToRad(pitch_deg) * 0.5f);
    const float sp = std::sin(degToRad(pitch_deg) * 0.5f);
    const float cy = std::cos(degToRad(yaw_deg) * 0.5f);
    const float sy = std::sin(degToRad(yaw_deg) * 0.5f);
    return quatNormalize({
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy});
}

inline Vec3 quatToEulerDeg(const Quaternion& q_in)
{
    const Quaternion q = quatNormalize(q_in);
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float roll = std::atan2(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    const float pitch = std::abs(sinp) >= 1.0f ? std::copysign(kPi * 0.5f, sinp) : std::asin(sinp);

    const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    const float yaw = std::atan2(siny_cosp, cosy_cosp);
    return {radToDeg(roll), radToDeg(pitch), radToDeg(yaw)};
}

inline Quaternion quatIntegrateBodyRate(const Quaternion& q_in, const Vec3& omega_radps, float dt)
{
    if (dt <= 0.0f) return quatNormalize(q_in);
    const Quaternion q = quatNormalize(q_in);
    const Quaternion omega{0.0f, omega_radps.x, omega_radps.y, omega_radps.z};
    const Quaternion dq = quatMultiply(q, omega);
    Quaternion out{
        q.w + 0.5f * dq.w * dt,
        q.x + 0.5f * dq.x * dt,
        q.y + 0.5f * dq.y * dt,
        q.z + 0.5f * dq.z * dt};
    return quatNormalize(out);
}

inline float pointSegmentDistance2D(float px, float py, float ax, float ay, float bx, float by)
{
    const float abx = bx - ax;
    const float aby = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float denom = abx * abx + aby * aby;
    const float t = denom > 1.0e-8f ? clampf((apx * abx + apy * aby) / denom, 0.0f, 1.0f) : 0.0f;
    const float dx = px - (ax + t * abx);
    const float dy = py - (ay + t * aby);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace sa
