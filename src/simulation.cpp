#include "simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sa {
namespace {

float rayBoxDistance(float ox, float oy, float dx, float dy, float min_x, float min_y, float max_x, float max_y)
{
    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::infinity();
    if (std::abs(dx) < 1.0e-6f)
    {
        if (ox < min_x || ox > max_x) return -1.0f;
    }
    else
    {
        float t1 = (min_x - ox) / dx;
        float t2 = (max_x - ox) / dx;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
    }
    if (std::abs(dy) < 1.0e-6f)
    {
        if (oy < min_y || oy > max_y) return -1.0f;
    }
    else
    {
        float t1 = (min_y - oy) / dy;
        float t2 = (max_y - oy) / dy;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
    }
    if (tmax < tmin || tmax < 0.0f) return -1.0f;
    return tmin;
}

} // namespace

Simulation::Simulation() : Simulation(SimulationConfig{}) {}
Simulation::Simulation(const SimulationConfig& config) : config_(config) { reset(); }

void Simulation::reset()
{
    rng_state_ = config_.seed == 0 ? 1u : config_.seed;
    tick_ = 0;
    elapsed_s_ = 0.0;
    finished_ = false;
}

bool Simulation::finished() const { return finished_; }

float Simulation::randomUniform()
{
    uint32_t x = rng_state_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state_ = x;
    return static_cast<float>(x & 0x00FFFFFFu) / 16777216.0f;
}

float Simulation::gaussian(float sigma)
{
    const float u1 = std::max(1.0e-6f, randomUniform());
    const float u2 = randomUniform();
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(kTwoPi * u2) * sigma;
}

Pose2D Simulation::poseAt(double t) const
{
    if (t <= 3.0) return {0.0f, 0.0f, 0.0f};
    const double u = std::clamp(t - 3.0, 0.0, 12.0);
    const double phase = kPi * u / 12.0;
    Pose2D pose{};
    pose.x = static_cast<float>(0.55 * 0.5 * (1.0 - std::cos(phase)));
    pose.y = static_cast<float>(0.10 * std::sin(phase));
    pose.yaw_deg = static_cast<float>(4.5 * std::sin(phase));
    return pose;
}

Pose2D Simulation::truthPose() const { return poseAt(elapsed_s_); }

Vec3 Simulation::personPosition(int index, double t) const
{
    switch (index)
    {
    case 0:
        return {3.20f + 0.30f * static_cast<float>(std::sin(0.18 * t)),
                1.00f + 0.35f * static_cast<float>(std::cos(0.20 * t)),
                0.95f};
    case 1:
        return {2.90f + 0.20f * static_cast<float>(std::sin(0.20 * t)),
                -0.52f + 0.42f * static_cast<float>(std::sin(0.31 * t)),
                0.95f};
    case 2:
        return {4.80f + 0.20f * static_cast<float>(std::cos(0.16 * t)),
                2.10f + 0.32f * static_cast<float>(std::sin(0.23 * t)),
                1.00f};
    default:
        return {3.50f, 4.40f, 1.00f};
    }
}

Vec3 Simulation::personVelocity(int index, double t) const
{
    const double eps = 0.01;
    return (personPosition(index, t + eps) - personPosition(index, t - eps)) / static_cast<float>(2.0 * eps);
}

Vec3 Simulation::markerPositionAt(double t) const
{
    // A single red marker on a wall ahead of the device. Its lateral motion
    // sweeps through the wrist aim cone, so vibration engages and releases.
    return {3.55f, 0.20f + 0.70f * static_cast<float>(std::sin(0.45 * t)), 1.18f};
}

Simulation::Person Simulation::personAt(int index, double t) const
{
    Person person;
    person.position = personPosition(index, t);
    person.velocity = personVelocity(index, t);
    if (index == 0)
    {
        person.safe_tagged = true;
        person.tag_id = 0x2A01;
    }
    return person;
}

bool Simulation::lineOfSight(const Vec3& origin, const Vec3& target, double& blocked_distance) const
{
    const Vec3 delta = target - origin;
    const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (length < 1.0e-4f) return true;
    const float dx = delta.x / length;
    const float dy = delta.y / length;
    const struct
    {
        float min_x, min_y, max_x, max_y;
    } boxes[] = {
        {2.07f, -0.58f, 2.63f, -0.02f},
        {1.05f, -0.05f, 1.45f, 0.35f},
        {0.40f, 1.95f, 1.40f, 2.65f}};
    float closest = std::numeric_limits<float>::infinity();
    for (const auto& box : boxes)
    {
        const float hit = rayBoxDistance(origin.x, origin.y, dx, dy, box.min_x, box.min_y, box.max_x, box.max_y);
        if (hit > 0.03f && hit < length - 0.05f) closest = std::min(closest, hit);
    }
    if (std::isfinite(closest))
    {
        blocked_distance = closest;
        return false;
    }
    blocked_distance = length;
    return true;
}

void Simulation::addDetection(RadarFrame& frame,
                              const Pose2D& pose,
                              const Vec3& world_point,
                              const Vec3& world_velocity,
                              float snr)
{
    const Vec3 body = poseInverseApply(pose, world_point);
    const Vec3 body_velocity = rotateZ(world_velocity, -degToRad(pose.yaw_deg));
    Detection detection{};
    detection.position = body;
    detection.velocity = body_velocity;
    detection.range = std::sqrt(body.x * body.x + body.y * body.y + body.z * body.z);
    detection.snr = snr;
    detection.from_radar = true;
    frame.detections.push_back(detection);
}

void Simulation::addStaticReturns(RadarFrame& frame, const Pose2D& pose, double t)
{
    const Pose2D p0 = poseAt(t - 0.01);
    const Pose2D p1 = poseAt(t + 0.01);
    const Vec3 device_velocity{(p1.x - p0.x) / 0.02f, (p1.y - p0.y) / 0.02f, 0.0f};
    const Vec3 origin{pose.x, pose.y, 0.0f};

    struct Segment
    {
        float x0, y0, x1, y1;
    };
    const Segment walls[] = {
        {-2.0f, -3.5f, -2.0f, 3.5f},
        {7.0f, -3.5f, 7.0f, 3.5f},
        {-2.0f, -3.5f, 7.0f, -3.5f},
        {-2.0f, 3.5f, 7.0f, 3.5f}};

    const float yaw = degToRad(pose.yaw_deg);
    const float max_range = 7.0f;

    auto emitStatic = [&](float wx, float wy, float wz) {
        const float dx = wx - pose.x;
        const float dy = wy - pose.y;
        const float distance = std::sqrt(dx * dx + dy * dy);
        if (distance < 0.2f || distance > max_range) return;
        const float bearing = wrapRad(std::atan2(dy, dx) - yaw);
        if (std::abs(bearing) > degToRad(60.0f)) return;
        double blocked = 0.0;
        if (!lineOfSight(origin, {wx, wy, wz}, blocked)) return;
        addDetection(frame, pose, {wx, wy, wz}, -device_velocity, 9.0f + 7.0f * randomUniform());
    };

    for (const auto& wall : walls)
    {
        const float length = std::sqrt((wall.x1 - wall.x0) * (wall.x1 - wall.x0) +
                                       (wall.y1 - wall.y0) * (wall.y1 - wall.y0));
        const int count = static_cast<int>(length / 0.32f);
        for (int i = 0; i <= count; ++i)
        {
            const float f = static_cast<float>(i) / static_cast<float>(std::max(1, count));
            const float wx = lerpf(wall.x0, wall.x1, f) + 0.02f * gaussian(1.0f);
            const float wy = lerpf(wall.y0, wall.y1, f) + 0.02f * gaussian(1.0f);
            emitStatic(wx, wy, 0.55f + 0.03f * gaussian(1.0f));
        }
    }

    const struct
    {
        float min_x, min_y, max_x, max_y, z;
    } objects[] = {
        {2.07f, -0.58f, 2.63f, -0.02f, 0.9f},
        {1.05f, -0.05f, 1.45f, 0.35f, 0.4f},
        {0.40f, 1.95f, 1.40f, 2.65f, 0.7f}};
    for (const auto& object : objects)
    {
        const float sx = object.max_x - object.min_x;
        const float sy = object.max_y - object.min_y;
        const int nx = std::max(1, static_cast<int>(sx / 0.22f));
        const int ny = std::max(1, static_cast<int>(sy / 0.22f));
        for (int i = 0; i <= nx; ++i)
        {
            const float x = lerpf(object.min_x, object.max_x, static_cast<float>(i) / nx);
            emitStatic(x, object.min_y, object.z);
            emitStatic(x, object.max_y, object.z);
        }
        for (int i = 0; i <= ny; ++i)
        {
            const float y = lerpf(object.min_y, object.max_y, static_cast<float>(i) / ny);
            emitStatic(object.min_x, y, object.z);
            emitStatic(object.max_x, y, object.z);
        }
    }
}

void Simulation::addPersonReturns(RadarFrame& frame, const Pose2D& pose, double t)
{
    const Pose2D p0 = poseAt(t - 0.01);
    const Pose2D p1 = poseAt(t + 0.01);
    const Vec3 device_velocity{(p1.x - p0.x) / 0.02f, (p1.y - p0.y) / 0.02f, 0.0f};
    const Vec3 origin{pose.x, pose.y, 0.0f};
    const float yaw = degToRad(pose.yaw_deg);

    for (int index = 0; index < 3; ++index)
    {
        const Person person = personAt(index, t);
        const Vec3 delta = person.position - origin;
        const float distance = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (distance < 0.3f || distance > 7.0f) continue;
        if (std::abs(wrapRad(std::atan2(delta.y, delta.x) - yaw)) > degToRad(60.0f)) continue;
        double blocked = 0.0;
        if (!lineOfSight(origin, person.position, blocked)) continue;

        const Vec3 relative_velocity = person.velocity - device_velocity;
        const int returns = 2 + (randomUniform() > 0.55f ? 1 : 0);
        for (int r = 0; r < returns; ++r)
        {
            const Vec3 point = person.position + Vec3{0.05f * gaussian(1.0f), 0.05f * gaussian(1.0f),
                                                      0.16f * gaussian(1.0f)};
            addDetection(frame, pose, point, relative_velocity, 14.0f + 5.0f * randomUniform());
        }
    }
}

float Simulation::ultrasonicForward(const Pose2D& pose, double t) const
{
    const float yaw = degToRad(pose.yaw_deg);
    const float dx = std::cos(yaw);
    const float dy = std::sin(yaw);
    float best = 4.0f;

    const float walls[][4] = {
        {-2.0f, -3.5f, -2.0f, 3.5f},
        {7.0f, -3.5f, 7.0f, 3.5f},
        {-2.0f, -3.5f, 7.0f, -3.5f},
        {-2.0f, 3.5f, 7.0f, 3.5f}};
    for (const auto& wall : walls)
    {
        const float min_x = std::min(wall[0], wall[2]) - 0.02f;
        const float max_x = std::max(wall[0], wall[2]) + 0.02f;
        const float min_y = std::min(wall[1], wall[3]) - 0.02f;
        const float max_y = std::max(wall[1], wall[3]) + 0.02f;
        const float hit = rayBoxDistance(pose.x, pose.y, dx, dy, min_x, min_y, max_x, max_y);
        if (hit > 0.05f) best = std::min(best, hit);
    }
    const struct
    {
        float min_x, min_y, max_x, max_y;
    } boxes[] = {
        {2.07f, -0.58f, 2.63f, -0.02f},
        {1.05f, -0.05f, 1.45f, 0.35f},
        {0.40f, 1.95f, 1.40f, 2.65f}};
    for (const auto& box : boxes)
    {
        const float hit = rayBoxDistance(pose.x, pose.y, dx, dy, box.min_x, box.min_y, box.max_x, box.max_y);
        if (hit > 0.05f) best = std::min(best, hit);
    }
    for (int index = 0; index < 3; ++index)
    {
        const Vec3 p = personPosition(index, t);
        const float px = p.x - pose.x;
        const float py = p.y - pose.y;
        const float projection = px * dx + py * dy;
        if (projection <= 0.0f) continue;
        const float lateral = std::abs(px * dy - py * dx);
        if (lateral < 0.18f) best = std::min(best, projection);
    }
    // Deterministic, bounded sensor jitter: this keeps ultrasonicForward const
    // and preserves replay determinism without consuming the shared RNG stream.
    const float jitter = 0.004f * std::sin(37.0f * static_cast<float>(t));
    return std::max(0.2f, best + jitter);
}

bool Simulation::next(SensorFrame& frame)
{
    if (finished_) return false;

    const double t = elapsed_s_;
    const Pose2D pose = poseAt(t);
    const uint64_t timestamp = config_.start_timestamp_ms + static_cast<uint64_t>(std::llround(t * 1000.0));
    frame = SensorFrame{};
    frame.frame_id = static_cast<uint32_t>(tick_ + 1);
    frame.timestamp_ms = timestamp;

    const double eps = 0.01;
    const Pose2D p0 = poseAt(t - eps);
    const Pose2D p1 = poseAt(t + eps);
    const Vec3 world_accel{(p1.x - 2.0f * pose.x + p0.x) / static_cast<float>(eps * eps),
                           (p1.y - 2.0f * pose.y + p0.y) / static_cast<float>(eps * eps), 0.0f};
    const float yaw_rate = degToRad(wrapDeg(static_cast<float>((p1.yaw_deg - p0.yaw_deg) / (2.0 * eps))));
    Vec3 body_accel = rotateZ(world_accel + Vec3{0.0f, 0.0f, kGravity}, -degToRad(pose.yaw_deg));
    body_accel += Vec3{0.03f * gaussian(1.0f), 0.03f * gaussian(1.0f), 0.05f * gaussian(1.0f)};

    ImuSample imu{};
    imu.timestamp_ms = timestamp;
    imu.accel_mps2 = body_accel;
    imu.gyro_radps = {0.002f * gaussian(1.0f), 0.002f * gaussian(1.0f), yaw_rate + 0.002f * gaussian(1.0f)};
    imu.temperature_c = 31.5f + 0.2f * gaussian(1.0f);
    frame.imu = imu;

    if ((tick_ % 2) == 0)
    {
        RadarFrame radar;
        radar.timestamp_ms = timestamp;
        addStaticReturns(radar, pose, t);
        addPersonReturns(radar, pose, t);
        frame.radar = std::move(radar);
    }

    if ((tick_ % 4) == 0)
    {
        const Person safe = personAt(0, t);
        if (safe.safe_tagged)
        {
            const Vec3 body = poseInverseApply(pose, safe.position);
            UwbMeasurement uwb{};
            uwb.timestamp_ms = timestamp;
            uwb.tag_id = safe.tag_id;
            uwb.range_m = std::sqrt(body.x * body.x + body.y * body.y) + 0.04f * gaussian(1.0f);
            uwb.has_aoa = true;
            uwb.aoa_rad = std::atan2(body.y, body.x) + 0.015f * gaussian(1.0f);
            const Vec3 d0 = personPosition(0, t - 0.05) - Vec3{poseAt(t - 0.05).x, poseAt(t - 0.05).y, 0.0f};
            const Vec3 d1 = personPosition(0, t + 0.05) - Vec3{poseAt(t + 0.05).x, poseAt(t + 0.05).y, 0.0f};
            const float r0 = std::sqrt(d0.x * d0.x + d0.y * d0.y);
            const float r1 = std::sqrt(d1.x * d1.x + d1.y * d1.y);
            uwb.has_range_rate = true;
            uwb.range_rate_mps = (r1 - r0) / 0.10f + 0.04f * gaussian(1.0f);
            uwb.confidence = 0.92f;
            frame.uwb.push_back(uwb);
        }

        const Vec3 b = personPosition(1, t);
        const Vec3 db = b - Vec3{pose.x, pose.y, 0.0f};
        NoveldaMeasurement presence{};
        presence.timestamp_ms = timestamp;
        presence.range_m = std::sqrt(db.x * db.x + db.y * db.y) + 0.12f * gaussian(1.0f);
        presence.bearing_center_rad = wrapRad(std::atan2(db.y, db.x) - degToRad(pose.yaw_deg));
        presence.bearing_width_rad = degToRad(22.0f);
        presence.motion = 0.55f;
        presence.breathing = 0.72f + 0.08f * static_cast<float>(std::sin(0.7 * t));
        presence.confidence = 0.58f + 0.18f * presence.breathing;
        presence.through_barrier = false;
        frame.novelda.push_back(presence);

        const Vec3 d = personPosition(3, t);
        const Vec3 dd = d - Vec3{pose.x, pose.y, 0.0f};
        NoveldaMeasurement through{};
        through.timestamp_ms = timestamp;
        through.range_m = std::sqrt(dd.x * dd.x + dd.y * dd.y) + 0.20f * gaussian(1.0f);
        through.bearing_center_rad = wrapRad(std::atan2(dd.y, dd.x) - degToRad(pose.yaw_deg));
        through.bearing_width_rad = degToRad(18.0f);
        through.motion = 0.12f;
        through.breathing = 0.66f;
        through.confidence = 0.48f;
        through.through_barrier = true;
        frame.novelda.push_back(through);
    }

    if ((tick_ % 2) == 0)
    {
        UltrasonicMeasurement ultrasonic{};
        ultrasonic.timestamp_ms = timestamp;
        ultrasonic.range_m = ultrasonicForward(pose, t);
        ultrasonic.valid = ultrasonic.range_m >= 0.15f && ultrasonic.range_m <= 5.0f;
        frame.ultrasonic = ultrasonic;
    }

    if (config_.marker_enabled && (tick_ % 2) == 0)
    {
        const Vec3 marker_world = markerPositionAt(t);
        const Vec3 body = poseInverseApply(pose, marker_world);
        const float bearing = std::atan2(body.y, body.x);
        constexpr float kCameraHalfFovRad = 0.56f; // about 32 degrees
        if (body.x > 0.25f && std::abs(bearing) <= kCameraHalfFovRad)
        {
            MarkerObservation marker{};
            marker.timestamp_ms = timestamp;
            // Small deterministic jitter keeps the demo realistic without
            // consuming the shared RNG stream used by the other sensors.
            marker.bearing_rad = bearing + 0.006f * std::sin(23.0f * static_cast<float>(t));
            const float range = std::sqrt(body.x * body.x + body.y * body.y + body.z * body.z);
            marker.area_ratio = clampf(0.020f / std::max(0.5f, range), 0.0005f, 0.25f);
            marker.confidence = clampf(
                0.90f - 0.02f * range + 0.03f * std::sin(0.7f * static_cast<float>(t)), 0.4f, 0.98f);
            marker.valid = true;
            frame.marker = marker;
        }
    }

    ++tick_;
    elapsed_s_ += config_.frame_period_s;
    if (elapsed_s_ >= config_.duration_s) finished_ = true;
    return true;
}

} // namespace sa
