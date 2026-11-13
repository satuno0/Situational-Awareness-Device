#include "slam_manager.hpp"

#include <algorithm>
#include <cmath>

namespace sa {

SlamManager::SlamManager() : SlamManager(SlamConfig{}) {}
SlamManager::SlamManager(const SlamConfig& config) : config_(config) { reset(); }

void SlamManager::reset(const Pose2D& pose)
{
    pose_ = pose;
    attitude_.reset();
    velocity_ = {0.0f, 0.0f, 0.0f};
    accel_filtered_ = {0.0f, 0.0f, 0.0f};
    last_ahrs_yaw_rad_ = attitude_.yawRad();
    last_radar_pose_ms_.reset();
    confidence_ = 0.35f;
    initialized_ = true;
}

void SlamManager::setPose(const Pose2D& pose)
{
    pose_ = pose;
    velocity_ = {0.0f, 0.0f, 0.0f};
}

void SlamManager::update() {}
void SlamManager::correctPose() {}

void SlamManager::update(const ImuSample& sample, float dt)
{
    if (!(dt > 0.0f)) return;
    dt = std::min(dt, 0.2f);

    attitude_.update(sample, dt);
    const float yaw = attitude_.yawRad();
    if (!initialized_)
    {
        last_ahrs_yaw_rad_ = yaw;
        initialized_ = true;
    }
    const float yaw_delta = wrapRad(yaw - last_ahrs_yaw_rad_);
    last_ahrs_yaw_rad_ = yaw;
    pose_.yaw_deg = wrapDeg(pose_.yaw_deg + radToDeg(yaw_delta));

    const bool radar_fresh = last_radar_pose_ms_.has_value() && sample.timestamp_ms >= *last_radar_pose_ms_ &&
                             sample.timestamp_ms - *last_radar_pose_ms_ <= 400;

    if (attitude_.stationary())
    {
        if (!radar_fresh)
        {
            velocity_ = {0.0f, 0.0f, 0.0f};
            accel_filtered_ = {0.0f, 0.0f, 0.0f};
        }
        return;
    }

    // When radar clutter gives an ego-velocity estimate, translation is
    // integrated once in update(RadarFrame). Integrating here as well would
    // double-count the same motion.
    if (radar_fresh) return;

    Vec3 world_accel = quatRotate(attitude_.orientation(), sample.accel_mps2);
    world_accel.z -= kGravity;
    world_accel.z = 0.0f;
    if (norm(world_accel) > 6.0f) world_accel = normalized(world_accel) * 6.0f;
    accel_filtered_ = accel_filtered_ * 0.80f + world_accel * 0.20f;

    velocity_ += accel_filtered_ * dt;
    velocity_ *= std::exp(-config_.velocity_damping_per_s * dt);
    const float speed = norm(velocity_);
    if (speed > 2.0f) velocity_ = velocity_ * (2.0f / speed);
    pose_.x += velocity_.x * dt;
    pose_.y += velocity_.y * dt;

    // Translation from a consumer IMU is weak evidence. The scan matcher below
    // is the absolute correction; keep confidence conservative while moving.
    confidence_ = std::max(0.22f, confidence_ - 0.015f * dt);
}

float SlamManager::scorePose(const std::vector<Vec3>& points,
                             const Pose2D& candidate,
                             const OccupancyGrid& grid) const
{
    if (points.empty()) return 0.0f;
    float sum = 0.0f;
    for (const auto& point : points)
    {
        const Vec3 world = poseApply(candidate, point);
        int cx = 0;
        int cy = 0;
        if (!grid.worldToCell(world.x, world.y, cx, cy)) continue;
        const float p = grid.probability(cx, cy);
        sum += (p - 0.5f);
    }
    return sum / static_cast<float>(points.size());
}

bool SlamManager::estimateEgoVelocity(const RadarFrame& radar)
{
    struct Sample
    {
        float ux;
        float uy;
        float vr;
    };

    std::vector<Sample> samples;
    samples.reserve(radar.detections.size());
    for (const auto& detection : radar.detections)
    {
        const float range = norm(detection.position);
        if (range < 0.35f || range > config_.scan_max_range_m) continue;
        if (std::abs(detection.position.z) > 2.5f) continue;
        const Vec3 unit = detection.position * (1.0f / range);
        if (std::sqrt(unit.x * unit.x + unit.y * unit.y) < 0.25f) continue;
        samples.push_back(Sample{unit.x, unit.y, dot(detection.velocity, unit)});
    }
    if (samples.size() < 8) return false;

    auto solve_pair = [](const Sample& a, const Sample& b, float& vx, float& vy) {
        const float det = a.ux * b.uy - a.uy * b.ux;
        if (std::abs(det) < 0.15f) return false;
        vx = (-a.vr * b.uy + b.vr * a.uy) / det;
        vy = (-b.vr * a.ux + a.vr * b.ux) / det;
        return std::isfinite(vx) && std::isfinite(vy);
    };

    auto fit = [](const std::vector<Sample>& input, float& vx, float& vy) {
        double sxx = 0.0;
        double sxy = 0.0;
        double syy = 0.0;
        double bx = 0.0;
        double by = 0.0;
        for (const auto& s : input)
        {
            sxx += static_cast<double>(s.ux) * s.ux;
            sxy += static_cast<double>(s.ux) * s.uy;
            syy += static_cast<double>(s.uy) * s.uy;
            bx += static_cast<double>(s.ux) * (-s.vr);
            by += static_cast<double>(s.uy) * (-s.vr);
        }
        sxx += 1.0e-3;
        syy += 1.0e-3;
        const double det = sxx * syy - sxy * sxy;
        if (std::abs(det) < 1.0e-6) return false;
        vx = static_cast<float>((bx * syy - by * sxy) / det);
        vy = static_cast<float>((by * sxx - bx * sxy) / det);
        return std::isfinite(vx) && std::isfinite(vy);
    };

    // Deterministic RANSAC over pairs of bearings. Static clutter is the
    // majority, so the ego model with the most low-residual detections wins;
    // moving people remain outliers instead of dragging the estimate.
    float best_vx = 0.0f;
    float best_vy = 0.0f;
    std::size_t best_inliers = 0;
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        for (std::size_t j = i + 1; j < samples.size(); ++j)
        {
            float vx = 0.0f;
            float vy = 0.0f;
            if (!solve_pair(samples[i], samples[j], vx, vy)) continue;
            if (vx * vx + vy * vy > 4.0f) continue;
            std::size_t inliers = 0;
            for (const auto& s : samples)
            {
                if (std::abs(s.ux * vx + s.uy * vy + s.vr) <= 0.06f) ++inliers;
            }
            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_vx = vx;
                best_vy = vy;
            }
        }
    }

    const std::size_t required = std::max<std::size_t>(8, (samples.size() * 3) / 5);
    if (best_inliers < required) return false;

    std::vector<Sample> inliers;
    inliers.reserve(best_inliers);
    for (const auto& s : samples)
    {
        if (std::abs(s.ux * best_vx + s.uy * best_vy + s.vr) <= 0.06f) inliers.push_back(s);
    }

    float vx = best_vx;
    float vy = best_vy;
    if (inliers.size() >= 8) fit(inliers, vx, vy);

    Vec3 body{vx, vy, 0.0f};
    const float speed = norm(body);
    if (speed > 2.0f) body = body * (2.0f / speed);
    velocity_ = bodyVectorToWorld(body);
    return true;
}

void SlamManager::update(const RadarFrame& radar, const OccupancyGrid& grid)
{
    if (estimateEgoVelocity(radar))
    {
        float dt = 0.0f;
        if (last_radar_pose_ms_ && radar.timestamp_ms >= *last_radar_pose_ms_)
        {
            dt = static_cast<float>(radar.timestamp_ms - *last_radar_pose_ms_) / 1000.0f;
            dt = clampf(dt, 0.0f, 0.5f);
        }
        pose_.x += velocity_.x * dt;
        pose_.y += velocity_.y * dt;
        last_radar_pose_ms_ = radar.timestamp_ms;
    }

    if (!config_.enable_scan_matching) return;
    // A stationary device has no translation to correct, and a self-built map
    // would only let the matcher drift against itself.
    if (norm(velocity_) < 0.04f) return;
    if (grid.occupiedCellCount(0.62f) < 80) return;

    std::vector<Vec3> points;
    points.reserve(std::min(config_.max_scan_points, radar.detections.size()));
    for (const auto& detection : radar.detections)
    {
        if (points.size() >= config_.max_scan_points) break;
        if (std::abs(detection.velocity.z) > config_.scan_doppler_gate_mps) continue;
        const float speed = norm(detection.velocity);
        if (speed > config_.scan_doppler_gate_mps) continue;
        if (detection.range < 0.35f || detection.range > config_.scan_max_range_m) continue;
        if (std::abs(detection.position.z) > 2.5f) continue;
        points.push_back(detection.position);
    }
    if (points.size() < 8) return;

    const Pose2D base = pose_;
    const float base_score = scorePose(points, base, grid);
    float best_score = base_score;
    Pose2D best = base;

    const int yaw_steps = 5;
    const int xy_steps = 3;
    const float yaw_step_deg = config_.search_yaw_deg / static_cast<float>(yaw_steps);
    const float xy_step = config_.search_translation_m / static_cast<float>(xy_steps);

    for (int yi = -yaw_steps; yi <= yaw_steps; ++yi)
    {
        for (int xi = -xy_steps; xi <= xy_steps; ++xi)
        {
            for (int yi2 = -xy_steps; yi2 <= xy_steps; ++yi2)
            {
                Pose2D candidate = base;
                candidate.x += static_cast<float>(xi) * xy_step;
                candidate.y += static_cast<float>(yi2) * xy_step;
                candidate.yaw_deg = wrapDeg(base.yaw_deg + static_cast<float>(yi) * yaw_step_deg);
                float score = scorePose(points, candidate, grid);
                // Preserve the short-term prior; do not snap to unrelated map
                // structure when several poses fit equally well.
                score -= 0.012f * static_cast<float>(xi * xi + yi2 * yi2) / 9.0f;
                if (score > best_score)
                {
                    best_score = score;
                    best = candidate;
                }
            }
        }
    }

    const float improvement = best_score - base_score;
    if (improvement > 0.012f && best_score > 0.06f)
    {
        pose_.x = 0.75f * best.x + 0.25f * base.x;
        pose_.y = 0.75f * best.y + 0.25f * base.y;
        pose_.yaw_deg = wrapDeg(base.yaw_deg + 0.75f * wrapDeg(best.yaw_deg - base.yaw_deg));
        attitude_.correctYaw(degToRad(pose_.yaw_deg), 0.20f);
        last_ahrs_yaw_rad_ = attitude_.yawRad();
        const float quality = clampf(best_score * 4.5f, 0.0f, 1.0f);
        confidence_ = clampf(0.78f * confidence_ + 0.22f * quality + 0.02f, 0.0f, 0.98f);
    }
    else
    {
        confidence_ = std::max(0.20f, confidence_ * 0.995f);
    }
}

void SlamManager::correctPose(const Pose2D& measurement, float confidence)
{
    const float w = clampf(confidence, 0.0f, 0.95f);
    pose_.x = lerpf(pose_.x, measurement.x, w);
    pose_.y = lerpf(pose_.y, measurement.y, w);
    pose_.yaw_deg = wrapDeg(pose_.yaw_deg + w * wrapDeg(measurement.yaw_deg - pose_.yaw_deg));
    attitude_.correctYaw(degToRad(pose_.yaw_deg), w * 0.5f);
    confidence_ = clampf(confidence_ + (1.0f - confidence_) * w * 0.35f, 0.0f, 0.99f);
}

bool SlamManager::detectLoopClosure() { return false; }
bool SlamManager::detectLoopClosure(const std::vector<MapObject>& objects)
{
    (void)objects;
    return false;
}

Vec3 SlamManager::bodyToWorld(const Vec3& body) const { return poseApply(pose_, body); }
Vec3 SlamManager::worldToBody(const Vec3& world) const { return poseInverseApply(pose_, world); }
Vec3 SlamManager::bodyVectorToWorld(const Vec3& body) const { return poseApplyVector(pose_, body); }

} // namespace sa
