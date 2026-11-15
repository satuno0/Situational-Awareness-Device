#include "slam_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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
    confidence_ = 0.35f;
    initialized_ = true;
    burst_points_.clear();
    burst_reference_valid_ = false;
}

void SlamManager::setPose(const Pose2D& pose)
{
    pose_ = pose;
    velocity_ = {0.0f, 0.0f, 0.0f};
    burst_points_.clear();
    burst_reference_valid_ = false;
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

    if (attitude_.stationary())
    {
        velocity_ = {0.0f, 0.0f, 0.0f};
        accel_filtered_ = {0.0f, 0.0f, 0.0f};
        return;
    }

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
        if (grid.worldToCell(world.x, world.y, cx, cy))
            sum += grid.probability(cx, cy);
    }
    return sum;
}

void SlamManager::matchSurfacePoints(const std::vector<Vec3>& input_points,
                                     const OccupancyGrid& grid,
                                     float environment_confidence)
{
    if (!config_.enable_scan_matching || !(environment_confidence > 0.50f)) return;

    std::vector<Vec3> points;
    points.reserve(std::min(config_.max_scan_points, input_points.size()));
    for (const auto& point : input_points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            norm(point) < 0.15f || norm(point) > config_.scan_max_range_m ||
            std::abs(point.z) > 2.5f)
            continue;
        points.push_back(point);
    }
    if (points.size() > config_.max_scan_points)
    {
        std::vector<Vec3> sampled;
        sampled.reserve(config_.max_scan_points);
        for (std::size_t i = 0; i < config_.max_scan_points; ++i)
            sampled.push_back(points[i * points.size() / config_.max_scan_points]);
        points = std::move(sampled);
    }
    if (points.size() < 8) return;

    const Pose2D base = pose_;
    const float base_score = scorePose(points, base, grid);
    float best_rank = base_score;
    float best_score = base_score;
    Pose2D best = base;

    constexpr float kTranslationStepM = 0.05f;
    constexpr float kYawStepDeg = 1.0f;
    const int translation_steps = std::max(
        0, static_cast<int>(std::lround(clampf(config_.search_translation_m, 0.0f, 0.15f) /
                                        kTranslationStepM)));
    const int yaw_steps = std::max(
        0, static_cast<int>(std::lround(clampf(config_.search_yaw_deg, 0.0f, 4.0f) /
                                        kYawStepDeg)));

    for (int yi = -yaw_steps; yi <= yaw_steps; ++yi)
    {
        for (int xi = -translation_steps; xi <= translation_steps; ++xi)
        {
            for (int yi2 = -translation_steps; yi2 <= translation_steps; ++yi2)
            {
                Pose2D candidate = base;
                const float dx = static_cast<float>(xi) * kTranslationStepM;
                const float dy = static_cast<float>(yi2) * kTranslationStepM;
                const float dyaw = static_cast<float>(yi) * kYawStepDeg;
                candidate.x += dx;
                candidate.y += dy;
                candidate.yaw_deg = wrapDeg(base.yaw_deg + dyaw);
                const float score = scorePose(points, candidate, grid);
                const float prior_penalty =
                    0.002f * static_cast<float>(points.size()) *
                    (std::sqrt(dx * dx + dy * dy) / 0.15f + std::abs(dyaw) / 4.0f);
                const float rank = score - prior_penalty;
                if (rank > best_rank)
                {
                    best_rank = rank;
                    best_score = score;
                    best = candidate;
                }
            }
        }
    }

    if (best_rank > base_score + 1.0e-4f)
    {
        pose_ = best;
        attitude_.correctYaw(degToRad(pose_.yaw_deg), 0.25f);
        last_ahrs_yaw_rad_ = attitude_.yawRad();
        const float quality = clampf(best_score / static_cast<float>(points.size()), 0.0f, 1.0f);
        confidence_ = clampf(0.75f * confidence_ + 0.25f * quality + 0.01f, 0.0f, 0.98f);
    }
    else
    {
        confidence_ = std::max(0.20f, confidence_ * 0.995f);
    }
}

bool SlamManager::update(const LidarScan& scan,
                         const OccupancyGrid& grid,
                         float environment_confidence)
{
    bool loop_closed = false;
    if (scan.burst_active)
    {
        if (!burst_reference_valid_)
        {
            burst_reference_pose_ = pose_;
            burst_reference_valid_ = true;
        }
        for (const Vec3& point : scan.points)
        {
            if (burst_points_.size() >= kMaxLidarPointsPerScan) break;
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
                norm(point - scan.origin) > config_.scan_max_range_m)
                continue;
            const Vec3 world = poseApply(pose_, point);
            burst_points_.push_back(poseInverseApply(burst_reference_pose_, world));
        }
    }

    matchSurfacePoints(scan.points, grid, environment_confidence);

    if (scan.burst_complete)
    {
        const Pose2D end_pose = pose_;
        if (burst_reference_valid_)
        {
            pose_ = burst_reference_pose_;
            loop_closed = detectLoopClosure(burst_points_, grid, environment_confidence);
            if (loop_closed)
            {
                const Vec3 end_offset =
                    poseInverseApply(burst_reference_pose_, Vec3{end_pose.x, end_pose.y, 0.0f});
                const Vec3 corrected_end = poseApply(pose_, end_offset);
                pose_.x = corrected_end.x;
                pose_.y = corrected_end.y;
                pose_.yaw_deg = wrapDeg(pose_.yaw_deg +
                                        wrapDeg(end_pose.yaw_deg - burst_reference_pose_.yaw_deg));
                attitude_.correctYaw(degToRad(pose_.yaw_deg), 0.45f);
                last_ahrs_yaw_rad_ = attitude_.yawRad();
            }
            else
            {
                pose_ = end_pose;
            }
        }
        burst_points_.clear();
        burst_reference_valid_ = false;
    }
    return loop_closed;
}

void SlamManager::update(const TofScan& scan,
                         const OccupancyGrid& grid,
                         float environment_confidence)
{
    std::vector<Vec3> points;
    points.reserve(scan.rays.size());
    for (const SurfaceRay& ray : scan.rays)
    {
        if (!std::isfinite(ray.origin.x) || !std::isfinite(ray.origin.y) ||
            !std::isfinite(ray.origin.z) || !std::isfinite(ray.endpoint.x) ||
            !std::isfinite(ray.endpoint.y) || !std::isfinite(ray.endpoint.z))
            continue;
        const float range = norm(ray.endpoint - ray.origin);
        if (range <= 0.15f || range > config_.scan_max_range_m) continue;
        points.push_back(ray.endpoint);
    }
    matchSurfacePoints(points, grid, environment_confidence);
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

void SlamManager::correctPose(const UwbMeasurement& anchor)
{
    if (!anchor.is_fixed_anchor || !std::isfinite(anchor.range_m) || anchor.range_m <= 0.05f ||
        !std::isfinite(anchor.anchor_x_m) || !std::isfinite(anchor.anchor_y_m) ||
        !std::isfinite(anchor.confidence))
        return;

    const float dx = pose_.x - anchor.anchor_x_m;
    const float dy = pose_.y - anchor.anchor_y_m;
    const float estimated_range = std::sqrt(dx * dx + dy * dy);
    if (!(estimated_range > 0.05f)) return;

    const float weight = clampf(anchor.confidence, 0.0f, 1.0f) * 0.25f;
    const float range_error = clampf(anchor.range_m - estimated_range, -1.0f, 1.0f);
    pose_.x += (dx / estimated_range) * range_error * weight;
    pose_.y += (dy / estimated_range) * range_error * weight;
    confidence_ = clampf(confidence_ + 0.02f * weight, 0.0f, 0.99f);
}

float SlamManager::loopClosureScore(const std::vector<Vec3>& points,
                                    const Pose2D& candidate,
                                    const OccupancyGrid& grid) const
{
    if (points.empty()) return 0.0f;
    std::size_t matches = 0;
    for (const Vec3& point : points)
    {
        const Vec3 world = poseApply(candidate, point);
        int cx = 0;
        int cy = 0;
        if (grid.worldToCell(world.x, world.y, cx, cy) &&
            grid.probability(cx, cy) > 0.75f)
            ++matches;
    }
    return static_cast<float>(matches) / static_cast<float>(points.size());
}

bool SlamManager::detectLoopClosure(const std::vector<Vec3>& input_points,
                                    const OccupancyGrid& grid,
                                    float environment_confidence)
{
    if (!config_.enable_scan_matching || !(environment_confidence > 0.50f)) return false;

    std::vector<Vec3> points;
    points.reserve(input_points.size());
    for (const Vec3& point : input_points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            norm(point) < 0.15f || norm(point) > config_.scan_max_range_m ||
            std::abs(point.z) > 2.5f)
            continue;
        points.push_back(point);
    }
    constexpr std::size_t kMaxLoopPoints = 1024;
    if (points.size() > kMaxLoopPoints)
    {
        std::vector<Vec3> sampled;
        sampled.reserve(kMaxLoopPoints);
        for (std::size_t i = 0; i < kMaxLoopPoints; ++i)
            sampled.push_back(points[i * points.size() / kMaxLoopPoints]);
        points = std::move(sampled);
    }
    if (points.size() < 12) return false;

    const Pose2D prior = pose_;
    Pose2D best = prior;
    float best_score = loopClosureScore(points, prior, grid);
    float best_rank = best_score;

    const auto search = [&](const Pose2D& center,
                            float translation_radius,
                            float translation_step,
                            float yaw_radius,
                            float yaw_step) {
        const int translation_steps = static_cast<int>(std::lround(translation_radius / translation_step));
        const int yaw_steps = static_cast<int>(std::lround(yaw_radius / yaw_step));
        for (int yi = -yaw_steps; yi <= yaw_steps; ++yi)
        {
            for (int xi = -translation_steps; xi <= translation_steps; ++xi)
            {
                for (int yi2 = -translation_steps; yi2 <= translation_steps; ++yi2)
                {
                    Pose2D candidate = center;
                    const float dx = static_cast<float>(xi) * translation_step;
                    const float dy = static_cast<float>(yi2) * translation_step;
                    const float dyaw = static_cast<float>(yi) * yaw_step;
                    candidate.x += dx;
                    candidate.y += dy;
                    candidate.yaw_deg = wrapDeg(center.yaw_deg + dyaw);
                    const float score = loopClosureScore(points, candidate, grid);
                    const float distance_from_prior =
                        std::sqrt((candidate.x - prior.x) * (candidate.x - prior.x) +
                                  (candidate.y - prior.y) * (candidate.y - prior.y));
                    const float yaw_from_prior = std::abs(wrapDeg(candidate.yaw_deg - prior.yaw_deg));
                    const float rank =
                        score - 0.002f * (distance_from_prior / 1.5f + yaw_from_prior / 25.0f);
                    if (rank > best_rank)
                    {
                        best = candidate;
                        best_score = score;
                        best_rank = rank;
                    }
                }
            }
        }
    };

    search(prior, 1.5f, 0.5f, 25.0f, 5.0f);
    search(best, 0.5f, 0.1f, 5.0f, 1.0f);
    search(best, 0.1f, 0.05f, 1.0f, 1.0f);
    if (best_score < 0.35f) return false;

    pose_ = best;
    attitude_.correctYaw(degToRad(pose_.yaw_deg), 0.45f);
    last_ahrs_yaw_rad_ = attitude_.yawRad();
    confidence_ = clampf(std::max(confidence_, 0.65f + 0.30f * best_score), 0.0f, 0.99f);
    return true;
}

Vec3 SlamManager::bodyToWorld(const Vec3& body) const { return poseApply(pose_, body); }
Vec3 SlamManager::worldToBody(const Vec3& world) const { return poseInverseApply(pose_, world); }
Vec3 SlamManager::bodyVectorToWorld(const Vec3& body) const { return poseApplyVector(pose_, body); }

} // namespace sa
