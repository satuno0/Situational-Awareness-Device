#pragma once

#include "imu_driver.hpp"
#include "occupancy_grid.hpp"
#include "types.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace sa {

struct SlamConfig
{
    bool enable_scan_matching = true;
    float scan_max_range_m = 30.0f;
    float search_translation_m = 0.15f;
    float search_yaw_deg = 4.0f;
    float velocity_damping_per_s = 1.5f;
    std::size_t max_scan_points = 96;
};

// Deterministic lightweight AHRS, occupancy-grid scan matching, and loop
// closure. All time-dependent inputs are supplied by the caller.
class SlamManager
{
public:
    SlamManager();
    explicit SlamManager(const SlamConfig& config);

    void reset(const Pose2D& pose = Pose2D{0.0f, 0.0f, 0.0f});
    void setPose(const Pose2D& pose);

    void update();
    void update(const ImuSample& sample, float dt);
    bool update(const LidarScan& scan, const OccupancyGrid& grid,
                float environment_confidence = 0.0f);
    void update(const TofScan& scan, const OccupancyGrid& grid,
                float environment_confidence = 0.0f);

    void correctPose();
    void correctPose(const Pose2D& measurement, float confidence);
    void correctPose(const UwbMeasurement& anchor);

    bool detectLoopClosure(const std::vector<Vec3>& points,
                           const OccupancyGrid& grid,
                           float environment_confidence);

    Pose2D pose() const { return pose_; }
    const Quaternion& attitude() const { return attitude_.orientation(); }
    const AttitudeEstimator& ahrs() const { return attitude_; }
    float headingRad() const { return degToRad(pose_.yaw_deg); }
    float confidence() const { return confidence_; }

    Vec3 bodyToWorld(const Vec3& body) const;
    Vec3 worldToBody(const Vec3& world) const;
    Vec3 bodyVectorToWorld(const Vec3& body) const;

    // Damped IMU ego-motion estimate, added to radar-relative velocities.
    const Vec3& egoVelocity() const { return velocity_; }

private:
    float scorePose(const std::vector<Vec3>& points, const Pose2D& candidate, const OccupancyGrid& grid) const;
    void matchSurfacePoints(const std::vector<Vec3>& points,
                            const OccupancyGrid& grid,
                            float environment_confidence);
    float loopClosureScore(const std::vector<Vec3>& points,
                           const Pose2D& candidate,
                           const OccupancyGrid& grid) const;

    SlamConfig config_;
    Pose2D pose_{0.0f, 0.0f, 0.0f};
    AttitudeEstimator attitude_;
    Vec3 velocity_{0.0f, 0.0f, 0.0f};
    Vec3 accel_filtered_{0.0f, 0.0f, 0.0f};
    float last_ahrs_yaw_rad_ = 0.0f;
    float confidence_ = 0.35f;
    bool initialized_ = false;
    std::vector<Vec3> burst_points_;
    Pose2D burst_reference_pose_{0.0f, 0.0f, 0.0f};
    bool burst_reference_valid_ = false;
};

} // namespace sa
