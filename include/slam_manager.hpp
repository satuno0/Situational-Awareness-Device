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
    float scan_max_range_m = 7.0f;
    float scan_doppler_gate_mps = 0.55f;
    float search_translation_m = 0.18f;
    float search_yaw_deg = 5.0f;
    float velocity_damping_per_s = 1.5f;
    std::size_t max_scan_points = 96;
};

// v1 localization: AHRS + damped IMU ego-motion with radar-to-map scan
// correction. Loop closure is deliberately a stub until a keyframe database
// exists; pretending otherwise would hide drift instead of exposing it.
class SlamManager
{
public:
    SlamManager();
    explicit SlamManager(const SlamConfig& config);

    void reset(const Pose2D& pose = Pose2D{0.0f, 0.0f, 0.0f});
    void setPose(const Pose2D& pose);

    void update();
    void update(const ImuSample& sample, float dt);
    void update(const RadarFrame& radar, const OccupancyGrid& grid);

    void correctPose();
    void correctPose(const Pose2D& measurement, float confidence);

    bool detectLoopClosure();
    bool detectLoopClosure(const std::vector<MapObject>& objects);

    Pose2D pose() const { return pose_; }
    const Quaternion& attitude() const { return attitude_.orientation(); }
    const AttitudeEstimator& ahrs() const { return attitude_; }
    float headingRad() const { return degToRad(pose_.yaw_deg); }
    float confidence() const { return confidence_; }

    Vec3 bodyToWorld(const Vec3& body) const;
    Vec3 worldToBody(const Vec3& world) const;
    Vec3 bodyVectorToWorld(const Vec3& body) const;

    // Damped IMU ego-motion estimate. Radar detections report velocity relative
    // to the sensor, so world-frame velocity needs this added back.
    const Vec3& egoVelocity() const { return velocity_; }

private:
    float scorePose(const std::vector<Vec3>& points, const Pose2D& candidate, const OccupancyGrid& grid) const;
    bool estimateEgoVelocity(const RadarFrame& radar);

    SlamConfig config_;
    Pose2D pose_{0.0f, 0.0f, 0.0f};
    AttitudeEstimator attitude_;
    Vec3 velocity_{0.0f, 0.0f, 0.0f};
    Vec3 accel_filtered_{0.0f, 0.0f, 0.0f};
    float last_ahrs_yaw_rad_ = 0.0f;
    float confidence_ = 0.35f;
    std::optional<uint64_t> last_radar_pose_ms_;
    bool initialized_ = false;
};

} // namespace sa
