#pragma once

#include "occupancy_grid.hpp"
#include "schematic.hpp"
#include "slam_manager.hpp"
#include "track_manager.hpp"
#include "types.hpp"

#include <deque>
#include <optional>
#include <vector>

namespace sa {

struct FusionConfig
{
    TrackManagerConfig track;
    SlamConfig slam;
    GridConfig grid;

    float occupancy_decay = 0.999f;
    float max_mapping_range_m = 7.0f;
    float map_people_exclusion_m = 0.70f;
    float map_refresh_interval_s = 1.0f;
    float presence_timeout_s = 1.5f;
    float tag_range_sigma_m = 0.35f;
    float tag_bearing_sigma_rad = 0.20f;
    float tag_velocity_sigma_mps = 0.60f;

    // Only compact radar clusters at torso height are offered to the track
    // manager. LiDAR owns surface mapping; radar rays are a fallback only when
    // LiDAR health is false.
    float spawn_cluster_radius_m = 0.36f;
    std::size_t spawn_max_cluster_points = 6;
    std::size_t spawn_min_cluster_points = 2;
    float spawn_min_z_m = 0.78f;
    float spawn_max_z_m = 1.70f;
    // A person torso cluster is compact in plan view but spread in height.
    // Flat, wide returns (walls, tables, boxes) are static structure.
    float spawn_max_xy_extent_m = 0.55f;
    float spawn_min_z_span_m = 0.015f;
    float spawn_structure_clearance_m = 0.40f;
};

struct ConfidenceBreakdown
{
    float existence = 0.0f;
    float position = 0.0f;
    float identity = 0.0f;
    float environment = 0.0f;
};

class ConfidenceEngine
{
public:
    ConfidenceBreakdown evaluate(const WorldModel& world,
                                 const SlamManager& slam,
                                 const OccupancyGrid& grid) const;
    void apply(WorldModel& world, const SlamManager& slam, const OccupancyGrid& grid) const;
};

class FusionEngine
{
public:
    FusionEngine();
    explicit FusionEngine(const FusionConfig& config);

    void reset();
    WorldModel process(const SensorFrame& frame);

    // Operator-supplied structural prior. It is re-asserted every tick so the
    // mandatory occupancy decay cannot erase walls. It never creates tracks.
    void setSchematic(const Schematic& schematic) { schematic_ = schematic; }
    const Schematic& schematic() const { return schematic_; }

    const TrackManager& trackManager() const { return tracks_; }
    const OccupancyGrid& occupancy() const { return occupancy_; }
    const SlamManager& slam() const { return slam_; }
    const SensorHealth& health() const { return health_; }

private:
    struct TimedZone
    {
        PresenceZone zone;
        uint64_t seen_ms = 0;
    };

    void updateHealth(uint64_t now_ms);
    std::vector<Detection> radarToWorld(const RadarFrame& radar) const;
    std::vector<Detection> selectPersonCandidates(const std::vector<Detection>& world_detections) const;
    void updateUwb(const std::vector<UwbMeasurement>& measurements, uint64_t now_ms);
    void updatePresence(const std::vector<NoveldaMeasurement>& measurements,
                        const std::vector<PresenceZone>& wallpad_zones,
                        uint64_t now_ms);
    void updateMap(const LidarScan* lidar_scan,
                   const TofScan* tof_scan,
                   const std::vector<Detection>& radar_detections,
                   uint64_t now_ms);
    void updateStaticObjects(const std::vector<Track>& tracks);

    FusionConfig config_;
    TrackManager tracks_;
    SlamManager slam_;
    OccupancyGrid occupancy_;
    ConfidenceEngine confidence_;
    SensorHealth health_{false, false, false, false, false, false, 0.0f};
    Schematic schematic_;

    std::optional<uint64_t> last_imu_ms_;
    std::optional<uint64_t> last_radar_ms_;
    std::optional<uint64_t> last_lidar_ms_;
    std::optional<uint64_t> last_tof_ms_;
    std::optional<uint64_t> last_uwb_ms_;
    std::optional<uint64_t> last_novelda_ms_;
    std::optional<uint64_t> last_ultrasonic_ms_;
    std::optional<uint64_t> last_process_ms_;
    std::optional<uint64_t> last_map_refresh_ms_;

    std::vector<MapObject> static_objects_;
    std::deque<TimedZone> active_zones_;
};

} // namespace sa
