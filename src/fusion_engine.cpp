#include "fusion_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sa {
namespace {

constexpr uint64_t kImuTimeoutMs = 350;
constexpr uint64_t kRadarTimeoutMs = 650;
constexpr uint64_t kLidarTimeoutMs = 650;
constexpr uint64_t kTofTimeoutMs = 650;
constexpr uint64_t kUwbTimeoutMs = 2200;
constexpr uint64_t kNoveldaTimeoutMs = 2200;
constexpr uint64_t kUltrasonicTimeoutMs = 750;

bool fresh(const std::optional<uint64_t>& last, uint64_t now_ms, uint64_t timeout_ms)
{
    return last.has_value() && now_ms >= *last && now_ms - *last <= timeout_ms;
}

} // namespace

FusionEngine::FusionEngine() : FusionEngine(FusionConfig{}) {}
FusionEngine::FusionEngine(const FusionConfig& config)
    : config_(config), tracks_(config.track), slam_(config.slam), occupancy_(config.grid)
{
    reset();
}

void FusionEngine::reset()
{
    tracks_.reset();
    slam_.reset();
    occupancy_.reset();
    static_objects_.clear();
    active_zones_.clear();
    last_imu_ms_.reset();
    last_radar_ms_.reset();
    last_lidar_ms_.reset();
    last_tof_ms_.reset();
    last_uwb_ms_.reset();
    last_novelda_ms_.reset();
    last_ultrasonic_ms_.reset();
    last_process_ms_.reset();
    last_map_refresh_ms_.reset();
    health_ = SensorHealth{false, false, false, false, false, false, false, 0.0f};
}

void FusionEngine::updateHealth(uint64_t now_ms)
{
    health_.imu = fresh(last_imu_ms_, now_ms, kImuTimeoutMs);
    health_.radar = fresh(last_radar_ms_, now_ms, kRadarTimeoutMs);
    health_.lidar = fresh(last_lidar_ms_, now_ms, kLidarTimeoutMs);
    health_.tof = fresh(last_tof_ms_, now_ms, kTofTimeoutMs);
    health_.uwb = fresh(last_uwb_ms_, now_ms, kUwbTimeoutMs);
    health_.novelda = fresh(last_novelda_ms_, now_ms, kNoveldaTimeoutMs);
    health_.ultrasonic = fresh(last_ultrasonic_ms_, now_ms, kUltrasonicTimeoutMs);
    health_.health_score =
        0.17f * (health_.imu ? 1.0f : 0.0f) +
        0.28f * (health_.radar ? 1.0f : 0.0f) +
        0.18f * (health_.lidar ? 1.0f : 0.0f) +
        0.12f * (health_.tof ? 1.0f : 0.0f) +
        0.09f * (health_.uwb ? 1.0f : 0.0f) +
        0.09f * (health_.novelda ? 1.0f : 0.0f) +
        0.07f * (health_.ultrasonic ? 1.0f : 0.0f);
}

std::vector<Detection> FusionEngine::radarToWorld(const RadarFrame& radar) const
{
    std::vector<Detection> out;
    out.reserve(radar.detections.size());
    for (const auto& local : radar.detections)
    {
        Detection world = local;
        world.position = slam_.bodyToWorld(local.position);
        world.velocity = slam_.bodyVectorToWorld(local.velocity) + slam_.egoVelocity();
        world.from_radar = true;
        out.push_back(world);
    }
    return out;
}

std::vector<Detection> FusionEngine::selectPersonCandidates(
    const std::vector<Detection>& world_detections) const
{
    std::vector<Detection> candidates;
    if (world_detections.empty()) return candidates;

    struct Cluster
    {
        std::vector<std::size_t> indices;
        bool structure = false;
    };

    const float radius2 = config_.spawn_cluster_radius_m * config_.spawn_cluster_radius_m;
    std::vector<uint8_t> assigned(world_detections.size(), 0);
    std::vector<Cluster> clusters;
    std::vector<std::size_t> stack;

    for (std::size_t seed = 0; seed < world_detections.size(); ++seed)
    {
        if (assigned[seed]) continue;
        Cluster cluster;
        stack.clear();
        stack.push_back(seed);
        assigned[seed] = 1;

        while (!stack.empty())
        {
            const std::size_t current = stack.back();
            stack.pop_back();
            cluster.indices.push_back(current);
            const Vec3& a = world_detections[current].position;
            for (std::size_t other = 0; other < world_detections.size(); ++other)
            {
                if (assigned[other]) continue;
                const Vec3& b = world_detections[other].position;
                const float dx = a.x - b.x;
                const float dy = a.y - b.y;
                const float dz = a.z - b.z;
                if (dx * dx + dy * dy + dz * dz > radius2) continue;
                assigned[other] = 1;
                stack.push_back(other);
            }
        }
        clusters.push_back(std::move(cluster));
    }

    // Long chains are walls and furniture outlines. Remember them so short
    // fragments next to structure are not mistaken for a torso.
    for (auto& cluster : clusters)
        cluster.structure = cluster.indices.size() > std::max<std::size_t>(1, config_.spawn_max_cluster_points);

    const std::size_t min_points = std::max<std::size_t>(1, config_.spawn_min_cluster_points);
    const float clearance = config_.spawn_structure_clearance_m;

    for (const auto& cluster : clusters)
    {
        if (cluster.structure) continue;
        if (cluster.indices.size() < min_points) continue;
        if (cluster.indices.size() > config_.spawn_max_cluster_points) continue;

        Detection centroid{};
        float snr = 0.0f;
        float min_z = std::numeric_limits<float>::infinity();
        float max_z = -std::numeric_limits<float>::infinity();
        float max_xy_extent2 = 0.0f;
        for (std::size_t index : cluster.indices)
        {
            centroid.position += world_detections[index].position;
            centroid.velocity += world_detections[index].velocity;
            snr = std::max(snr, world_detections[index].snr);
            min_z = std::min(min_z, world_detections[index].position.z);
            max_z = std::max(max_z, world_detections[index].position.z);
        }
        const float inv = 1.0f / static_cast<float>(cluster.indices.size());
        centroid.position *= inv;
        centroid.velocity *= inv;
        centroid.snr = snr;
        centroid.range = std::sqrt(centroid.position.x * centroid.position.x +
                                   centroid.position.y * centroid.position.y +
                                   centroid.position.z * centroid.position.z);
        centroid.from_radar = true;

        if (centroid.position.z < config_.spawn_min_z_m || centroid.position.z > config_.spawn_max_z_m)
            continue;
        if (max_z - min_z < config_.spawn_min_z_span_m) continue; // flat, static surface

        for (std::size_t i = 0; i < cluster.indices.size(); ++i)
        {
            for (std::size_t j = i + 1; j < cluster.indices.size(); ++j)
            {
                const Vec3 d = world_detections[cluster.indices[i]].position -
                               world_detections[cluster.indices[j]].position;
                max_xy_extent2 = std::max(max_xy_extent2, d.x * d.x + d.y * d.y);
            }
        }
        if (max_xy_extent2 > config_.spawn_max_xy_extent_m * config_.spawn_max_xy_extent_m) continue;

        bool near_structure = false;
        for (const auto& other : clusters)
        {
            if (!other.structure) continue;
            for (std::size_t index : cluster.indices)
            {
                const Vec3& p = world_detections[index].position;
                for (std::size_t s : other.indices)
                {
                    if (norm(p - world_detections[s].position) < clearance)
                    {
                        near_structure = true;
                        break;
                    }
                }
                if (near_structure) break;
            }
            if (near_structure) break;
        }
        if (near_structure) continue;

        candidates.push_back(centroid);
    }

    return candidates;
}

void FusionEngine::updateUwb(const std::vector<UwbMeasurement>& measurements, uint64_t now_ms)
{
    if (measurements.empty() || tracks_.tracks().empty()) return;

    struct Candidate
    {
        float score;
        std::size_t track_index;
        std::size_t measurement_index;
    };
    std::vector<Candidate> candidates;

    const auto& all_tracks = tracks_.tracks();
    for (std::size_t ti = 0; ti < all_tracks.size(); ++ti)
    {
        const Track& track = all_tracks[ti];
        const Vec3 body = slam_.worldToBody(track.filter.position());
        const float range_pred = std::sqrt(body.x * body.x + body.y * body.y);
        if (range_pred < 0.15f) continue;
        const float bearing_pred = std::atan2(body.x, body.y);
        const Vec3 world_delta = track.filter.position() - Vec3{slam_.pose().x, slam_.pose().y, 0.0f};
        const Vec3 unit = normalized(Vec3{world_delta.x, world_delta.y, 0.0f});
        const float range_rate_pred = dot(track.filter.velocity(), unit);

        for (std::size_t mi = 0; mi < measurements.size(); ++mi)
        {
            const auto& uwb = measurements[mi];
            if (uwb.is_fixed_anchor || uwb.tag_id == 0 || uwb.range_m <= 0.05f) continue;
            const float range_error = std::abs(uwb.range_m - range_pred);
            if (range_error > std::max(0.75f, 3.0f * config_.tag_range_sigma_m)) continue;

            float score = range_error / std::max(0.05f, config_.tag_range_sigma_m);
            if (uwb.has_aoa)
            {
                const float bearing_error = std::abs(wrapRad(uwb.aoa_rad - bearing_pred));
                score += bearing_error / std::max(0.02f, config_.tag_bearing_sigma_rad);
            }
            if (uwb.has_range_rate)
            {
                const float velocity_error = std::abs(uwb.range_rate_mps - range_rate_pred);
                score += velocity_error / std::max(0.05f, config_.tag_velocity_sigma_mps);
            }
            if (score <= 3.0f) candidates.push_back(Candidate{score, ti, mi});
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.score != b.score) return a.score < b.score;
        if (a.track_index != b.track_index) return a.track_index < b.track_index;
        return a.measurement_index < b.measurement_index;
    });

    std::vector<uint8_t> track_used(all_tracks.size(), 0);
    std::vector<uint8_t> measurement_used(measurements.size(), 0);
    for (const auto& candidate : candidates)
    {
        if (track_used[candidate.track_index] || measurement_used[candidate.measurement_index]) continue;
        const auto& track = all_tracks[candidate.track_index];
        const auto& uwb = measurements[candidate.measurement_index];
        if (tracks_.applyIdentity(track.id, uwb.tag_id, uwb.confidence, now_ms))
        {
            track_used[candidate.track_index] = 1;
            measurement_used[candidate.measurement_index] = 1;
        }
    }
}

void FusionEngine::updatePresence(const std::vector<NoveldaMeasurement>& measurements,
                                  const std::vector<PresenceZone>& wallpad_zones,
                                  uint64_t now_ms)
{
    for (const auto& sample : measurements)
    {
        if (sample.range_m <= 0.05f || sample.confidence < 0.05f) continue;
        PresenceZone zone{};
        const float half_range = std::max(0.35f, 0.14f * sample.range_m);
        zone.range_min = std::max(0.10f, sample.range_m - half_range);
        zone.range_max = sample.range_m + half_range;
        zone.bearing_center_rad = sample.bearing_center_rad;
        zone.bearing_width_rad = clampf(sample.bearing_width_rad, degToRad(5.0f), degToRad(170.0f));
        zone.confidence = clampf(sample.confidence * (sample.through_barrier ? 0.82f : 1.0f), 0.0f, 1.0f);
        active_zones_.push_back(TimedZone{zone, now_ms});
    }
    for (const PresenceZone& zone : wallpad_zones)
    {
        if (!std::isfinite(zone.range_min) || !std::isfinite(zone.range_max) ||
            !std::isfinite(zone.bearing_center_rad) || !std::isfinite(zone.bearing_width_rad) ||
            !std::isfinite(zone.confidence) || zone.range_min <= 0.05f ||
            zone.range_max <= zone.range_min || zone.range_max > 35.0f ||
            zone.bearing_width_rad <= 0.0f || zone.bearing_width_rad > degToRad(170.0f) ||
            zone.confidence < 0.05f)
            continue;
        PresenceZone bounded_zone = zone;
        bounded_zone.bearing_center_rad = wrapRad(zone.bearing_center_rad);
        bounded_zone.confidence = clampf(zone.confidence, 0.0f, 1.0f);
        active_zones_.push_back(TimedZone{bounded_zone, now_ms});
    }

    const uint64_t timeout = static_cast<uint64_t>(config_.presence_timeout_s * 1000.0f);
    while (!active_zones_.empty() && now_ms > active_zones_.front().seen_ms + timeout)
        active_zones_.pop_front();
    while (active_zones_.size() > 16) active_zones_.pop_front();

    // Presence zones can only reinforce tracks that already exist.
    const auto& all_tracks = tracks_.tracks();
    for (const auto& timed : active_zones_)
    {
        const auto& zone = timed.zone;
        for (const auto& track : all_tracks)
        {
            if (track.visible || track.state == TrackState::VISIBLE) continue;
            const Vec3 body = slam_.worldToBody(track.filter.position());
            const float range = std::sqrt(body.x * body.x + body.y * body.y);
            const float sigma = std::max(0.15f, track.position_sigma_m);
            if (range < zone.range_min - sigma || range > zone.range_max + sigma) continue;

            const float bearing = std::atan2(body.x, body.y);
            const float angular_sigma = std::atan2(sigma, std::max(0.30f, range));
            const float allowed = 0.5f * zone.bearing_width_rad + angular_sigma;
            if (std::abs(wrapRad(bearing - zone.bearing_center_rad)) > allowed) continue;

            const float boosted = clampf(zone.confidence * (0.7f + 0.3f * (1.0f - std::min(1.0f, sigma))), 0.0f, 1.0f);
            tracks_.applyPresenceSupport(track.id, boosted, now_ms);
        }
    }
}

void FusionEngine::updateMap(const LidarScan* lidar_scan,
                             const TofScan* tof_scan,
                             const std::vector<Detection>& radar_detections,
                             uint64_t now_ms)
{
    occupancy_.decay(config_.occupancy_decay);

    // Schematic priors are re-applied after decay. Sensor rays still refine the
    // same cells, so a wrong schematic is visible as a conflict rather than
    // being silently accepted.
    if (!schematic_.empty()) schematic_.applyToGrid(occupancy_);

    const auto& all_tracks = tracks_.tracks();
    if (tof_scan && health_.tof)
    {
        for (const SurfaceRay& ray : tof_scan->rays)
        {
            if (!std::isfinite(ray.origin.x) || !std::isfinite(ray.origin.y) ||
                !std::isfinite(ray.origin.z) || !std::isfinite(ray.endpoint.x) ||
                !std::isfinite(ray.endpoint.y) || !std::isfinite(ray.endpoint.z))
                continue;
            if (norm(ray.endpoint - ray.origin) > 4.0f) continue;

            const Vec3 endpoint = slam_.bodyToWorld(ray.endpoint);
            bool near_person = false;
            for (const auto& track : all_tracks)
            {
                const Vec3 delta = endpoint - track.filter.position();
                if (std::sqrt(delta.x * delta.x + delta.y * delta.y) < config_.map_people_exclusion_m)
                {
                    near_person = true;
                    break;
                }
            }
            if (near_person) continue;

            occupancy_.integrateRay(slam_.bodyToWorld(ray.origin), endpoint, 0.78f, -0.28f, 4.0f);
        }
    }

    if (health_.lidar)
    {
        if (lidar_scan)
        {
            const Vec3 origin = slam_.bodyToWorld(lidar_scan->origin);
            for (const Vec3& point : lidar_scan->points)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
                if (norm(point - lidar_scan->origin) > config_.grid.range_m) continue;
                const Vec3 endpoint = slam_.bodyToWorld(point);

                bool near_person = false;
                for (const auto& track : all_tracks)
                {
                    const Vec3 delta = endpoint - track.filter.position();
                    if (std::sqrt(delta.x * delta.x + delta.y * delta.y) < config_.map_people_exclusion_m)
                    {
                        near_person = true;
                        break;
                    }
                }
                if (near_person) continue;
                occupancy_.integrateRay(origin, endpoint, 0.85f, -0.35f, config_.grid.range_m);
            }
        }
    }
    else if (!health_.tof)
    {
        const Vec3 origin{slam_.pose().x, slam_.pose().y, 0.0f};
        for (const auto& detection : radar_detections)
        {
            if (!(detection.snr > 1.0f)) continue;
            if (detection.range > config_.max_mapping_range_m) continue;

            bool near_person = false;
            for (const auto& track : all_tracks)
            {
                const Vec3 delta = detection.position - track.filter.position();
                if (std::sqrt(delta.x * delta.x + delta.y * delta.y) < config_.map_people_exclusion_m)
                {
                    near_person = true;
                    break;
                }
            }
            if (near_person) continue;
            occupancy_.integrateRay(origin, detection.position, 0.85f, -0.35f, config_.max_mapping_range_m);
        }
    }

    const uint64_t interval_ms = static_cast<uint64_t>(config_.map_refresh_interval_s * 1000.0f);
    if (!last_map_refresh_ms_ || now_ms >= *last_map_refresh_ms_ + interval_ms)
    {
        static_objects_ = occupancy_.extractMapObjects();
        last_map_refresh_ms_ = now_ms;
    }
    updateStaticObjects(all_tracks);
}

void FusionEngine::updateStaticObjects(const std::vector<Track>& tracks)
{
    // A map object may never be a person track. This is a second line of
    // defence on top of excluding detections near tracks during mapping.
    std::erase_if(static_objects_, [&](const MapObject& object) {
        for (const auto& track : tracks)
        {
            const Vec3 delta = object.position - track.filter.position();
            if (std::sqrt(delta.x * delta.x + delta.y * delta.y) < config_.map_people_exclusion_m) return true;
        }
        return false;
    });
}

WorldModel FusionEngine::process(const SensorFrame& frame)
{
    const uint64_t now_ms = frame.timestamp_ms;
    float dt = 0.0f;
    if (last_process_ms_ && now_ms >= *last_process_ms_)
        dt = static_cast<float>(now_ms - *last_process_ms_) / 1000.0f;
    dt = clampf(dt, 0.0f, 2.0f);
    last_process_ms_ = now_ms;

    if (frame.imu)
    {
        last_imu_ms_ = frame.imu->timestamp_ms;
        slam_.update(*frame.imu, dt);
    }

    if (frame.lidar)
        last_lidar_ms_ = frame.lidar->timestamp_ms;
    if (frame.tof)
        last_tof_ms_ = frame.tof->timestamp_ms;
    if (frame.radar)
        last_radar_ms_ = frame.radar->timestamp_ms;
    if (!frame.uwb.empty())
        last_uwb_ms_ = now_ms;
    if (!frame.novelda.empty())
        last_novelda_ms_ = now_ms;
    if (frame.ultrasonic)
        last_ultrasonic_ms_ = frame.ultrasonic->timestamp_ms;
    updateHealth(now_ms);
    for (const UwbMeasurement& measurement : frame.uwb)
    {
        if (measurement.is_fixed_anchor) slam_.correctPose(measurement);
    }

    WorldModel map_context{};
    map_context.health = health_;
    const float prior_environment_confidence =
        confidence_.evaluate(map_context, slam_, occupancy_).environment;
    const bool lidar_fresh =
        frame.lidar && fresh(last_lidar_ms_, now_ms, kLidarTimeoutMs);
    bool loop_closure_applied = false;
    if (lidar_fresh)
        loop_closure_applied =
            slam_.update(*frame.lidar, occupancy_, prior_environment_confidence);
    if (frame.tof && fresh(last_tof_ms_, now_ms, kTofTimeoutMs) &&
        (!lidar_fresh || frame.lidar->points.size() < 8))
        slam_.update(*frame.tof, occupancy_, prior_environment_confidence);

    std::vector<Detection> world_detections;
    if (frame.radar)
    {
        world_detections = radarToWorld(*frame.radar);
        world_detections.erase(std::remove_if(world_detections.begin(), world_detections.end(), [](const Detection& d) {
                                   return !std::isfinite(d.position.x) || !std::isfinite(d.position.y) ||
                                          !std::isfinite(d.position.z);
                               }),
                               world_detections.end());
    }

    // Only compact torso-height clusters are allowed to create or update
    // person tracks. Static structure stays in the occupancy map.
    const std::vector<Detection> person_candidates = selectPersonCandidates(world_detections);
    tracks_.process(dt, person_candidates, now_ms);

    if (!frame.uwb.empty()) updateUwb(frame.uwb, now_ms);
    if (!frame.novelda.empty() || !frame.wallpad_zones.empty())
    {
        updatePresence(frame.novelda, frame.wallpad_zones, now_ms);
    }
    updateMap(frame.lidar ? &*frame.lidar : nullptr,
              frame.tof ? &*frame.tof : nullptr,
              world_detections,
              now_ms);

    WorldModel world{};
    world.timestamp_ms = now_ms;
    world.pose = slam_.pose();
    world.health = health_;
    world.occupancy = occupancy_;
    world.static_objects = static_objects_;
    if (!schematic_.empty())
    {
        for (const auto& mark : schematic_.marks())
        {
            MapObject object{};
            object.position = mark.position;
            object.confidence = 0.90f;
            world.static_objects.push_back(object);
        }
        world.walls = schematic_.walls();
        world.walls.insert(world.walls.end(), schematic_.doors().begin(), schematic_.doors().end());
        world.rooms = schematic_.rooms();
    }
    world.tracks = tracks_.tracks();
    world.zones.reserve(active_zones_.size());
    for (const auto& timed : active_zones_) world.zones.push_back(timed.zone);

    confidence_.apply(world, slam_, occupancy_);
    if (loop_closure_applied)
        world.environment_confidence = clampf(world.environment_confidence + 0.05f, 0.0f, 0.99f);
    return world;
}

ConfidenceBreakdown ConfidenceEngine::evaluate(const WorldModel& world,
                                               const SlamManager& slam,
                                               const OccupancyGrid& grid) const
{
    ConfidenceBreakdown out;
    float max_existence = 0.0f;
    float sum_position = 0.0f;
    int visible = 0;
    float sum_identity = 0.0f;
    int tagged = 0;
    for (const auto& track : world.tracks)
    {
        max_existence = std::max(max_existence, track.existence_confidence);
        if (track.visible)
        {
            sum_position += track.position_confidence;
            ++visible;
        }
        if (track.identity.tagged_safe)
        {
            sum_identity += track.identity.confidence;
            ++tagged;
        }
    }
    out.existence = max_existence;
    out.position = visible > 0 ? sum_position / static_cast<float>(visible) : 0.0f;
    out.identity = tagged > 0 ? sum_identity / static_cast<float>(tagged) : 0.0f;

    const float localization = clampf(slam.confidence(), 0.0f, 1.0f);
    const float map_coverage = clampf(static_cast<float>(grid.occupiedCellCount()) / 500.0f, 0.0f, 1.0f);
    const float map_known = clampf(static_cast<float>(grid.knownCellCount()) / 5000.0f, 0.0f, 1.0f);
    const float map_trust = 0.65f * map_coverage + 0.35f * map_known;
    out.environment = clampf(0.48f * localization + 0.30f * map_trust + 0.22f * world.health.health_score, 0.0f, 1.0f);
    return out;
}

void ConfidenceEngine::apply(WorldModel& world, const SlamManager& slam, const OccupancyGrid& grid) const
{
    world.environment_confidence = evaluate(world, slam, grid).environment;
}

} // namespace sa
