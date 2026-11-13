#pragma once

#include "kalman_track.hpp"
#include "math.hpp"
#include "occupancy_grid.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sa {

constexpr int kGridWidth = 160;
constexpr int kGridHeight = 160;
constexpr float kGridCellSizeM = 0.25f;
constexpr float kGridRangeM = 20.0f;

struct SensorHealth
{
    bool imu;
    bool radar;
    bool uwb;
    bool novelda;
    bool ultrasonic;

    float health_score;
};

enum class TrackState
{
    VISIBLE,
    OCCLUDED,
    PRESENCE_SUPPORTED,
    EXPIRED
};

enum class PersonClass
{
    SAFE_TAGGED,
    UNKNOWN_PERSON,
    PRESENCE_ONLY
};

struct IdentityInfo
{
    bool tagged_safe;

    uint16_t tag_id;

    float confidence;
};

struct Detection
{
    Vec3 position;

    Vec3 velocity;

    float snr;

    float range;

    bool from_radar;
};

struct Track
{
    uint32_t id;

    KalmanTrack filter;

    IdentityInfo identity;

    TrackState state;

    float existence_confidence;

    float position_confidence;

    float position_sigma_m;

    bool visible;
};

struct PresenceZone
{
    float range_min;

    float range_max;

    float bearing_center_rad;

    float bearing_width_rad;

    float confidence;
};

// Static structure imported from an operator-supplied schematic. Walls are
// world-frame segments; rooms are axis-aligned world-frame rectangles. These
// are priors, not sensor detections, and never identify people.
struct MapWall
{
    Vec3 start;
    Vec3 end;
    float height;
};

struct MapRoom
{
    std::string name;
    float min_x;
    float min_y;
    float max_x;
    float max_y;
};

struct WorldModel
{
    uint64_t timestamp_ms;

    Pose2D pose;

    float environment_confidence;

    SensorHealth health;

    OccupancyGrid occupancy;

    std::vector<MapObject> static_objects;

    std::vector<Track> tracks;

    std::vector<PresenceZone> zones;

    std::vector<MapWall> walls;

    std::vector<MapRoom> rooms;
};

struct ImuSample
{
    uint64_t timestamp_ms = 0;
    Vec3 accel_mps2{0.0f, 0.0f, kGravity};
    Vec3 gyro_radps{0.0f, 0.0f, 0.0f};
    float temperature_c = 25.0f;
};

struct RadarFrame
{
    uint64_t timestamp_ms = 0;
    std::vector<Detection> detections;
};

struct UwbMeasurement
{
    uint64_t timestamp_ms = 0;
    uint16_t tag_id = 0;
    float range_m = 0.0f;
    bool has_aoa = false;
    float aoa_rad = 0.0f;
    bool has_range_rate = false;
    float range_rate_mps = 0.0f;
    float confidence = 0.5f;
};

struct NoveldaMeasurement
{
    uint64_t timestamp_ms = 0;
    float range_m = 0.0f;
    float bearing_center_rad = 0.0f;
    float bearing_width_rad = degToRad(60.0f);
    float motion = 0.0f;
    float breathing = 0.0f;
    float confidence = 0.0f;
    bool through_barrier = false;
};

struct UltrasonicMeasurement
{
    uint64_t timestamp_ms = 0;
    float range_m = 0.0f;
    bool valid = false;
};

// Visual red-marker bearing. Bearing is measured relative to the camera axis,
// positive toward the left. A red marker is a navigation cue only; it is never
// used for person identification.
struct MarkerObservation
{
    uint64_t timestamp_ms = 0;
    float bearing_rad = 0.0f;
    float confidence = 0.0f;
    float area_ratio = 0.0f;
    bool valid = false;
};

struct SensorFrame
{
    uint32_t frame_id = 0;
    uint64_t timestamp_ms = 0;
    std::optional<ImuSample> imu;
    std::optional<RadarFrame> radar;
    std::vector<UwbMeasurement> uwb;
    std::vector<NoveldaMeasurement> novelda;
    std::optional<UltrasonicMeasurement> ultrasonic;
    std::optional<MarkerObservation> marker;
};

inline const char* trackStateName(TrackState state)
{
    switch (state)
    {
    case TrackState::VISIBLE: return "VISIBLE";
    case TrackState::OCCLUDED: return "OCCLUDED";
    case TrackState::PRESENCE_SUPPORTED: return "PRESENCE_SUPPORTED";
    case TrackState::EXPIRED: return "EXPIRED";
    }
    return "EXPIRED";
}

inline const char* personClassName(PersonClass cls)
{
    switch (cls)
    {
    case PersonClass::SAFE_TAGGED: return "SAFE_TAGGED";
    case PersonClass::UNKNOWN_PERSON: return "UNKNOWN_PERSON";
    case PersonClass::PRESENCE_ONLY: return "PRESENCE_ONLY";
    }
    return "UNKNOWN_PERSON";
}

inline PersonClass classifyTrack(const Track& track)
{
    if (track.identity.tagged_safe) return PersonClass::SAFE_TAGGED;
    if (!track.visible && track.state == TrackState::PRESENCE_SUPPORTED) return PersonClass::PRESENCE_ONLY;
    return PersonClass::UNKNOWN_PERSON;
}

inline uint64_t monoNowMs()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

inline uint64_t wallClockMs()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
}

} // namespace sa
