#pragma once

#include "types.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace sa {

struct TrackManagerConfig
{
    float gate_m = 1.40f;
    float gate_sigma_factor = 2.5f;
    float untagged_expiry_s = 10.0f;
    float tagged_expiry_s = 90.0f;
    float untagged_existence_tau_s = 3.2f;
    float tagged_existence_tau_s = 25.0f;
    float presence_hold_s = 2.0f;
    float min_existence = 0.08f;
    float identity_switch_lockout_s = 10.0f;
    float coast_velocity_damping_per_s = 2.0f;
    float duplicate_merge_m = 0.30f;
    std::size_t max_tracks = 64;
};

class TrackManager
{
public:
    TrackManager();
    explicit TrackManager(const TrackManagerConfig& config);

    void reset();

    // Predicts, performs deterministic gated association, updates and spawns.
    void process(float dt, const std::vector<Detection>& detections, uint64_t now_ms);

    const std::vector<Track>& tracks() const { return tracks_; }
    const Track* findTrack(uint32_t id) const;

    // UWB confirmation only. It never creates a track.
    bool applyIdentity(uint32_t id, uint16_t tag_id, float measurement_confidence, uint64_t now_ms);

    // Novelda confirmation only. It only reinforces an already-occluded track.
    bool applyPresenceSupport(uint32_t id, float zone_confidence, uint64_t now_ms);

    uint64_t lastTagSeenMs(uint32_t id) const;
    const TrackManagerConfig& config() const { return config_; }

private:
    struct Meta
    {
        uint64_t first_seen_ms = 0;
        uint64_t last_seen_ms = 0;
        uint64_t presence_until_ms = 0;
        uint64_t last_tag_seen_ms = 0;
        float existence_base = 0.65f;
    };

    void updateDerived(Track& track);
    void refreshStates(uint64_t now_ms);
    void removeExpired(uint64_t now_ms);
    void mergeDuplicates();
    Track* mutableTrack(uint32_t id);
    void spawn(const Detection& detection, uint64_t now_ms);

    TrackManagerConfig config_;
    std::vector<Track> tracks_;
    std::unordered_map<uint32_t, Meta> meta_;
    uint32_t next_id_ = 1;
};

} // namespace sa
