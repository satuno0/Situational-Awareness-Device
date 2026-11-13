#include "track_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sa {

TrackManager::TrackManager() : TrackManager(TrackManagerConfig{}) {}

TrackManager::TrackManager(const TrackManagerConfig& config) : config_(config)
{
    reset();
}

void TrackManager::reset()
{
    tracks_.clear();
    meta_.clear();
    next_id_ = 1;
}

const Track* TrackManager::findTrack(uint32_t id) const
{
    for (const auto& track : tracks_)
        if (track.id == id) return &track;
    return nullptr;
}

Track* TrackManager::mutableTrack(uint32_t id)
{
    for (auto& track : tracks_)
        if (track.id == id) return &track;
    return nullptr;
}

void TrackManager::updateDerived(Track& track)
{
    track.position_sigma_m = track.filter.positionSigma();
    track.position_confidence = clampf(std::exp(-0.8f * track.position_sigma_m), 0.0f, 1.0f);
}

void TrackManager::spawn(const Detection& detection, uint64_t now_ms)
{
    if (tracks_.size() >= config_.max_tracks) return;

    Track track{};
    track.id = next_id_++;
    track.filter = KalmanTrack(detection, now_ms);
    track.identity = IdentityInfo{false, 0, 0.0f};
    track.state = TrackState::VISIBLE;
    track.existence_confidence = 0.65f;
    track.visible = true;
    updateDerived(track);

    Meta meta;
    meta.first_seen_ms = now_ms;
    meta.last_seen_ms = now_ms;
    meta.existence_base = 0.65f;
    meta_.emplace(track.id, meta);
    tracks_.push_back(track);
}

void TrackManager::process(float dt, const std::vector<Detection>& detections, uint64_t now_ms)
{
    if (dt > 0.0f)
    {
        for (auto& track : tracks_) track.filter.predict(dt);
    }

    struct Candidate
    {
        float score;
        std::size_t track_index;
        std::size_t detection_index;
    };

    std::vector<Candidate> candidates;
    candidates.reserve(tracks_.size() * detections.size());

    for (std::size_t ti = 0; ti < tracks_.size(); ++ti)
    {
        auto& track = tracks_[ti];
        const Vec3 predicted = track.filter.position();
        const float sigma = std::max(0.10f, track.filter.positionSigma());
        const float gate = std::max(config_.gate_m, config_.gate_sigma_factor * sigma);
        for (std::size_t di = 0; di < detections.size(); ++di)
        {
            const auto& det = detections[di];
            const float distance = norm(det.position - predicted);
            if (distance > gate) continue;
            const float velocity_error = norm(det.velocity - track.filter.velocity());
            const float score = distance / gate + 0.08f * velocity_error;
            candidates.push_back(Candidate{score, ti, di});
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.score != b.score) return a.score < b.score;
        if (a.track_index != b.track_index) return a.track_index < b.track_index;
        return a.detection_index < b.detection_index;
    });

    std::vector<uint8_t> track_used(tracks_.size(), 0);
    std::vector<uint8_t> detection_used(detections.size(), 0);

    for (const auto& candidate : candidates)
    {
        if (track_used[candidate.track_index] || detection_used[candidate.detection_index]) continue;
        track_used[candidate.track_index] = 1;
        detection_used[candidate.detection_index] = 1;

        auto& track = tracks_[candidate.track_index];
        const auto& det = detections[candidate.detection_index];
        track.filter.update(det);
        track.filter.setLastUpdateMs(now_ms);
        track.state = TrackState::VISIBLE;
        track.visible = true;
        updateDerived(track);

        auto& meta = meta_[track.id];
        meta.last_seen_ms = now_ms;
        meta.existence_base = std::min(1.0f, meta.existence_base + (1.0f - meta.existence_base) * 0.45f);
        meta.existence_base = std::max(meta.existence_base, 0.72f);
        track.existence_confidence = meta.existence_base;
    }

    for (std::size_t di = 0; di < detections.size(); ++di)
    {
        if (!detection_used[di]) spawn(detections[di], now_ms);
    }

    if (dt > 0.0f)
    {
        const float coast_factor = std::exp(-config_.coast_velocity_damping_per_s * dt);
        for (std::size_t ti = 0; ti < tracks_.size() && ti < track_used.size(); ++ti)
        {
            if (!track_used[ti]) tracks_[ti].filter.scaleVelocity(coast_factor);
        }
    }

    mergeDuplicates();
    refreshStates(now_ms);
    removeExpired(now_ms);
}

void TrackManager::mergeDuplicates()
{
    if (tracks_.size() < 2) return;
    const float merge2 = config_.duplicate_merge_m * config_.duplicate_merge_m;
    std::vector<uint32_t> remove;
    auto is_removed = [&](uint32_t id) {
        return std::find(remove.begin(), remove.end(), id) != remove.end();
    };

    for (std::size_t i = 0; i < tracks_.size(); ++i)
    {
        if (is_removed(tracks_[i].id)) continue;
        for (std::size_t j = i + 1; j < tracks_.size(); ++j)
        {
            if (is_removed(tracks_[j].id)) continue;
            const Vec3 delta = tracks_[i].filter.position() - tracks_[j].filter.position();
            if (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z > merge2) continue;

            const bool j_better = (tracks_[j].identity.tagged_safe && !tracks_[i].identity.tagged_safe) ||
                                  (tracks_[j].identity.tagged_safe == tracks_[i].identity.tagged_safe &&
                                   tracks_[j].visible && !tracks_[i].visible);
            const std::size_t keep_index = j_better ? j : i;
            const std::size_t drop_index = j_better ? i : j;

            Track& keep = tracks_[keep_index];
            const Track& drop = tracks_[drop_index];
            if (!keep.identity.tagged_safe && drop.identity.tagged_safe) keep.identity = drop.identity;
            keep.existence_confidence = std::max(keep.existence_confidence, drop.existence_confidence);
            remove.push_back(drop.id);
            if (drop_index == i) break;
        }
    }

    if (remove.empty()) return;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
                      return is_removed(track.id);
                  }),
                  tracks_.end());
    for (uint32_t id : remove) meta_.erase(id);
}

void TrackManager::refreshStates(uint64_t now_ms)
{
    for (auto& track : tracks_)
    {
        auto it = meta_.find(track.id);
        if (it == meta_.end()) continue;
        Meta& meta = it->second;

        const float age_s = static_cast<float>(now_ms - meta.last_seen_ms) / 1000.0f;
        const bool tagged = track.identity.tagged_safe;
        const float tau = tagged ? config_.tagged_existence_tau_s : config_.untagged_existence_tau_s;
        const float decayed = meta.existence_base * std::exp(-std::max(0.0f, age_s) / std::max(0.2f, tau));
        track.existence_confidence = decayed;
        track.visible = false;

        if (meta.last_seen_ms == now_ms)
        {
            track.state = TrackState::VISIBLE;
            track.visible = true;
            track.existence_confidence = meta.existence_base;
        }
        else if (now_ms <= meta.presence_until_ms)
        {
            track.state = TrackState::PRESENCE_SUPPORTED;
            track.existence_confidence = std::max(track.existence_confidence, 0.42f);
        }
        else
        {
            track.state = TrackState::OCCLUDED;
        }

        updateDerived(track);
    }
}

void TrackManager::removeExpired(uint64_t now_ms)
{
    std::vector<uint32_t> remove;
    remove.reserve(tracks_.size());
    for (const auto& track : tracks_)
    {
        const auto it = meta_.find(track.id);
        if (it == meta_.end())
        {
            remove.push_back(track.id);
            continue;
        }
        const Meta& meta = it->second;
        const float age_s = static_cast<float>(now_ms - meta.last_seen_ms) / 1000.0f;
        const float max_age_s = track.identity.tagged_safe ? config_.tagged_expiry_s : config_.untagged_expiry_s;
        const bool presence_active = now_ms <= meta.presence_until_ms;
        const bool expired = !presence_active &&
                             (age_s > max_age_s || track.existence_confidence < config_.min_existence);
        if (expired) remove.push_back(track.id);
    }

    if (remove.empty()) return;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
                      return std::find(remove.begin(), remove.end(), track.id) != remove.end();
                  }),
                  tracks_.end());
    for (uint32_t id : remove) meta_.erase(id);
}

uint64_t TrackManager::lastTagSeenMs(uint32_t id) const
{
    const auto it = meta_.find(id);
    return it == meta_.end() ? 0 : it->second.last_tag_seen_ms;
}

bool TrackManager::applyIdentity(uint32_t id, uint16_t tag_id, float measurement_confidence, uint64_t now_ms)
{
    if (tag_id == 0) return false;
    Track* track = mutableTrack(id);
    if (!track) return false;

    auto& meta = meta_[id];
    const bool different = track->identity.tagged_safe && track->identity.tag_id != tag_id;
    if (different && meta.last_tag_seen_ms != 0 &&
        now_ms - meta.last_tag_seen_ms < static_cast<uint64_t>(config_.identity_switch_lockout_s * 1000.0f) &&
        track->identity.confidence > 0.45f)
    {
        return false;
    }

    const float confidence = clampf(measurement_confidence, 0.0f, 1.0f);

    // A physical tag can only own one track at a time. If the lowest-score
    // association has moved to a fresh track, transfer ownership rather than
    // leaving the same tag printed on two people.
    for (auto& other : tracks_)
    {
        if (other.id == id) continue;
        if (!other.identity.tagged_safe || other.identity.tag_id != tag_id) continue;
        other.identity = IdentityInfo{false, 0, 0.0f};
        auto other_meta = meta_.find(other.id);
        if (other_meta != meta_.end()) other_meta->second.last_tag_seen_ms = 0;
    }

    if (!track->identity.tagged_safe || different)
    {
        track->identity.tagged_safe = true;
        track->identity.tag_id = tag_id;
        track->identity.confidence = clampf(0.62f + 0.30f * confidence, 0.0f, 0.95f);
    }
    else
    {
        track->identity.tagged_safe = true;
        track->identity.tag_id = tag_id;
        track->identity.confidence = clampf(
            track->identity.confidence + (1.0f - track->identity.confidence) * (0.20f + 0.50f * confidence),
            0.0f,
            0.995f);
    }

    meta.last_tag_seen_ms = now_ms;
    meta.existence_base = std::min(1.0f, std::max(meta.existence_base, 0.80f));
    track->existence_confidence = std::max(track->existence_confidence, meta.existence_base);
    return true;
}

bool TrackManager::applyPresenceSupport(uint32_t id, float zone_confidence, uint64_t now_ms)
{
    Track* track = mutableTrack(id);
    if (!track) return false;
    if (track->visible || track->state == TrackState::VISIBLE) return false;

    const float c = clampf(zone_confidence, 0.0f, 1.0f);
    auto& meta = meta_[id];
    meta.presence_until_ms = now_ms + static_cast<uint64_t>(config_.presence_hold_s * 1000.0f);
    meta.existence_base = std::min(0.96f,
                                   std::max(meta.existence_base,
                                            track->existence_confidence + (1.0f - track->existence_confidence) *
                                                                              (0.18f + 0.55f * c)));
    track->existence_confidence = std::max(track->existence_confidence, meta.existence_base);
    if (!track->visible) track->state = TrackState::PRESENCE_SUPPORTED;
    return true;
}

} // namespace sa
