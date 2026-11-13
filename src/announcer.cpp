#include "announcer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

namespace sa {
namespace {

std::string countPhrase(int count, const char* singular, const char* plural)
{
    return std::to_string(count) + " " + (count == 1 ? singular : plural);
}

std::string distancePhrase(float range_m)
{
    int meters = static_cast<int>(std::lround(std::max(0.0f, range_m)));
    if (meters < 1) meters = 1;
    return std::to_string(meters) + (meters == 1 ? " meter" : " meters");
}

// Positive bearing is left of the device forward axis.
std::string directionPhrase(float bearing_deg)
{
    const float magnitude = std::abs(bearing_deg);
    if (magnitude <= 22.5f) return "ahead";
    if (magnitude <= 67.5f) return bearing_deg > 0.0f ? "to the left" : "to the right";
    if (magnitude <= 112.5f) return bearing_deg > 0.0f ? "on the left" : "on the right";
    if (magnitude <= 157.5f) return bearing_deg > 0.0f ? "behind and to the left" : "behind and to the right";
    return "behind";
}

const char* classLabel(PersonClass cls)
{
    switch (cls)
    {
    case PersonClass::SAFE_TAGGED: return "Friendly";
    case PersonClass::PRESENCE_ONLY: return "Presence";
    case PersonClass::UNKNOWN_PERSON: break;
    }
    return "Unit";
}

struct Entry
{
    float range_m = 0.0f;
    float bearing_deg = 0.0f;
    PersonClass cls = PersonClass::UNKNOWN_PERSON;
    uint32_t id = 0;
};

} // namespace

std::string buildAnnouncement(const WorldModel& world, const AnnounceConfig& config)
{
    std::vector<Entry> entries;
    entries.reserve(world.tracks.size());
    for (const auto& track : world.tracks)
    {
        if (track.state == TrackState::EXPIRED) continue;
        if (track.existence_confidence < config.min_existence) continue;

        const Vec3 body = poseInverseApply(world.pose, track.filter.position());
        const float range = std::sqrt(body.x * body.x + body.y * body.y);
        if (!std::isfinite(range) || range < config.min_report_range_m || range > config.max_range_m) continue;

        Entry entry;
        entry.range_m = range;
        entry.bearing_deg = radToDeg(std::atan2(body.y, body.x));
        entry.cls = classifyTrack(track);
        entry.id = track.id;
        entries.push_back(entry);
    }

    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
        if (a.range_m != b.range_m) return a.range_m < b.range_m;
        return a.id < b.id;
    });

    if (entries.empty())
    {
        if (!world.zones.empty()) return "Presence detected, no tracked people.";
        return "Area clear.";
    }

    int units = 0;
    int friendlies = 0;
    int presence = 0;
    for (const auto& entry : entries)
    {
        if (entry.cls == PersonClass::SAFE_TAGGED) ++friendlies;
        else if (entry.cls == PersonClass::PRESENCE_ONLY) ++presence;
        else ++units;
    }

    std::ostringstream out;
    const int total = static_cast<int>(entries.size());
    if (total == 1)
    {
        const auto& entry = entries.front();
        out << classLabel(entry.cls) << " " << distancePhrase(entry.range_m) << " "
            << directionPhrase(entry.bearing_deg) << ".";
        return out.str();
    }

    out << total << " people:";
    bool first = true;
    const auto appendCount = [&](int count, const char* singular, const char* plural) {
        if (count <= 0) return;
        out << (first ? " " : ", ") << countPhrase(count, singular, plural);
        first = false;
    };
    appendCount(units, "unit", "units");
    appendCount(friendlies, "friendly", "friendlies");
    appendCount(presence, "presence", "presence contacts");
    out << ".";

    const std::size_t limit = std::min(entries.size(), std::max<std::size_t>(1, config.max_entries));
    for (std::size_t i = 0; i < limit; ++i)
    {
        const auto& entry = entries[i];
        out << " " << classLabel(entry.cls) << " " << distancePhrase(entry.range_m) << " "
            << directionPhrase(entry.bearing_deg) << ".";
    }
    return out.str();
}

Announcer::Announcer() : Announcer(AnnounceConfig{}) {}
Announcer::Announcer(const AnnounceConfig& config) : config_(config) {}
Announcer::~Announcer() { stop(); }

bool Announcer::start(std::string& error)
{
    stop();
    if (!config_.enabled) return true;
    if (!config_.device_path.empty())
    {
        device_.open(config_.device_path, std::ios::out | std::ios::app);
        if (!device_)
        {
            error = "cannot open announcement device " + config_.device_path;
            return false;
        }
    }
    running_ = true;
    last_emit_ms_.reset();
    last_text_.clear();
    return true;
}

void Announcer::stop()
{
    if (device_.is_open()) device_.close();
    running_ = false;
}

void Announcer::emit(const std::string& text, uint64_t now_ms)
{
    if (device_.is_open())
    {
        device_ << "SAY " << text << "\n";
        device_.flush();
    }
    last_emit_ms_ = now_ms;
    last_text_ = text;
    if (config_.verbose) std::clog << "[announce] " << text << "\n";
}

void Announcer::update(const WorldModel& world, uint64_t now_ms)
{
    if (!config_.enabled || !running_) return;

    const std::string text = buildAnnouncement(world, config_);
    if (text.empty()) return;

    uint64_t required_ms = static_cast<uint64_t>(
        (text != last_text_ ? config_.min_interval_s : config_.repeat_interval_s) * 1000.0);
    if (!last_emit_ms_ || now_ms < *last_emit_ms_)
    {
        emit(text, now_ms);
        return;
    }
    if (now_ms - *last_emit_ms_ >= required_ms) emit(text, now_ms);
}

} // namespace sa
