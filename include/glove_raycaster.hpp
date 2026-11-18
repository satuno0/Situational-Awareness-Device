#pragma once

#include "types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace sa {

struct GlovePosePacket
{
    uint8_t sequence = 0;
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    bool trigger_held = false;
    bool zero_heading_requested = false;
    uint8_t events = 0;
};

constexpr uint8_t kGloveEventRadarPing = 1u << 0;
constexpr uint8_t kGloveEventQuickGlance = 1u << 1;
constexpr uint8_t kGloveEventTrailToggle = 1u << 2;
constexpr uint8_t kGloveEventPanicWipe = 1u << 3;

struct GloveRayState
{
    bool active = false;
    float yaw_deg = 0.0f;
    uint32_t locked_target_id = 0;
    float locked_dist_m = 0.0f;
};

bool parseGlovePosePacket(const uint8_t* data,
                          std::size_t size,
                          GlovePosePacket& packet,
                          std::string& error);

class GloveRaycaster
{
public:
    static constexpr float kHalfAngleDeg = 7.5f;
    static constexpr float kMaxRangeM = 8.0f;
    static constexpr uint64_t kPacketTimeoutMs = 250;

    GloveRayState update(const WorldModel& world,
                         const std::optional<GlovePosePacket>& packet,
                         uint64_t timestamp_ms);
    void disableScan();
    void reset();

private:
    std::optional<GlovePosePacket> latest_packet_;
    uint64_t latest_packet_timestamp_ms_ = 0;
    float zero_heading_offset_deg_ = 0.0f;
    bool scan_enabled_ = false;
    bool button_was_pressed_ = false;
};

} // namespace sa
