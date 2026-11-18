#pragma once

#include "types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace sa {

constexpr std::size_t kHudMaxWalls = 128;
constexpr std::size_t kHudMaxTargets = 64;
constexpr std::size_t kHudMaxPresenceZones = kMaxWallpadZonesPerFrame;
constexpr std::size_t kHudMaxBreadcrumbs = 512;
constexpr std::size_t kHudAnnouncementBytes = 128;

enum class HudTargetSource : uint8_t
{
    Mmwave,
    Uwb,
    Tof,
    Lidar
};

struct HudLineSegment
{
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
};

struct HudTarget
{
    float x = 0.0f;
    float y = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float confidence = 0.0f;
    HudTargetSource source = HudTargetSource::Mmwave;
};

struct HudPoint
{
    float x = 0.0f;
    float y = 0.0f;
};

struct HudGloveRay
{
    bool active = false;
    float yaw_deg = 0.0f;
    uint32_t locked_target_id = 0;
    float locked_dist_m = 0.0f;
};

struct HudRenderFrame
{
    Pose2D pose{0.0f, 0.0f, 0.0f};
    int floor = 0;
    std::array<HudLineSegment, kHudMaxWalls> walls{};
    std::size_t wall_count = 0;
    std::array<HudTarget, kHudMaxTargets> targets{};
    std::size_t target_count = 0;
    std::array<PresenceZone, kHudMaxPresenceZones> presence_zones{};
    std::size_t presence_zone_count = 0;
    std::array<HudPoint, kHudMaxBreadcrumbs> breadcrumb_path{};
    std::size_t breadcrumb_count = 0;
    float return_heading_deg = 0.0f;
    bool trail_enabled = false;
    bool auto_mode = false;
    bool declutter = false;
    bool blackout = false;
    bool has_glove_ray = false;
    HudGloveRay glove_ray{};
    std::array<char, kHudAnnouncementBytes> announcement{};
};

} // namespace sa
