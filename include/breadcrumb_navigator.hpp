#pragma once

#include "protocol_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace sa {

class BreadcrumbNavigator
{
public:
    static constexpr float kWaypointSpacingM = 0.5f;
    static constexpr std::size_t kMaxWaypoints = kHudMaxBreadcrumbs;

    bool setEnabled(bool enabled, const Pose2D& pose, uint64_t timestamp_ms);
    bool update(const Pose2D& pose, uint64_t timestamp_ms);
    void purge();

    bool enabled() const { return enabled_; }
    std::span<const HudPoint> breadcrumbs() const { return {waypoints_.data(), count_}; }
    std::size_t returnWaypointCount() const { return count_; }
    const HudPoint* returnWaypoint(std::size_t index) const;
    float returnHeadingDeg(const Pose2D& pose) const;

private:
    static bool validPose(const Pose2D& pose);
    void clearWaypoints();

    std::array<HudPoint, kMaxWaypoints> waypoints_{};
    HudPoint sample_anchor_{};
    std::size_t count_ = 0;
    uint64_t last_timestamp_ms_ = 0;
    bool has_timestamp_ = false;
    bool enabled_ = true;
};

} // namespace sa
