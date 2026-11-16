#include "breadcrumb_navigator.hpp"

#include <cmath>

namespace sa {

bool BreadcrumbNavigator::validPose(const Pose2D& pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw_deg);
}

void BreadcrumbNavigator::clearWaypoints()
{
    volatile unsigned char* bytes =
        reinterpret_cast<volatile unsigned char*>(waypoints_.data());
    for (std::size_t i = 0; i < sizeof(waypoints_); ++i) bytes[i] = 0;
    count_ = 0;
    has_timestamp_ = false;
    last_timestamp_ms_ = 0;
}

bool BreadcrumbNavigator::setEnabled(bool enabled, const Pose2D& pose, uint64_t timestamp_ms)
{
    if (!enabled)
    {
        enabled_ = false;
        clearWaypoints();
        return true;
    }
    if (enabled_) return true;
    if (!validPose(pose)) return false;

    clearWaypoints();
    waypoints_[0] = {pose.x, pose.y};
    count_ = 1;
    last_timestamp_ms_ = timestamp_ms;
    has_timestamp_ = true;
    enabled_ = true;
    return true;
}

bool BreadcrumbNavigator::update(const Pose2D& pose, uint64_t timestamp_ms)
{
    if (!enabled_) return true;
    if (!validPose(pose) || (has_timestamp_ && timestamp_ms < last_timestamp_ms_)) return false;
    last_timestamp_ms_ = timestamp_ms;
    has_timestamp_ = true;
    if (count_ == 0) return false;

    float dx = pose.x - waypoints_[count_ - 1].x;
    float dy = pose.y - waypoints_[count_ - 1].y;
    float distance = std::sqrt(dx * dx + dy * dy);
    while (distance >= kWaypointSpacingM && count_ < waypoints_.size())
    {
        const float fraction = kWaypointSpacingM / distance;
        const HudPoint previous = waypoints_[count_ - 1];
        const HudPoint next{previous.x + dx * fraction, previous.y + dy * fraction};
        waypoints_[count_++] = next;
        dx = pose.x - next.x;
        dy = pose.y - next.y;
        distance = std::sqrt(dx * dx + dy * dy);
    }
    return count_ < waypoints_.size() || distance < kWaypointSpacingM;
}

void BreadcrumbNavigator::purge()
{
    enabled_ = false;
    clearWaypoints();
}

const HudPoint* BreadcrumbNavigator::returnWaypoint(std::size_t index) const
{
    if (index >= count_) return nullptr;
    return &waypoints_[count_ - index - 1];
}

float BreadcrumbNavigator::returnHeadingDeg(const Pose2D& pose) const
{
    if (!validPose(pose)) return 0.0f;
    if (count_ == 0) return pose.yaw_deg;
    const HudPoint& origin = waypoints_[0];
    return radToDeg(std::atan2(origin.x - pose.x, origin.y - pose.y));
}

} // namespace sa
