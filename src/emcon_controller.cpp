#include "emcon_controller.hpp"

#include <limits>

namespace sa {

bool EmconController::setMode(EmconMode mode)
{
    if (mode_ == mode) return false;
    mode_ = mode;
    return true;
}

void EmconController::snoozeAlerts(uint64_t timestamp_ms)
{
    constexpr uint64_t kSnoozeDurationMs = 5 * 60 * 1000;
    const uint64_t max_time = std::numeric_limits<uint64_t>::max();
    snooze_until_ms_ = timestamp_ms > max_time - kSnoozeDurationMs
                           ? max_time
                           : timestamp_ms + kSnoozeDurationMs;
    snooze_active_ = true;
}

bool EmconController::alertsSnoozed(uint64_t timestamp_ms) const
{
    return snooze_active_ && timestamp_ms < snooze_until_ms_;
}

} // namespace sa
