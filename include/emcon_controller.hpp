#pragma once

#include <cstdint>

namespace sa {

enum class EmconMode
{
    Manual,
    Auto
};

class EmconController
{
public:
    EmconMode mode() const { return mode_; }
    bool setMode(EmconMode mode);
    bool radarEnabled() const { return mode_ == EmconMode::Auto; }

    void snoozeAlerts(uint64_t timestamp_ms);
    bool alertsSnoozed(uint64_t timestamp_ms) const;

private:
    EmconMode mode_ = EmconMode::Manual;
    uint64_t snooze_until_ms_ = 0;
    bool snooze_active_ = false;
};

} // namespace sa
