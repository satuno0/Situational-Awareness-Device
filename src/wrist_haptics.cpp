#include "wrist_haptics.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace sa {

WristGuidance::WristGuidance() : WristGuidance(WristConfig{}) {}
WristGuidance::WristGuidance(const WristConfig& config) : config_(config) {}

void WristGuidance::reset() { aimed_ = false; }

WristState WristGuidance::update(const std::optional<MarkerObservation>& marker, uint64_t now_ms)
{
    WristState state{};
    if (!config_.enabled || !marker || !marker->valid)
    {
        aimed_ = false;
        return state;
    }

    state.marker_detected = true;
    state.confidence = clampf(marker->confidence, 0.0f, 1.0f);
    state.bearing_deg = radToDeg(wrapRad(marker->bearing_rad + degToRad(config_.camera_yaw_offset_deg)));

    const uint64_t age_ms = now_ms >= marker->timestamp_ms ? now_ms - marker->timestamp_ms : 0;
    if (age_ms > config_.max_age_ms || marker->confidence < config_.min_confidence)
    {
        aimed_ = false;
        return state;
    }

    const float magnitude = std::abs(state.bearing_deg);
    aimed_ = aimed_ ? magnitude <= config_.release_deg : magnitude <= config_.engage_deg;
    state.aimed = aimed_;
    return state;
}

WristVibrator::WristVibrator() : WristVibrator(WristConfig{}) {}
WristVibrator::WristVibrator(const WristConfig& config) : config_(config) {}
WristVibrator::~WristVibrator() { stop(); }

bool WristVibrator::start(std::string& error)
{
    stop();
    if (!config_.enabled) return true;
    if (!config_.device_path.empty())
    {
        device_.open(config_.device_path, std::ios::out | std::ios::binary);
        if (!device_)
        {
            error = "cannot open wrist device " + config_.device_path;
            return false;
        }
    }
    running_ = true;
    on_ = false;
    return true;
}

void WristVibrator::setOutput(bool on)
{
    if (!device_.is_open()) return;
    device_.seekp(0);
    device_ << (on ? config_.active_value : config_.inactive_value);
    device_.flush();
}

void WristVibrator::update(uint64_t now_ms, bool aimed)
{
    (void)now_ms;
    if (!running_ || aimed == on_) return;
    setOutput(aimed);
    on_ = aimed;
    if (config_.verbose)
        std::clog << "[wrist] " << (aimed ? "marker aimed: vibrate" : "marker lost: stop") << "\n";
}

void WristVibrator::stop()
{
    if (running_)
    {
        setOutput(false);
        on_ = false;
        running_ = false;
    }
    if (device_.is_open()) device_.close();
}

} // namespace sa
