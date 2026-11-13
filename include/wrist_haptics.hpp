#pragma once

#include "types.hpp"

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>

namespace sa {

struct WristConfig
{
    bool enabled = true;
    std::string device_path;
    int active_value = 1;
    int inactive_value = 0;

    // Aim cone with hysteresis. The wrist vibrates while the marker bearing is
    // inside engage_deg, and keeps vibrating until it leaves release_deg.
    float engage_deg = 8.0f;
    float release_deg = 14.0f;
    float min_confidence = 0.35f;
    uint32_t max_age_ms = 250;
    float camera_yaw_offset_deg = 0.0f;
    bool verbose = false;
};

struct WristState
{
    bool marker_detected = false;
    bool aimed = false;
    float bearing_deg = 0.0f;
    float confidence = 0.0f;
};

// Pure directional logic. It decides whether a detected red marker lies close
// enough to the device's forward axis for wrist feedback. It never identifies
// the marker or the person carrying it.
class WristGuidance
{
public:
    WristGuidance();
    explicit WristGuidance(const WristConfig& config);

    WristState update(const std::optional<MarkerObservation>& marker, uint64_t now_ms);
    void reset();

    const WristConfig& config() const { return config_; }

private:
    WristConfig config_;
    bool aimed_ = false;
};

// Separate actuator output for the wrist band. With an empty device path the
// state is still tracked for telemetry and tests but no hardware is written.
class WristVibrator
{
public:
    WristVibrator();
    explicit WristVibrator(const WristConfig& config);
    ~WristVibrator();

    bool start(std::string& error);
    void stop();
    void update(uint64_t now_ms, bool aimed);

    bool running() const { return running_; }
    bool outputOn() const { return on_; }

private:
    void setOutput(bool on);

    WristConfig config_;
    std::ofstream device_;
    bool running_ = false;
    bool on_ = false;
};

} // namespace sa
