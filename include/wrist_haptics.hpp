#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <cstdint>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

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
    std::string module_port;
    int module_baud_rate = 115200;
    float wrist_yaw_offset_deg = 0.0f;
    uint32_t orientation_timeout_ms = 250;
    float pointing_cone_deg = 15.0f;
    bool verbose = false;
};

enum class WristHapticPattern
{
    SILENT,
    COLLISION,
    UNKNOWN_PERSON,
    SAFE_TAGGED,
    PRESENCE_ONLY
};

struct WristHapticState
{
    bool orientation_valid = false;
    bool pointing = false;
    float wrist_yaw_deg = 0.0f;
    float wrist_pitch_deg = 0.0f;
    float target_bearing_deg = 0.0f;
    float angular_error_deg = 0.0f;
    float duty_pct = 0.0f;
    float frequency_hz = 0.0f;
    float target_range_m = 0.0f;
    WristHapticPattern pattern = WristHapticPattern::SILENT;
};

const char* wristHapticPatternName(WristHapticPattern pattern);

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
    void outputLoop();

    WristConfig config_;
    std::ofstream device_;
    std::mutex output_mutex_;
    std::condition_variable output_ready_;
    std::thread output_thread_;
    bool pending_output_ = false;
    bool has_pending_output_ = false;
    bool stop_output_ = false;
    bool running_ = false;
    bool on_ = false;
};

// Reads WRIST,yaw_deg,pitch_deg packets and asynchronously writes HAPTIC
// command frames. update() is deterministic and safe to exercise without I/O.
class WristHaptics
{
public:
    WristHaptics();
    explicit WristHaptics(const WristConfig& config);
    ~WristHaptics();

    bool start(std::string& error);
    void stop();
    bool poll(uint64_t timestamp_ms, WristOrientation& orientation, std::string& error);
    WristHapticState update(const WorldModel& world,
                            const std::optional<WristOrientation>& orientation,
                            uint64_t timestamp_ms,
                            bool collision_warning = false);
    void reset();

    static bool parseOrientationLine(const std::string& line,
                                     float& yaw_deg,
                                     float& pitch_deg,
                                     std::string& error);

private:
    void queueOutput(const WristHapticState& state);
    void outputLoop();

    WristConfig config_;
    SerialPort module_;
    std::string line_buffer_;
    std::optional<WristOrientation> latest_orientation_;
    bool calibrated_ = false;
    float calibrated_yaw_offset_deg_ = 0.0f;
    WristHapticState last_state_{};

    std::mutex output_mutex_;
    std::condition_variable output_ready_;
    std::thread output_thread_;
    WristHapticState pending_output_{};
    bool has_pending_output_ = false;
    bool stop_output_ = false;
    bool running_ = false;
};

} // namespace sa
