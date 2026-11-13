#pragma once

#include "types.hpp"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <string>

namespace sa {

struct AnnounceConfig
{
    bool enabled = false;
    // Line-oriented speaker/earpiece bridge. Each announcement is written as
    // "SAY <text>\n". An empty path keeps the logic active for telemetry and
    // tests but writes no hardware.
    std::string device_path;

    float min_existence = 0.35f;
    float max_range_m = 15.0f;
    float min_report_range_m = 0.30f;
    std::size_t max_entries = 2;

    double min_interval_s = 1.5;
    double repeat_interval_s = 8.0;
    bool verbose = false;
};

// Builds a short, deterministic sentence describing how many people the world
// model currently contains, what they are classified as, and roughly where they
// are relative to the device. Red markers and map objects are never described
// as people.
std::string buildAnnouncement(const WorldModel& world, const AnnounceConfig& config);

class Announcer
{
public:
    Announcer();
    explicit Announcer(const AnnounceConfig& config);
    ~Announcer();

    bool start(std::string& error);
    void stop();
    bool enabled() const { return config_.enabled; }
    bool running() const { return running_; }

    // Emits at most one line, with a short debounce for changes and a longer
    // repeat interval for an unchanged scene.
    void update(const WorldModel& world, uint64_t now_ms);

    const std::string& lastText() const { return last_text_; }

private:
    void emit(const std::string& text, uint64_t now_ms);

    AnnounceConfig config_;
    std::ofstream device_;
    bool running_ = false;
    std::optional<uint64_t> last_emit_ms_;
    std::string last_text_;
};

} // namespace sa
