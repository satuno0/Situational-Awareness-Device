#pragma once

#include "types.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

namespace sa {

enum class HapticPattern
{
    NONE,
    COLLISION,
    SAFE_TAGGED,
    UNKNOWN_PERSON,
    LOST_TRACK,
    PRESENCE_BARRIER
};

struct HapticConfig
{
    bool enabled = true;
    bool verbose = false;
    std::string device_path;
    int active_value = 1;
    int inactive_value = 0;
};

const char* hapticPatternName(HapticPattern pattern);

class Haptics
{
public:
    Haptics();
    explicit Haptics(const HapticConfig& config);
    ~Haptics();

    bool start(std::string& error);
    void stop();
    bool running() const { return running_.load(); }

    void update(const WorldModel& world, const UltrasonicMeasurement* ultrasonic = nullptr);
    void trigger(HapticPattern pattern);
    HapticPattern activePattern() const;

private:
    void workerLoop();
    void setMotor(bool on);
    void resetTracking();

    HapticConfig config_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::thread worker_;
    std::atomic<bool> running_{false};
    bool stop_ = false;

    std::deque<HapticPattern> event_queue_;
    std::optional<float> collision_range_;
    bool presence_active_ = false;
    HapticPattern active_pattern_ = HapticPattern::NONE;

    std::unordered_map<uint32_t, bool> previous_visible_;
    std::unordered_map<uint32_t, bool> previous_tagged_;
    bool previous_presence_ = false;

    std::ofstream device_;
};

} // namespace sa
