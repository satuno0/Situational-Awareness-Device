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
    std::string drv2605_i2c_device;
    uint8_t drv2605_i2c_address = 0x5A;
    int active_value = 1;
    int inactive_value = 0;
};

const char* hapticPatternName(HapticPattern pattern);
std::optional<float> closeRangeCollisionRange(const WorldModel& world,
                                              const UltrasonicMeasurement* ultrasonic);

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
    bool setMotor(bool on);
    bool writeDrv2605(uint8_t reg, uint8_t value, std::string& error);
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
    int drv2605_fd_ = -1;
    bool drv2605_active_ = false;
};

} // namespace sa
