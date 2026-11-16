#pragma once

#include "gpio_line.hpp"
#include "types.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace sa {

struct TofSensorConfig
{
    GpioLineConfig xshut;
    Vec3 mount_translation_m{};
    float mount_yaw_deg = 0.0f;
    float mount_pitch_deg = 0.0f;
};

struct TofConfig
{
    std::string i2c_device = "/dev/i2c-1";
    TofSensorConfig left{{"/dev/gpiochip3", 10, true}, {-0.035f, 0.0f, 0.0f}, 30.0f, 0.0f};
    TofSensorConfig right{{"/dev/gpiochip3", 11, true}, {0.035f, 0.0f, 0.0f}, -30.0f, 0.0f};
    float horizontal_fov_deg = 65.0f;
    float vertical_fov_deg = 45.0f;
    float max_range_m = 4.0f;
    uint8_t ranging_frequency_hz = 15;
    uint16_t left_i2c_address = 0x60;
    uint16_t right_i2c_address = 0x52;
    bool enabled = true;
};

class TofDriver
{
public:
    TofDriver();
    explicit TofDriver(const TofConfig& config);
    ~TofDriver();

    TofDriver(const TofDriver&) = delete;
    TofDriver& operator=(const TofDriver&) = delete;

    bool start(std::string& error);
    void stop();
    bool running() const { return running_.load(); }

    // Returns each completed sensor frame at most once. I2C transactions are
    // performed on the driver's worker, never on the fusion thread.
    bool poll(TofScan& scan, std::string& error);
    std::string lastError() const;

private:
    struct Impl;
    void workerLoop();

    TofConfig config_;
    std::unique_ptr<Impl> impl_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::optional<TofScan> latest_scan_;
    uint64_t published_sequence_ = 0;
    uint64_t consumed_sequence_ = 0;
    std::string last_error_;
};

} // namespace sa
