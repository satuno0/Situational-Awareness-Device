#pragma once

#include "gpio_line.hpp"
#include "serial_port.hpp"
#include "types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sa {

constexpr uint64_t kLidarSpinupDiscardMs = 300;
constexpr float kDefaultLidarBurstDurationSec = 1.5f;

struct LidarConfig
{
    std::string port;
    int baud_rate = 230400;
    Vec3 mount_translation_m{0.0f, 0.0f, 0.0f};
    float mount_yaw_deg = 0.0f;
    GpioLineConfig motor_enable{"/dev/gpiochip3", 12, true};
    bool power_gate_enabled = true;
    bool enabled = true;
};

class LidarDriver
{
public:
    LidarDriver();
    explicit LidarDriver(const LidarConfig& config);
    ~LidarDriver();

    LidarDriver(const LidarDriver&) = delete;
    LidarDriver& operator=(const LidarDriver&) = delete;

    bool open(std::string& error);
    void close();
    bool isOpen() const { return serial_.isOpen(); }

    bool triggerScanBurst(uint64_t timestamp_ms, float duration_sec = kDefaultLidarBurstDurationSec);
    bool triggerScanBurst(uint64_t timestamp_ms, float duration_sec, std::string& error);
    bool isBurstActive(uint64_t timestamp_ms) const;
    bool motorPowered() const { return motor_powered_; }

    // Collects complete ASCII bridge points or native LD19 packets for one tick.
    bool poll(uint64_t timestamp_ms, LidarScan& scan, std::string& error);

    // Accepts bytes from a deterministic simulation or replay fixture.
    bool ingestBytes(const uint8_t* data, std::size_t size, uint64_t timestamp_ms,
                     LidarScan& scan, std::string& error);

    static bool parsePointLine(const std::string& line, Vec3& point, std::string& error);
    static uint8_t crc8(const uint8_t* data, std::size_t size);
    static bool parseLd19Packet(const std::array<uint8_t, 47>& packet,
                                std::vector<Vec3>& points,
                                std::string& error);

private:
    bool updateBurst(uint64_t timestamp_ms, std::string& error);
    void transformPoint(Vec3& point) const;

    LidarConfig config_;
    SerialPort serial_;
    GpioLine motor_line_;
    std::vector<uint8_t> input_buffer_;
    uint64_t burst_start_ms_ = 0;
    uint64_t burst_deadline_ms_ = 0;
    bool burst_active_ = false;
    bool burst_completion_pending_ = false;
    bool motor_powered_ = false;
};

} // namespace sa
