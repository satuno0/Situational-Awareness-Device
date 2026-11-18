#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sa {

namespace hal_pinout {

inline constexpr std::string_view i2c_device = "/dev/i2c-1";
inline constexpr uint32_t i2c_frequency_hz = 400000;
inline constexpr uint8_t tmp117_address = 0x48;
inline constexpr uint8_t icm42688_address = 0x68;
inline constexpr uint8_t tof_s3_default_address = 0x29;
inline constexpr uint8_t tof_s3_address = 0x30;
inline constexpr uint8_t tof_s4_address = 0x29;

inline constexpr uint32_t gpio_sda = 2;
inline constexpr uint32_t gpio_scl = 3;
inline constexpr uint32_t gpio_tof_s4_xshut = 11;
inline constexpr uint32_t gpio_sw1_auto_manual = 13;
inline constexpr uint32_t gpio_radar_power_gate = 15;
inline constexpr uint32_t gpio_button1 = 16;
inline constexpr uint32_t gpio_lid_reed = 18;

inline constexpr std::string_view xiao_usb_device = "/dev/ttyACM0";
inline constexpr uint32_t xiao_baud = 115200;
inline constexpr uint32_t xiao_uart_gpio_07 = 7;
inline constexpr uint32_t xiao_uart_gpio_08 = 8;
inline constexpr uint8_t haptic_packet_magic = 0xA5;
inline constexpr std::size_t haptic_packet_size = 6;
inline constexpr uint8_t spoke_packet_magic = 0x5A;
inline constexpr std::size_t spoke_packet_size = 12;

} // namespace hal_pinout

struct LidInterlockCallbacks
{
    void* context = nullptr;
    void (*zeroize_ram_buffers)(void*) noexcept = nullptr;
    void (*drop_luks_keys)(void*) noexcept = nullptr;
};

struct EmconHardwareConfig
{
    std::string gpio_chip = "/dev/gpiochip0";
    uint32_t input_debounce_ms = 25;
    uint32_t double_click_ms = 500;
    uint32_t manual_sweep_ms = 500;
    uint32_t advisory_snooze_ms = 5 * 60 * 1000;
};

struct EmconHardwareStatus
{
    bool auto_mode = false;
    bool radar_powered = false;
    bool button_pressed = false;
    bool lid_open = false;
    bool advisory_snoozed = false;
};

class EmconHardware
{
public:
    EmconHardware();
    ~EmconHardware();

    EmconHardware(const EmconHardware&) = delete;
    EmconHardware& operator=(const EmconHardware&) = delete;

    bool start(const EmconHardwareConfig& config, const LidInterlockCallbacks& lid_callbacks,
               uint64_t timestamp_ms, std::string& error);
    bool poll(uint64_t timestamp_ms, EmconHardwareStatus& status, std::string& error);
    void stop();
    bool running() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

using TofAddressSetter = bool (*)(void*, uint8_t current_address, uint8_t new_address,
                                  std::string& error) noexcept;

class FrontTofPodControl
{
public:
    FrontTofPodControl();
    ~FrontTofPodControl();

    FrontTofPodControl(const FrontTofPodControl&) = delete;
    FrontTofPodControl& operator=(const FrontTofPodControl&) = delete;

    bool start(std::string_view gpio_chip, TofAddressSetter set_s3_address,
               void* context, std::string& error);
    void stop();
    bool running() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct I2cBusConfig
{
    std::string device = std::string(hal_pinout::i2c_device);
    uint32_t timeout_ms = 25;
    uint32_t reconnect_delay_ms = 250;
    std::size_t max_queued_transactions = 64;
};

struct I2cResult
{
    uint64_t transaction_id = 0;
    bool success = false;
    std::vector<uint8_t> read_data;
    std::string error;
};

class AsyncI2cBus
{
public:
    AsyncI2cBus();
    ~AsyncI2cBus();

    AsyncI2cBus(const AsyncI2cBus&) = delete;
    AsyncI2cBus& operator=(const AsyncI2cBus&) = delete;

    bool start(const I2cBusConfig& config, std::string& error);
    void stop();
    bool submit(uint8_t address, std::span<const uint8_t> write_data,
                std::size_t read_size, uint64_t& transaction_id, std::string& error);
    bool tryTakeResult(uint64_t transaction_id, I2cResult& result);
    bool running() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sa
