#include "tof_driver.hpp"

#include "vl53l8cx_api.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>
#include <vector>

#if defined(__linux__)
#include <cerrno>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace sa {
namespace {

constexpr std::size_t kMaxI2cTransfer = 256;

struct I2cContext
{
    int fd = -1;
    uint16_t address = 0x52;
};

uint8_t writeI2c(void* handle, uint16_t address, uint8_t* data, uint32_t size)
{
#if defined(__linux__)
    auto* context = static_cast<I2cContext*>(handle);
    if (context == nullptr || context->fd < 0 || data == nullptr) return 1;
    std::size_t offset = 0;
    while (offset < size)
    {
        const std::size_t count = std::min(kMaxI2cTransfer, static_cast<std::size_t>(size) - offset);
        std::array<uint8_t, kMaxI2cTransfer + 2> buffer{};
        const uint16_t reg = static_cast<uint16_t>(address + offset);
        buffer[0] = static_cast<uint8_t>(reg >> 8);
        buffer[1] = static_cast<uint8_t>(reg & 0xFFu);
        std::copy_n(data + offset, count, buffer.begin() + 2);

        i2c_msg message{};
        message.addr = static_cast<uint16_t>(context->address >> 1);
        message.flags = 0;
        message.len = static_cast<uint16_t>(count + 2);
        message.buf = buffer.data();
        i2c_rdwr_ioctl_data transaction{};
        transaction.msgs = &message;
        transaction.nmsgs = 1;
        if (ioctl(context->fd, I2C_RDWR, &transaction) != 1) return 1;
        offset += count;
    }
    return 0;
#else
    (void)handle;
    (void)address;
    (void)data;
    (void)size;
    return 1;
#endif
}

uint8_t readI2c(void* handle, uint16_t address, uint8_t* data, uint32_t size)
{
#if defined(__linux__)
    auto* context = static_cast<I2cContext*>(handle);
    if (context == nullptr || context->fd < 0 || data == nullptr) return 1;
    std::size_t offset = 0;
    while (offset < size)
    {
        const std::size_t count = std::min(kMaxI2cTransfer, static_cast<std::size_t>(size) - offset);
        const uint16_t reg = static_cast<uint16_t>(address + offset);
        uint8_t register_bytes[2] = {
            static_cast<uint8_t>(reg >> 8),
            static_cast<uint8_t>(reg & 0xFFu)};
        i2c_msg messages[2]{};
        messages[0].addr = static_cast<uint16_t>(context->address >> 1);
        messages[0].flags = 0;
        messages[0].len = 2;
        messages[0].buf = register_bytes;
        messages[1].addr = static_cast<uint16_t>(context->address >> 1);
        messages[1].flags = I2C_M_RD;
        messages[1].len = static_cast<uint16_t>(count);
        messages[1].buf = data + offset;
        i2c_rdwr_ioctl_data transaction{};
        transaction.msgs = messages;
        transaction.nmsgs = 2;
        if (ioctl(context->fd, I2C_RDWR, &transaction) != 2) return 1;
        offset += count;
    }
    return 0;
#else
    (void)handle;
    (void)address;
    (void)data;
    (void)size;
    return 1;
#endif
}

uint8_t waitMs(void*, uint32_t delay_ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    return 0;
}

bool validSensorConfig(const TofSensorConfig& sensor)
{
    return std::isfinite(sensor.mount_translation_m.x) &&
           std::isfinite(sensor.mount_translation_m.y) &&
           std::isfinite(sensor.mount_translation_m.z) &&
           std::isfinite(sensor.mount_yaw_deg) &&
           std::isfinite(sensor.mount_pitch_deg) &&
           std::abs(sensor.mount_yaw_deg) <= 180.0f &&
           std::abs(sensor.mount_pitch_deg) <= 90.0f;
}

Vec3 projectZone(const TofSensorConfig& sensor,
                 float horizontal_fov_deg,
                 float vertical_fov_deg,
                 std::size_t row,
                 std::size_t column,
                 float range_m)
{
    const float bearing = degToRad((static_cast<float>(column) - 3.5f) * horizontal_fov_deg / 8.0f);
    const float elevation = degToRad(sensor.mount_pitch_deg +
                                     (3.5f - static_cast<float>(row)) * vertical_fov_deg / 8.0f);
    const float horizontal_range = range_m * std::cos(elevation);
    const Vec3 local{
        std::sin(bearing) * horizontal_range,
        std::cos(bearing) * horizontal_range,
        std::sin(elevation) * range_m};
    return rotateZ(local, degToRad(sensor.mount_yaw_deg)) + sensor.mount_translation_m;
}

} // namespace

struct TofDriver::Impl
{
    int fd = -1;
    I2cContext left_bus;
    I2cContext right_bus;
    GpioLine left_xshut;
    GpioLine right_xshut;
    VL53L8CX_Configuration left{};
    VL53L8CX_Configuration right{};
    bool left_started = false;
    bool right_started = false;
};

TofDriver::TofDriver() : TofDriver(TofConfig{}) {}
TofDriver::TofDriver(const TofConfig& config) : config_(config), impl_(std::make_unique<Impl>()) {}
TofDriver::~TofDriver() { stop(); }

bool TofDriver::start(std::string& error)
{
    stop();
    if (!config_.enabled)
    {
        error = "ToF driver disabled";
        return false;
    }
    if (config_.i2c_device.empty() || !validSensorConfig(config_.left) || !validSensorConfig(config_.right) ||
        !std::isfinite(config_.horizontal_fov_deg) || !std::isfinite(config_.vertical_fov_deg) ||
        !std::isfinite(config_.max_range_m) || config_.horizontal_fov_deg <= 0.0f ||
        config_.horizontal_fov_deg > 120.0f || config_.vertical_fov_deg <= 0.0f ||
        config_.vertical_fov_deg > 120.0f || config_.max_range_m <= 0.0f ||
        config_.max_range_m > 4.0f || config_.ranging_frequency_hz == 0 ||
        config_.left_i2c_address == config_.right_i2c_address ||
        (config_.left_i2c_address & 1u) != 0 || (config_.right_i2c_address & 1u) != 0)
    {
        error = "invalid ToF bus, address, range, or mounting configuration";
        return false;
    }
#if defined(__linux__)
    impl_->fd = ::open(config_.i2c_device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (impl_->fd < 0)
    {
        error = "cannot open ToF I2C device " + config_.i2c_device + ": " + std::strerror(errno);
        return false;
    }
    impl_->left_bus.fd = impl_->fd;
    impl_->left_bus.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
    impl_->right_bus.fd = impl_->fd;
    impl_->right_bus.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
    if (!impl_->left_xshut.open(config_.left.xshut, false, "sa-vl53l8cx-left", error) ||
        !impl_->right_xshut.open(config_.right.xshut, false, "sa-vl53l8cx-right", error))
    {
        stop();
        return false;
    }
    running_ = true;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_.clear();
        latest_scan_.reset();
        published_sequence_ = 0;
        consumed_sequence_ = 0;
    }
    worker_ = std::thread(&TofDriver::workerLoop, this);
    error.clear();
    return true;
#else
    error = "VL53L8CX I2C hardware is supported only on Linux";
    return false;
#endif
}

void TofDriver::stop()
{
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (!impl_) return;
    if (impl_->left_xshut.isOpen())
    {
        std::string ignored;
        impl_->left_xshut.set(false, ignored);
    }
    if (impl_->right_xshut.isOpen())
    {
        std::string ignored;
        impl_->right_xshut.set(false, ignored);
    }
    impl_->left_xshut.close();
    impl_->right_xshut.close();
#if defined(__linux__)
    if (impl_->fd >= 0) ::close(impl_->fd);
#endif
    impl_->fd = -1;
    impl_->left_bus.fd = -1;
    impl_->right_bus.fd = -1;
    impl_->left_started = false;
    impl_->right_started = false;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_.reset();
    published_sequence_ = 0;
    consumed_sequence_ = 0;
}

bool TofDriver::poll(TofScan& scan, std::string& error)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_error_.empty())
    {
        error = last_error_;
        return false;
    }
    if (!latest_scan_ || consumed_sequence_ == published_sequence_) return false;
    scan = *latest_scan_;
    consumed_sequence_ = published_sequence_;
    error.clear();
    return true;
}

std::string TofDriver::lastError() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

void TofDriver::workerLoop()
{
    auto fail = [this](const std::string& message) {
        if (impl_->left_started) (void)vl53l8cx_stop_ranging(&impl_->left);
        if (impl_->right_started) (void)vl53l8cx_stop_ranging(&impl_->right);
        impl_->left_started = false;
        impl_->right_started = false;
        std::string ignored;
        (void)impl_->left_xshut.set(false, ignored);
        (void)impl_->right_xshut.set(false, ignored);
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = message;
        running_ = false;
    };

    const auto initialize = [](VL53L8CX_Configuration& sensor, I2cContext& bus,
                               std::string& error) {
        sensor.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
        sensor.platform.Write = writeI2c;
        sensor.platform.Read = readI2c;
        sensor.platform.Wait = waitMs;
        bus.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
        sensor.platform.handle = &bus;
        if (vl53l8cx_init(&sensor) != VL53L8CX_STATUS_OK)
        {
            error = "VL53L8CX initialization failed";
            return false;
        }
        return true;
    };
    const auto startRanging = [this](VL53L8CX_Configuration& sensor, std::string& error) {
        if (vl53l8cx_set_resolution(&sensor, VL53L8CX_RESOLUTION_8X8) != VL53L8CX_STATUS_OK ||
            vl53l8cx_set_ranging_frequency_hz(&sensor, config_.ranging_frequency_hz) !=
                VL53L8CX_STATUS_OK ||
            vl53l8cx_set_ranging_mode(&sensor, VL53L8CX_RANGING_MODE_CONTINUOUS) !=
                VL53L8CX_STATUS_OK ||
            vl53l8cx_start_ranging(&sensor) != VL53L8CX_STATUS_OK)
        {
            error = "VL53L8CX ranging configuration failed";
            return false;
        }
        return true;
    };
    const auto setAddress = [](VL53L8CX_Configuration& sensor, I2cContext& bus,
                               uint16_t address, std::string& error) {
        if (bus.address != address && vl53l8cx_set_i2c_address(&sensor, address) != VL53L8CX_STATUS_OK)
        {
            error = "VL53L8CX address assignment failed";
            return false;
        }
        bus.address = address;
        return true;
    };

    uint16_t temporary_address = 0;
    for (uint16_t address = 0x54; address <= 0x76; address = static_cast<uint16_t>(address + 2))
    {
        if (address != config_.left_i2c_address && address != config_.right_i2c_address)
        {
            temporary_address = address;
            break;
        }
    }
    if (temporary_address == 0)
    {
        fail("no temporary VL53L8CX address is available for XSHUT sequencing");
        return;
    }

    std::string error;
    if (!impl_->left_xshut.set(true, error))
    {
        fail("left VL53L8CX XSHUT failed: " + error);
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!initialize(impl_->left, impl_->left_bus, error) ||
        !setAddress(impl_->left, impl_->left_bus, temporary_address, error) ||
        !startRanging(impl_->left, error))
    {
        fail(error);
        return;
    }
    impl_->left_started = true;
    if (!impl_->right_xshut.set(true, error))
    {
        fail("right VL53L8CX XSHUT failed: " + error);
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!initialize(impl_->right, impl_->right_bus, error) ||
        !setAddress(impl_->right, impl_->right_bus, config_.right_i2c_address, error) ||
        !startRanging(impl_->right, error))
    {
        fail(error);
        return;
    }
    impl_->right_started = true;
    if (vl53l8cx_stop_ranging(&impl_->left) != VL53L8CX_STATUS_OK)
    {
        fail("left VL53L8CX could not stop for final address assignment");
        return;
    }
    impl_->left_started = false;
    if (!setAddress(impl_->left, impl_->left_bus, config_.left_i2c_address, error) ||
        vl53l8cx_start_ranging(&impl_->left) != VL53L8CX_STATUS_OK)
    {
        fail(error.empty() ? "left VL53L8CX restart failed" : error);
        return;
    }
    impl_->left_started = true;

    while (running_.load())
    {
        TofScan scan{};
        bool have_new_frame = false;
        VL53L8CX_ResultsData results{};
        uint8_t ready = 0;
        if (vl53l8cx_check_data_ready(&impl_->left, &ready) == VL53L8CX_STATUS_OK && ready != 0)
        {
            if (vl53l8cx_get_ranging_data(&impl_->left, &results) != VL53L8CX_STATUS_OK)
            {
                fail("left VL53L8CX ranging read failed");
                break;
            }
            have_new_frame = true;
            for (std::size_t row = 0; row < 8; ++row)
            {
                for (std::size_t column = 0; column < 8; ++column)
                {
                    const std::size_t index = row * 8 + column;
                    if (results.target_status[index] != 5 && results.target_status[index] != 9) continue;
                    const int16_t distance_mm = results.distance_mm[index];
                    if (distance_mm <= 0) continue;
                    const float range_m = static_cast<float>(distance_mm) * 0.001f;
                    if (range_m > config_.max_range_m) continue;
                    scan.rays.push_back({config_.left.mount_translation_m,
                                         projectZone(config_.left, config_.horizontal_fov_deg,
                                                     config_.vertical_fov_deg, row, column, range_m)});
                }
            }
        }

        ready = 0;
        if (vl53l8cx_check_data_ready(&impl_->right, &ready) == VL53L8CX_STATUS_OK && ready != 0)
        {
            if (vl53l8cx_get_ranging_data(&impl_->right, &results) != VL53L8CX_STATUS_OK)
            {
                fail("right VL53L8CX ranging read failed");
                break;
            }
            have_new_frame = true;
            for (std::size_t row = 0; row < 8; ++row)
            {
                for (std::size_t column = 0; column < 8; ++column)
                {
                    const std::size_t index = row * 8 + column;
                    if (results.target_status[index] != 5 && results.target_status[index] != 9) continue;
                    const int16_t distance_mm = results.distance_mm[index];
                    if (distance_mm <= 0) continue;
                    const float range_m = static_cast<float>(distance_mm) * 0.001f;
                    if (range_m > config_.max_range_m) continue;
                    scan.rays.push_back({config_.right.mount_translation_m,
                                         projectZone(config_.right, config_.horizontal_fov_deg,
                                                     config_.vertical_fov_deg, row, column, range_m)});
                }
            }
        }

        if (have_new_frame)
        {
            scan.timestamp_ms = monoNowMs();
            std::lock_guard<std::mutex> lock(mutex_);
            latest_scan_ = std::move(scan);
            ++published_sequence_;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (impl_->left_started) (void)vl53l8cx_stop_ranging(&impl_->left);
    if (impl_->right_started) (void)vl53l8cx_stop_ranging(&impl_->right);
    impl_->left_started = false;
    impl_->right_started = false;
}

} // namespace sa
