#include "imu_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <thread>

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

constexpr uint8_t kRegDeviceConfig = 0x11;
constexpr uint8_t kRegTempData1 = 0x1D;
constexpr uint8_t kRegPwrMgmt0 = 0x4E;
constexpr uint8_t kRegGyroConfig0 = 0x4F;
constexpr uint8_t kRegAccelConfig0 = 0x50;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kWhoAmIValue = 0x47;

int16_t be16(const uint8_t* p)
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

} // namespace

ImuDriver::ImuDriver() : ImuDriver(ImuConfig{}) {}
ImuDriver::ImuDriver(const ImuConfig& config) : config_(config) {}

ImuDriver::~ImuDriver() { close(); }

bool ImuDriver::open(std::string& error)
{
    close();
    last_read_ok_ = false;
    if (!config_.enabled)
    {
        error = "IMU driver disabled";
        return false;
    }
#if defined(__linux__)
    fd_ = ::open(config_.i2c_device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0)
    {
        error = "failed to open " + config_.i2c_device + ": " + std::strerror(errno);
        last_error_ = error;
        return false;
    }
    if (ioctl(fd_, I2C_SLAVE, config_.i2c_address) < 0)
    {
        error = "I2C_SLAVE failed for address " + std::to_string(config_.i2c_address) + ": " + std::strerror(errno);
        close();
        last_error_ = error;
        return false;
    }
    if (!configure(error))
    {
        close();
        last_error_ = error;
        return false;
    }
    return true;
#else
    error = "ICM-42688 I2C hardware is only supported on Linux";
    last_error_ = error;
    return false;
#endif
}

void ImuDriver::close()
{
#if defined(__linux__)
    if (fd_ >= 0) ::close(fd_);
#endif
    fd_ = -1;
}

bool ImuDriver::writeRegister(uint8_t reg, uint8_t value, std::string& error)
{
#if defined(__linux__)
    uint8_t buffer[2] = {reg, value};
    if (::write(fd_, buffer, 2) != 2)
    {
        error = "ICM-42688 register write failed: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
#else
    (void)reg;
    (void)value;
    error = "no I2C backend";
    return false;
#endif
}

bool ImuDriver::readRegisters(uint8_t start_register, uint8_t* out, std::size_t count, std::string& error)
{
#if defined(__linux__)
    i2c_msg messages[2]{};
    messages[0].addr = static_cast<uint16_t>(config_.i2c_address);
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &start_register;
    messages[1].addr = static_cast<uint16_t>(config_.i2c_address);
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<uint16_t>(count);
    messages[1].buf = out;
    i2c_rdwr_ioctl_data transaction{};
    transaction.msgs = messages;
    transaction.nmsgs = 2;
    if (ioctl(fd_, I2C_RDWR, &transaction) < 0)
    {
        error = "ICM-42688 burst read failed: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
#else
    (void)start_register;
    (void)out;
    (void)count;
    error = "no I2C backend";
    return false;
#endif
}

bool ImuDriver::configure(std::string& error)
{
    uint8_t who = 0;
    if (!readRegisters(kRegWhoAmI, &who, 1, error)) return false;
    if (who != kWhoAmIValue)
    {
        error = "ICM-42688 WHO_AM_I mismatch: got " + std::to_string(who);
        return false;
    }

    if (!writeRegister(kRegDeviceConfig, 0x01, error)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!readRegisters(kRegWhoAmI, &who, 1, error)) return false;

    // Low-noise accel + gyro, 1 kHz ODR, +/-16 g, +/-2000 dps.
    if (!writeRegister(kRegPwrMgmt0, 0x0F, error)) return false;
    if (!writeRegister(kRegGyroConfig0, 0x06, error)) return false;
    if (!writeRegister(kRegAccelConfig0, 0x66, error)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return true;
}

bool ImuDriver::read(ImuSample& sample, std::string& error)
{
    if (fd_ < 0)
    {
        error = "IMU is not open";
        last_read_ok_ = false;
        return false;
    }
    uint8_t raw[14]{};
    if (!readRegisters(kRegTempData1, raw, sizeof(raw), error))
    {
        last_read_ok_ = false;
        last_error_ = error;
        return false;
    }

    const int16_t temp_raw = be16(raw + 0);
    const int16_t ax = be16(raw + 2);
    const int16_t ay = be16(raw + 4);
    const int16_t az = be16(raw + 6);
    const int16_t gx = be16(raw + 8);
    const int16_t gy = be16(raw + 10);
    const int16_t gz = be16(raw + 12);

    const float temperature = static_cast<float>(temp_raw) / 132.48f + 25.0f;
    constexpr float accel_scale = kGravity / 2048.0f; // +/-16 g
    constexpr float gyro_scale = kDegToRad / 16.4f;  // +/-2000 dps

    sample.timestamp_ms = monoNowMs();
    sample.temperature_c = temperature;
    sample.accel_mps2 = {
        static_cast<float>(ax) * accel_scale,
        static_cast<float>(ay) * accel_scale,
        static_cast<float>(az) * accel_scale};
    Vec3 gyro{
        static_cast<float>(gx) * gyro_scale,
        static_cast<float>(gy) * gyro_scale,
        static_cast<float>(gz) * gyro_scale};
    gyro -= config_.gyro_bias_radps;
    gyro -= Vec3{config_.gyro_temp_coeff_radps_per_c, config_.gyro_temp_coeff_radps_per_c,
                 config_.gyro_temp_coeff_radps_per_c} *
            (temperature - 25.0f);
    sample.gyro_radps = gyro;
    last_read_ok_ = true;
    last_error_.clear();
    return true;
}

AttitudeEstimator::AttitudeEstimator() { reset(); }

void AttitudeEstimator::reset()
{
    orientation_ = {1.0f, 0.0f, 0.0f, 0.0f};
    gyro_bias_ = {0.0f, 0.0f, 0.0f};
    stationary_ = false;
    stationary_time_s_ = 0.0f;
    gravity_lowpass_ = kGravity;
}

Quaternion AttitudeEstimator::update(const ImuSample& sample, float dt)
{
    if (!(dt > 0.0f)) return orientation_;
    dt = std::min(dt, 0.2f);

    const float accel_norm = norm(sample.accel_mps2);
    const float gyro_norm = norm(sample.gyro_radps);
    gravity_lowpass_ = 0.98f * gravity_lowpass_ + 0.02f * accel_norm;

    const bool quiet = std::abs(accel_norm - kGravity) < 0.28f && gyro_norm < 0.09f;
    if (quiet)
    {
        stationary_time_s_ += dt;
        if (stationary_time_s_ > 0.30f) stationary_ = true;
    }
    else
    {
        stationary_time_s_ = 0.0f;
        stationary_ = false;
    }

    Vec3 gyro = sample.gyro_radps;
    if (stationary_)
    {
        const float alpha = 0.02f;
        gyro_bias_ = gyro_bias_ * (1.0f - alpha) + gyro * alpha;
        if (norm(gyro_bias_) > 0.20f) gyro_bias_ = normalized(gyro_bias_) * 0.20f;
    }
    gyro -= gyro_bias_;

    if (accel_norm > 1.0f && accel_norm < 25.0f)
    {
        const Vec3 measured_up = sample.accel_mps2 / accel_norm;
        const Vec3 expected_up = quatRotate(quatConjugate(orientation_), {0.0f, 0.0f, 1.0f});
        const Vec3 error = cross(measured_up, expected_up);
        const float kp = stationary_ ? 1.8f : 0.55f;
        gyro += error * kp;
    }

    orientation_ = quatIntegrateBodyRate(orientation_, gyro, dt);
    return orientation_;
}

void AttitudeEstimator::correctYaw(float yaw_rad, float weight)
{
    if (!(weight > 0.0f)) return;
    const Vec3 euler = quatToEulerDeg(orientation_);
    const float current = degToRad(euler.z);
    const float delta = wrapRad(yaw_rad - current) * clampf(weight, 0.0f, 1.0f);
    orientation_ = quatNormalize(quatMultiply(quatFromEulerDeg(0.0f, 0.0f, radToDeg(delta)), orientation_));
}

float AttitudeEstimator::yawRad() const
{
    return degToRad(quatToEulerDeg(orientation_).z);
}

} // namespace sa
