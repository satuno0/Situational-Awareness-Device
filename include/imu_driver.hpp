#pragma once

#include "types.hpp"

#include <string>

namespace sa {

struct ImuConfig
{
    std::string i2c_device = "/dev/i2c-1";
    int i2c_address = 0x68;
    Vec3 gyro_bias_radps{0.0f, 0.0f, 0.0f};
    float gyro_temp_coeff_radps_per_c = 0.0f;
    bool enabled = true;
};

class ImuDriver
{
public:
    ImuDriver();
    explicit ImuDriver(const ImuConfig& config);
    ~ImuDriver();

    bool open(std::string& error);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Reads the most recent sample from the sensor. Non-blocking in practice:
    // the caller controls the poll rate.
    bool read(ImuSample& sample, std::string& error);
    bool healthy() const { return fd_ >= 0 && last_read_ok_; }
    const std::string& lastError() const { return last_error_; }

private:
    bool configure(std::string& error);
    bool readRegisters(uint8_t start_register, uint8_t* out, std::size_t count, std::string& error);
    bool writeRegister(uint8_t reg, uint8_t value, std::string& error);

    ImuConfig config_;
    int fd_ = -1;
    bool last_read_ok_ = false;
    std::string last_error_;
};

// Small Mahony/complementary AHRS. Yaw is intentionally not claimed to be
// absolute: without a magnetometer it is gyro-integrated and is corrected by
// the radar scan matcher at the SLAM layer.
class AttitudeEstimator
{
public:
    AttitudeEstimator();

    void reset();
    Quaternion update(const ImuSample& sample, float dt);
    void correctYaw(float yaw_rad, float weight);

    const Quaternion& orientation() const { return orientation_; }
    float yawRad() const;
    bool stationary() const { return stationary_; }
    const Vec3& gyroBias() const { return gyro_bias_; }

private:
    Quaternion orientation_{1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 gyro_bias_{0.0f, 0.0f, 0.0f};
    bool stationary_ = false;
    float stationary_time_s_ = 0.0f;
    float gravity_lowpass_ = kGravity;
};

} // namespace sa
