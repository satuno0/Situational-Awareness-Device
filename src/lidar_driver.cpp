#include "lidar_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <vector>

namespace sa {
namespace {

constexpr std::size_t kMaxBufferedInputBytes = 16384;
constexpr std::size_t kLd19PacketSize = 47;
constexpr float kLd19MaxRangeM = 12.0f;

std::string trim(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsv(const std::string& line)
{
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(trim(field));
    return fields;
}

bool parseFloat(const std::string& text, float& value)
{
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const float parsed = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || errno == ERANGE || *end != '\0' || !std::isfinite(parsed)) return false;
    value = parsed;
    return true;
}

uint16_t littleEndian16(const uint8_t* bytes)
{
    return static_cast<uint16_t>(bytes[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8);
}

} // namespace

LidarDriver::LidarDriver() : LidarDriver(LidarConfig{}) {}
LidarDriver::LidarDriver(const LidarConfig& config) : config_(config) {}
LidarDriver::~LidarDriver() { close(); }

bool LidarDriver::open(std::string& error)
{
    close();
    if (!config_.enabled)
    {
        error = "LiDAR driver disabled";
        return false;
    }
    if (config_.port.empty())
    {
        error = "LiDAR serial port is required";
        return false;
    }
    if (!std::isfinite(config_.mount_translation_m.x) ||
        !std::isfinite(config_.mount_translation_m.y) ||
        !std::isfinite(config_.mount_translation_m.z) ||
        !std::isfinite(config_.mount_yaw_deg))
    {
        error = "LiDAR mount pose must be finite";
        return false;
    }
    input_buffer_.clear();
    if (!serial_.open(config_.port, config_.baud_rate, error)) return false;
    if (config_.power_gate_enabled &&
        !motor_line_.open(config_.motor_enable, false, "sa-ld19-motor", error))
    {
        serial_.close();
        return false;
    }
    motor_powered_ = false;
    burst_active_ = false;
    burst_completion_pending_ = false;
    error.clear();
    return true;
}

void LidarDriver::close()
{
    if (motor_line_.isOpen())
    {
        std::string ignored;
        motor_line_.set(false, ignored);
    }
    motor_powered_ = false;
    burst_active_ = false;
    burst_completion_pending_ = false;
    motor_line_.close();
    serial_.close();
    input_buffer_.clear();
}

bool LidarDriver::triggerScanBurst(uint64_t timestamp_ms, float duration_sec)
{
    std::string ignored;
    return triggerScanBurst(timestamp_ms, duration_sec, ignored);
}

bool LidarDriver::triggerScanBurst(uint64_t timestamp_ms, float duration_sec, std::string& error)
{
    error.clear();
    if (!config_.enabled || (config_.power_gate_enabled && !motor_line_.isOpen()) ||
        (!serial_.isOpen() && config_.power_gate_enabled))
    {
        error = "LiDAR burst unavailable; driver or motor power gate is not open";
        return false;
    }
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0f || duration_sec > 60.0f)
    {
        error = "LiDAR burst duration must be in (0, 60] seconds";
        return false;
    }
    const uint64_t duration_ms = static_cast<uint64_t>(std::llround(duration_sec * 1000.0));
    if (duration_ms == 0 || timestamp_ms > std::numeric_limits<uint64_t>::max() - duration_ms)
    {
        error = "LiDAR burst duration overflows its timestamp";
        return false;
    }
    if (burst_active_ && timestamp_ms < burst_deadline_ms_) return false;
    if (!updateBurst(timestamp_ms, error)) return false;
    if (burst_active_) return false;

    if (config_.power_gate_enabled && !motor_line_.set(true, error)) return false;
    burst_start_ms_ = timestamp_ms;
    burst_deadline_ms_ = timestamp_ms + duration_ms;
    burst_active_ = true;
    motor_powered_ = true;
    input_buffer_.clear();
    return true;
}

bool LidarDriver::isBurstActive(uint64_t timestamp_ms) const
{
    return burst_active_ && (timestamp_ms < burst_deadline_ms_ || motor_powered_);
}

bool LidarDriver::updateBurst(uint64_t timestamp_ms, std::string& error)
{
    if (!burst_active_ || timestamp_ms < burst_deadline_ms_) return true;
    if (config_.power_gate_enabled && motor_line_.isOpen() && !motor_line_.set(false, error))
        return false;
    burst_active_ = false;
    motor_powered_ = false;
    burst_completion_pending_ = true;
    input_buffer_.clear();
    return true;
}

bool LidarDriver::parsePointLine(const std::string& input, Vec3& point, std::string& error)
{
    const std::string line = trim(input);
    const std::vector<std::string> fields = splitCsv(line);
    if ((fields.size() != 3 && fields.size() != 4) || fields[0] != "LIDAR")
    {
        error = "expected LIDAR,x_m,y_m[,z_m]";
        return false;
    }

    Vec3 parsed{};
    if (!parseFloat(fields[1], parsed.x) || !parseFloat(fields[2], parsed.y) ||
        (fields.size() == 4 && !parseFloat(fields[3], parsed.z)))
    {
        error = "LiDAR coordinates must be finite numbers";
        return false;
    }
    point = parsed;
    return true;
}

uint8_t LidarDriver::crc8(const uint8_t* data, std::size_t size)
{
    uint8_t crc = 0;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80u) != 0 ? static_cast<uint8_t>((crc << 1) ^ 0x4Du)
                                    : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

bool LidarDriver::parseLd19Packet(const std::array<uint8_t, 47>& packet,
                                  std::vector<Vec3>& points,
                                  std::string& error)
{
    points.clear();
    if (packet[0] != 0x54 || packet[1] != 0x2C)
    {
        error = "invalid LD19 packet header";
        return false;
    }
    if (crc8(packet.data(), packet.size() - 1) != packet.back())
    {
        error = "LD19 packet CRC8 mismatch";
        return false;
    }

    const uint16_t start_angle = littleEndian16(packet.data() + 4);
    const uint16_t end_angle = littleEndian16(packet.data() + 42);
    if (start_angle >= 36000u || end_angle >= 36000u)
    {
        error = "invalid LD19 angle";
        return false;
    }
    uint32_t angle_delta = end_angle >= start_angle
                               ? static_cast<uint32_t>(end_angle - start_angle)
                               : static_cast<uint32_t>(end_angle) + 36000u - start_angle;
    if (angle_delta > 36000u)
    {
        error = "invalid LD19 angular span";
        return false;
    }

    points.reserve(12);
    for (std::size_t i = 0; i < 12; ++i)
    {
        const uint16_t distance_mm = littleEndian16(packet.data() + 6 + i * 3);
        if (distance_mm == 0) continue;
        const float range_m = static_cast<float>(distance_mm) * 0.001f;
        if (range_m > kLd19MaxRangeM) continue;
        const float angle_deg = static_cast<float>(start_angle) +
                                static_cast<float>(angle_delta) * static_cast<float>(i) / 11.0f;
        const float angle = degToRad(angle_deg * 0.01f);
        // LD19 angle increases counter-clockwise from forward; +X is right.
        points.push_back({-std::sin(angle) * range_m, std::cos(angle) * range_m, 0.0f});
    }
    error.clear();
    return true;
}

void LidarDriver::transformPoint(Vec3& point) const
{
    point = rotateZ(point, degToRad(config_.mount_yaw_deg)) + config_.mount_translation_m;
}

bool LidarDriver::ingestBytes(const uint8_t* data, std::size_t size, uint64_t timestamp_ms,
                              LidarScan& scan, std::string& error)
{
    scan = LidarScan{};
    error.clear();
    if (!updateBurst(timestamp_ms, error)) return false;
    if (!isBurstActive(timestamp_ms))
    {
        input_buffer_.clear();
        return false;
    }
    if (timestamp_ms - burst_start_ms_ < kLidarSpinupDiscardMs)
    {
        input_buffer_.clear();
        return false;
    }
    if (size > 0 && data == nullptr)
    {
        error = "LiDAR input buffer is null";
        return false;
    }
    if (size > kMaxBufferedInputBytes || input_buffer_.size() > kMaxBufferedInputBytes - size)
    {
        input_buffer_.clear();
        error = "LiDAR input buffer exceeds the limit";
        return false;
    }
    if (size > 0) input_buffer_.insert(input_buffer_.end(), data, data + size);

    while (!input_buffer_.empty())
    {
        if (input_buffer_[0] == 0x54)
        {
            if (input_buffer_.size() < 2) break;
            if (input_buffer_[1] != 0x2C)
            {
                input_buffer_.erase(input_buffer_.begin());
                continue;
            }
            if (input_buffer_.size() < kLd19PacketSize) break;

            std::array<uint8_t, kLd19PacketSize> packet{};
            std::copy_n(input_buffer_.begin(), packet.size(), packet.begin());
            std::vector<Vec3> points;
            std::string packet_error;
            if (parseLd19Packet(packet, points, packet_error))
            {
                if (scan.points.size() + points.size() > kMaxLidarPointsPerScan)
                {
                    input_buffer_.clear();
                    error = "LiDAR poll exceeded the point limit";
                    return false;
                }
                for (Vec3& point : points)
                {
                    transformPoint(point);
                    scan.points.push_back(point);
                }
                input_buffer_.erase(input_buffer_.begin(),
                                    input_buffer_.begin() + static_cast<std::ptrdiff_t>(packet.size()));
            }
            else
            {
                input_buffer_.erase(input_buffer_.begin());
            }
            continue;
        }

        constexpr char kAsciiPrefix[] = "LIDAR,";
        const std::size_t prefix_size = sizeof(kAsciiPrefix) - 1;
        if (input_buffer_[0] == static_cast<uint8_t>(kAsciiPrefix[0]))
        {
            if (input_buffer_.size() < prefix_size &&
                std::equal(input_buffer_.begin(), input_buffer_.end(), kAsciiPrefix))
                break;
            if (input_buffer_.size() >= prefix_size &&
                std::equal(kAsciiPrefix, kAsciiPrefix + prefix_size, input_buffer_.begin()))
            {
                const auto newline = std::find(input_buffer_.begin(), input_buffer_.end(),
                                               static_cast<uint8_t>('\n'));
                if (newline == input_buffer_.end())
                {
                    if (input_buffer_.size() > 256)
                    {
                        input_buffer_.clear();
                        error = "LiDAR point line exceeds the input limit";
                        return false;
                    }
                    break;
                }
                const std::string line(input_buffer_.begin(), newline);
                Vec3 point{};
                if (!parsePointLine(line, point, error))
                {
                    input_buffer_.clear();
                    error = "invalid LiDAR record: " + error;
                    return false;
                }
                transformPoint(point);
                if (scan.points.size() >= kMaxLidarPointsPerScan)
                {
                    input_buffer_.clear();
                    error = "LiDAR poll exceeded the point limit";
                    return false;
                }
                scan.points.push_back(point);
                input_buffer_.erase(input_buffer_.begin(), newline + 1);
                continue;
            }
        }
        input_buffer_.erase(input_buffer_.begin());
    }

    if (scan.points.empty()) return false;
    scan.timestamp_ms = timestamp_ms;
    scan.origin = config_.mount_translation_m;
    scan.burst_active = burst_active_;
    return true;
}

bool LidarDriver::poll(uint64_t timestamp_ms, LidarScan& scan, std::string& error)
{
    scan = LidarScan{};
    error.clear();
    if (!serial_.isOpen())
    {
        error = "LiDAR is not open";
        return false;
    }
    if (!updateBurst(timestamp_ms, error)) return false;
    if (burst_completion_pending_)
    {
        scan.timestamp_ms = timestamp_ms;
        scan.origin = config_.mount_translation_m;
        scan.burst_complete = true;
        burst_completion_pending_ = false;
        return true;
    }
    if (!burst_active_) return false;

    std::vector<uint8_t> incoming;
    if (!serial_.readAvailable(incoming, error)) return false;
    if (!ingestBytes(incoming.data(), incoming.size(), timestamp_ms, scan, error)) return false;
    scan.burst_active = burst_active_;
    return true;
}

} // namespace sa
