#include "radar_driver.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <thread>

namespace sa {
namespace {

constexpr uint8_t kMagic[8] = {0x01, 0x02, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
constexpr std::size_t kHeaderSize = 40;

uint32_t le32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t le16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1] << 8);
}

float lef32(const uint8_t* p)
{
    const uint32_t bits = le32(p);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::string trim(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

} // namespace

RadarDriver::RadarDriver() : RadarDriver(RadarConfig{}) {}
RadarDriver::RadarDriver(const RadarConfig& config) : config_(config) {}
RadarDriver::~RadarDriver() { close(); }

void RadarDriver::close()
{
    data_.close();
    buffer_.clear();
    frames_.clear();
    healthy_ = false;
}

bool RadarDriver::open(std::string& error)
{
    close();
    if (!config_.enabled)
    {
        error = "radar driver disabled";
        return false;
    }
    if (config_.data_port.empty())
    {
        error = "radar data port is required";
        return false;
    }
    if (config_.configure_on_open && !config_.config_file.empty())
    {
        if (!configure(error)) return false;
    }
    if (!data_.open(config_.data_port, config_.data_baud, error)) return false;
    return true;
}

bool RadarDriver::configure(std::string& error)
{
    if (config_.cli_port.empty())
    {
        error = "radar CLI port is required to send a .cfg file";
        return false;
    }
    std::ifstream file(config_.config_file);
    if (!file)
    {
        error = "cannot open radar config file " + config_.config_file;
        return false;
    }
    SerialPort cli;
    if (!cli.open(config_.cli_port, 115200, error)) return false;

    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == '%') continue;
        if (!cli.writeAll(line + "\n", error)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(line.rfind("sensorStart", 0) == 0 ? 100 : 8));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

bool RadarDriver::readAndParse(std::string& error)
{
    std::vector<uint8_t> incoming;
    if (!data_.readAvailable(incoming, error)) return false;
    if (!incoming.empty()) buffer_.insert(buffer_.end(), incoming.begin(), incoming.end());

    if (buffer_.size() > 8u * 1024u * 1024u) buffer_.erase(buffer_.begin(), buffer_.begin() + 4u * 1024u * 1024u);

    while (true)
    {
        auto it = std::search(buffer_.begin(), buffer_.end(), std::begin(kMagic), std::end(kMagic));
        if (it == buffer_.end())
        {
            if (buffer_.size() > 7) buffer_.erase(buffer_.begin(), buffer_.end() - 7);
            return true;
        }
        const std::size_t offset = static_cast<std::size_t>(std::distance(buffer_.begin(), it));
        if (offset > 0) buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
        if (buffer_.size() < kHeaderSize) return true;

        const uint32_t total_length = le32(buffer_.data() + 12);
        if (total_length < kHeaderSize || total_length > 2u * 1024u * 1024u)
        {
            buffer_.erase(buffer_.begin());
            continue;
        }
        if (buffer_.size() < total_length) return true;

        RadarFrame frame;
        if (parsePacket(buffer_.data(), total_length, frame))
        {
            frames_.push_back(std::move(frame));
            while (frames_.size() > 8) frames_.pop_front();
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
    }
}

bool RadarDriver::parsePacket(const uint8_t* packet, std::size_t size, RadarFrame& frame)
{
    if (size < kHeaderSize || std::memcmp(packet, kMagic, sizeof(kMagic)) != 0) return false;
    const uint32_t detected = le32(packet + 28);
    const uint32_t num_tlvs = le32(packet + 32);
    if (num_tlvs > 1024) return false;

    struct Point
    {
        float x, y, z, doppler;
    };
    std::vector<Point> points;
    std::vector<float> snr;

    std::size_t offset = kHeaderSize;
    for (uint32_t tlv = 0; tlv < num_tlvs; ++tlv)
    {
        if (offset + 8 > size) break;
        const uint32_t type = le32(packet + offset);
        const uint32_t length = le32(packet + offset + 4);
        offset += 8;
        if (offset + length > size) break;

        if (type == 1)
        {
            const std::size_t count = length / 16u;
            points.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                const uint8_t* p = packet + offset + i * 16u;
                points.push_back(Point{lef32(p), lef32(p + 4), lef32(p + 8), lef32(p + 12)});
            }
        }
        else if (type == 7)
        {
            const std::size_t count = length / 4u;
            snr.reserve(count);
            for (std::size_t i = 0; i < count; ++i) snr.push_back(static_cast<float>(le16(packet + offset + i * 4u)));
        }
        offset += length;
    }

    if (points.empty() && detected > 0) return false;
    frame.timestamp_ms = monoNowMs();
    frame.detections.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const Point& p = points[i];
        Detection detection{};
        detection.position = {p.x, p.y, p.z};
        const float distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        const Vec3 radial = distance > 1.0e-5f ? Vec3{p.x / distance, p.y / distance, p.z / distance}
                                               : Vec3{0.0f, 0.0f, 0.0f};
        detection.velocity = radial * p.doppler;
        detection.range = distance;
        detection.snr = i < snr.size() ? snr[i] : 10.0f;
        detection.from_radar = true;
        frame.detections.push_back(detection);
    }
    transformToBody(frame);
    return true;
}

void RadarDriver::transformToBody(RadarFrame& frame) const
{
    const float yaw = degToRad(config_.mount_yaw_deg);
    for (auto& detection : frame.detections)
    {
        detection.position = rotateZ(detection.position, yaw) + config_.mount_translation_m;
        detection.velocity = rotateZ(detection.velocity, yaw);
        detection.range = norm(detection.position);
    }
}

bool RadarDriver::poll(RadarFrame& frame)
{
    if (!data_.isOpen()) return false;
    std::string error;
    if (!readAndParse(error))
    {
        healthy_ = false;
        return false;
    }
    if (frames_.empty()) return false;
    frame = std::move(frames_.front());
    frames_.pop_front();
    healthy_ = true;
    return true;
}

} // namespace sa
