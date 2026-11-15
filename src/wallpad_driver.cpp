#include "wallpad_driver.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace sa {
namespace {

constexpr std::size_t kMaxLineBufferBytes = 16384;

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
    if (end == text.c_str() || errno == ERANGE || *end != '\0' || !std::isfinite(parsed))
        return false;
    value = parsed;
    return true;
}

} // namespace

WallpadDriver::WallpadDriver() : WallpadDriver(WallpadConfig{}) {}
WallpadDriver::WallpadDriver(const WallpadConfig& config) : config_(config) {}
WallpadDriver::~WallpadDriver() { close(); }

bool WallpadDriver::open(std::string& error)
{
    close();
    if (!config_.enabled)
    {
        error = "wall-pad driver disabled";
        return false;
    }
    if (config_.port.empty())
    {
        error = "wall-pad USB serial port is required";
        return false;
    }
    if (config_.baud_rate <= 0 || !std::isfinite(config_.max_range_m) ||
        config_.max_range_m <= 0.05f || config_.max_range_m > 35.0f)
    {
        error = "wall-pad baud rate or maximum range is invalid";
        return false;
    }
    if (!serial_.open(config_.port, config_.baud_rate, error)) return false;
    line_buffer_.clear();
    return true;
}

void WallpadDriver::close()
{
    serial_.close();
    line_buffer_.clear();
}

bool WallpadDriver::parseZoneLine(const std::string& input,
                                  float max_range_m,
                                  PresenceZone& zone,
                                  std::string& error)
{
    error.clear();
    if (!std::isfinite(max_range_m) || max_range_m <= 0.05f || max_range_m > 35.0f)
    {
        error = "wall-pad maximum range is invalid";
        return false;
    }
    const std::vector<std::string> fields = splitCsv(trim(input));
    if (fields.size() != 6 || fields[0] != "WALLPAD")
    {
        error = "expected WALLPAD,range_min_m,range_max_m,bearing_center_deg,bearing_width_deg,confidence";
        return false;
    }

    float range_min = 0.0f;
    float range_max = 0.0f;
    float bearing_center_deg = 0.0f;
    float bearing_width_deg = 0.0f;
    float confidence = 0.0f;
    if (!parseFloat(fields[1], range_min) || !parseFloat(fields[2], range_max) ||
        !parseFloat(fields[3], bearing_center_deg) || !parseFloat(fields[4], bearing_width_deg) ||
        !parseFloat(fields[5], confidence))
    {
        error = "wall-pad sector fields must be finite numbers";
        return false;
    }
    if (range_min <= 0.05f || range_max <= range_min || range_max > max_range_m ||
        bearing_center_deg < -180.0f || bearing_center_deg > 180.0f ||
        bearing_width_deg < 5.0f || bearing_width_deg > 170.0f ||
        confidence < 0.05f || confidence > 1.0f)
    {
        error = "wall-pad sector is outside the configured range, bearing, width, or confidence limits";
        return false;
    }

    zone.range_min = range_min;
    zone.range_max = range_max;
    zone.bearing_center_rad = degToRad(bearing_center_deg);
    zone.bearing_width_rad = degToRad(bearing_width_deg);
    zone.confidence = confidence;
    return true;
}

bool WallpadDriver::poll(std::vector<PresenceZone>& zones, std::string& error)
{
    zones.clear();
    error.clear();
    if (!serial_.isOpen())
    {
        error = "wall-pad serial port is not open";
        return false;
    }

    std::vector<uint8_t> incoming;
    if (!serial_.readAvailable(incoming, error)) return false;
    if (incoming.size() > kMaxLineBufferBytes ||
        line_buffer_.size() > kMaxLineBufferBytes - incoming.size())
    {
        line_buffer_.clear();
        error = "wall-pad input buffer exceeds the limit";
        return false;
    }
    if (!incoming.empty())
        line_buffer_.append(reinterpret_cast<const char*>(incoming.data()), incoming.size());

    std::size_t newline = 0;
    while ((newline = line_buffer_.find('\n')) != std::string::npos)
    {
        const std::string line = line_buffer_.substr(0, newline);
        line_buffer_.erase(0, newline + 1);
        if (trim(line).empty()) continue;

        PresenceZone zone{};
        if (!parseZoneLine(line, config_.max_range_m, zone, error))
        {
            zones.clear();
            error = "invalid wall-pad record: " + error;
            return false;
        }
        if (zones.size() >= kMaxWallpadZonesPerFrame)
        {
            zones.clear();
            error = "wall-pad poll exceeds the sector limit";
            return false;
        }
        zones.push_back(zone);
    }
    if (line_buffer_.size() > kMaxLineBufferBytes)
    {
        line_buffer_.clear();
        zones.clear();
        error = "wall-pad line exceeds the input limit";
        return false;
    }
    return !zones.empty();
}

} // namespace sa
