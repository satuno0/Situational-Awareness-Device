#include "marker_driver.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace sa {
namespace {

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
    std::string field;
    std::istringstream stream(line);
    while (std::getline(stream, field, ',')) fields.push_back(trim(field));
    return fields;
}

bool parseFloat(const std::string& text, float& value)
{
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const float parsed = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || errno == ERANGE) return false;
    if (*end != '\0' && *end != ' ') return false;
    value = parsed;
    return std::isfinite(parsed);
}

} // namespace

MarkerDriver::MarkerDriver() : MarkerDriver(MarkerConfig{}) {}
MarkerDriver::MarkerDriver(const MarkerConfig& config) : config_(config) {}
MarkerDriver::~MarkerDriver() { close(); }

bool MarkerDriver::open(std::string& error)
{
    close();
    line_buffer_.clear();
    if (!config_.enabled)
    {
        error = "marker driver disabled";
        return false;
    }
    if (config_.port.empty())
    {
        error = "marker serial port is empty";
        return false;
    }
    return serial_.open(config_.port, config_.baud_rate, error);
}

void MarkerDriver::close()
{
    serial_.close();
    line_buffer_.clear();
}

void MarkerDriver::parseLine(const std::string& line, MarkerObservation& observation, bool& got)
{
    auto fields = splitCsv(line);
    if (fields.size() < 2) return;

    std::string kind = fields[0];
    std::transform(kind.begin(), kind.end(), kind.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    MarkerObservation parsed{};
    if (kind == "mark")
    {
        if (fields.size() < 3) return;
        float bearing_deg = 0.0f;
        float confidence = 0.0f;
        if (!parseFloat(fields[1], bearing_deg) || !parseFloat(fields[2], confidence)) return;
        parsed.bearing_rad = degToRad(bearing_deg);
        parsed.confidence = clampf(confidence, 0.0f, 1.0f);
        if (fields.size() >= 4)
        {
            float area = 0.0f;
            if (parseFloat(fields[3], area)) parsed.area_ratio = clampf(area, 0.0f, 1.0f);
        }
        parsed.valid = parsed.confidence > 0.0f;
    }
    else if (kind == "red")
    {
        if (fields.size() < 6) return;
        float cx = 0.0f;
        float cy = 0.0f;
        float width = 0.0f;
        float height = 0.0f;
        float confidence = 0.0f;
        if (!parseFloat(fields[1], cx) || !parseFloat(fields[2], cy) ||
            !parseFloat(fields[3], width) || !parseFloat(fields[4], height) ||
            !parseFloat(fields[5], confidence))
            return;
        if (width < 3.0f || height < 3.0f) return;

        float hfov_deg = config_.horizontal_fov_deg;
        if (fields.size() >= 7) (void)parseFloat(fields[6], hfov_deg);
        const float focal = (width * 0.5f) / std::tan(std::max(0.1f, degToRad(hfov_deg)) * 0.5f);
        parsed.bearing_rad = std::atan2(-(cx - width * 0.5f), focal);
        parsed.confidence = clampf(confidence, 0.0f, 1.0f);
        parsed.area_ratio = clampf(100.0f / std::max(1.0f, width * height), 0.0f, 1.0f);
        parsed.valid = parsed.confidence > 0.0f;
    }
    else
    {
        return;
    }

    parsed.timestamp_ms = monoNowMs();
    observation = parsed;
    got = true;
}

bool MarkerDriver::poll(MarkerObservation& observation)
{
    if (!serial_.isOpen()) return false;
    std::vector<uint8_t> incoming;
    std::string error;
    if (!serial_.readAvailable(incoming, error)) return false;
    if (incoming.empty()) return false;

    line_buffer_.append(reinterpret_cast<const char*>(incoming.data()), incoming.size());
    if (line_buffer_.size() > 64u * 1024u) line_buffer_.erase(0, line_buffer_.size() - 4096);

    bool got = false;
    std::size_t newline = 0;
    while ((newline = line_buffer_.find('\n')) != std::string::npos)
    {
        std::string line = line_buffer_.substr(0, newline);
        line_buffer_.erase(0, newline + 1);
        parseLine(trim(line), observation, got);
    }
    return got;
}

} // namespace sa
