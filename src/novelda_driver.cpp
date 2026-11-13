#include "novelda_driver.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>

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

NoveldaDriver::NoveldaDriver() : NoveldaDriver(NoveldaConfig{}) {}
NoveldaDriver::NoveldaDriver(const NoveldaConfig& config) : config_(config) {}
NoveldaDriver::~NoveldaDriver() { close(); }

bool NoveldaDriver::open(std::string& error)
{
    close();
    line_buffer_.clear();
    if (!config_.enabled)
    {
        error = "Novelda driver disabled";
        return false;
    }
    if (config_.port.empty())
    {
        error = "Novelda serial port is empty";
        return false;
    }
    return serial_.open(config_.port, config_.baud_rate, error);
}

void NoveldaDriver::close()
{
    serial_.close();
    line_buffer_.clear();
}

void NoveldaDriver::parseLine(const std::string& line, std::vector<NoveldaMeasurement>& measurements)
{
    auto fields = splitCsv(line);
    if (fields.empty()) return;
    std::string first = fields[0];
    std::transform(first.begin(), first.end(), first.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (first == "nov" || first == "novelda")
    {
        fields.erase(fields.begin());
    }
    if (fields.size() < 4) return;

    NoveldaMeasurement sample{};
    if (!parseFloat(fields[0], sample.range_m) || sample.range_m <= 0.05f || sample.range_m > 30.0f) return;
    if (!parseFloat(fields[1], sample.motion)) sample.motion = 0.0f;
    if (!parseFloat(fields[2], sample.breathing)) sample.breathing = 0.0f;
    if (!parseFloat(fields[3], sample.confidence)) return;
    sample.motion = clampf(sample.motion, 0.0f, 1.0f);
    sample.breathing = clampf(sample.breathing, 0.0f, 1.0f);
    sample.confidence = clampf(sample.confidence, 0.0f, 1.0f);
    if (sample.confidence < 0.05f) return;

    sample.timestamp_ms = monoNowMs();
    sample.bearing_center_rad = 0.0f;
    sample.bearing_width_rad = degToRad(60.0f);
    if (fields.size() >= 5)
    {
        float bearing_deg = 0.0f;
        if (parseFloat(fields[4], bearing_deg)) sample.bearing_center_rad = degToRad(bearing_deg);
    }
    if (fields.size() >= 6)
    {
        float width_deg = 60.0f;
        if (parseFloat(fields[5], width_deg)) sample.bearing_width_rad = degToRad(clampf(width_deg, 5.0f, 160.0f));
    }
    if (fields.size() >= 7)
    {
        sample.through_barrier = fields[6] == "1" || fields[6] == "true" || fields[6] == "yes";
    }
    measurements.push_back(sample);
}

bool NoveldaDriver::poll(std::vector<NoveldaMeasurement>& measurements)
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
        const std::size_t before = measurements.size();
        parseLine(trim(line), measurements);
        if (measurements.size() > before) got = true;
    }
    return got;
}

} // namespace sa
