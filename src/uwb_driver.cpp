#include "uwb_driver.hpp"

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

UwbDriver::UwbDriver() : UwbDriver(UwbConfig{}) {}
UwbDriver::UwbDriver(const UwbConfig& config) : config_(config) {}
UwbDriver::~UwbDriver() { close(); }

bool UwbDriver::open(std::string& error)
{
    close();
    line_buffer_.clear();
    if (!config_.enabled)
    {
        error = "UWB driver disabled";
        return false;
    }
    if (config_.port.empty())
    {
        error = "UWB serial port is empty";
        return false;
    }
    return serial_.open(config_.port, config_.baud_rate, error);
}

void UwbDriver::close()
{
    serial_.close();
    line_buffer_.clear();
}

void UwbDriver::parseLine(const std::string& line, std::vector<UwbMeasurement>& measurements)
{
    auto fields = splitCsv(line);
    if (fields.empty()) return;
    std::string first = fields[0];
    std::transform(first.begin(), first.end(), first.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (first == "uwb" || first == "tag")
    {
        fields.erase(fields.begin());
    }
    if (fields.size() < 4) return;

    char* end = nullptr;
    const unsigned long tag = std::strtoul(fields[0].c_str(), &end, 0);
    if (end == fields[0].c_str() || tag == 0 || tag > 65535) return;

    float range = 0.0f;
    if (!parseFloat(fields[1], range) || range <= 0.05f || range > config_.max_range_m) return;

    UwbMeasurement measurement{};
    measurement.timestamp_ms = monoNowMs();
    measurement.tag_id = static_cast<uint16_t>(tag);
    measurement.range_m = range;
    measurement.confidence = 0.7f;

    float aoa_deg = 0.0f;
    if (!fields[2].empty() && fields[2] != "nan" && fields[2] != "NaN" && parseFloat(fields[2], aoa_deg))
    {
        measurement.has_aoa = true;
        measurement.aoa_rad = degToRad(aoa_deg);
    }

    float confidence = 0.7f;
    if (parseFloat(fields[3], confidence)) measurement.confidence = clampf(confidence, 0.0f, 1.0f);
    if (fields.size() >= 5)
    {
        float range_rate = 0.0f;
        if (!fields[4].empty() && fields[4] != "nan" && parseFloat(fields[4], range_rate))
        {
            measurement.has_range_rate = true;
            measurement.range_rate_mps = range_rate;
        }
    }
    measurements.push_back(measurement);
}

bool UwbDriver::poll(std::vector<UwbMeasurement>& measurements)
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
