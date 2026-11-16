#include "ultrasonic_driver.hpp"

#include <cctype>
#include <cstdlib>

namespace sa {

UltrasonicDriver::UltrasonicDriver() : UltrasonicDriver(UltrasonicConfig{}) {}
UltrasonicDriver::UltrasonicDriver(const UltrasonicConfig& config) : config_(config) {}
UltrasonicDriver::~UltrasonicDriver() { close(); }

bool UltrasonicDriver::open(std::string& error)
{
    close();
    line_buffer_.clear();
    if (!config_.enabled)
    {
        error = "ultrasonic driver disabled";
        return false;
    }
    if (config_.port.empty())
    {
        error = "MaxBotix serial port is empty";
        return false;
    }
    return serial_.open(config_.port, config_.baud_rate, error);
}

void UltrasonicDriver::close()
{
    serial_.close();
    line_buffer_.clear();
}

void UltrasonicDriver::parseLine(const std::string& line,
                                 uint64_t timestamp_ms,
                                 std::vector<UltrasonicMeasurement>& measurements)
{
    if (line.size() < 5 || (line[0] != 'R' && line[0] != 'r')) return;
    const char* begin = line.c_str() + 1;
    char* end = nullptr;
    const long millimetres = std::strtol(begin, &end, 10);
    if (end == begin || millimetres < 0) return;

    UltrasonicMeasurement sample{};
    sample.timestamp_ms = timestamp_ms;
    if (millimetres == 0)
    {
        sample.valid = false;
        sample.range_m = 0.0f;
    }
    else
    {
        sample.range_m = static_cast<float>(millimetres) / 1000.0f;
        sample.valid = sample.range_m >= 0.10f && sample.range_m <= 10.0f;
    }
    measurements.push_back(sample);
}

bool UltrasonicDriver::poll(std::vector<UltrasonicMeasurement>& measurements)
{
    if (!serial_.isOpen()) return false;
    std::vector<uint8_t> incoming;
    std::string error;
    if (!serial_.readAvailable(incoming, error)) return false;
    return ingestBytes(incoming.data(), incoming.size(), monoNowMs(), measurements);
}

bool UltrasonicDriver::ingestBytes(const uint8_t* data,
                                   std::size_t size,
                                   uint64_t timestamp_ms,
                                   std::vector<UltrasonicMeasurement>& measurements)
{
    if (data == nullptr || size == 0) return false;
    line_buffer_.append(reinterpret_cast<const char*>(data), size);
    if (line_buffer_.size() > 16u * 1024u) line_buffer_.erase(0, line_buffer_.size() - 1024);

    bool got = false;
    std::size_t terminator = 0;
    while ((terminator = line_buffer_.find_first_of("\r\n")) != std::string::npos)
    {
        std::string line = line_buffer_.substr(0, terminator);
        line_buffer_.erase(0, terminator + 1);
        while (!line.empty() && line.back() == ' ') line.pop_back();
        const std::size_t before = measurements.size();
        parseLine(line, timestamp_ms, measurements);
        if (measurements.size() > before) got = true;
    }
    return got;
}

} // namespace sa
