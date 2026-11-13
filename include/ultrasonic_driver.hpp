#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace sa {

struct UltrasonicConfig
{
    std::string port;
    int baud_rate = 9600;
    bool enabled = true;
};

// MaxBotix asynchronous UART output (e.g. "R1234\r", millimetres).
class UltrasonicDriver
{
public:
    UltrasonicDriver();
    explicit UltrasonicDriver(const UltrasonicConfig& config);
    ~UltrasonicDriver();

    bool open(std::string& error);
    void close();
    bool isOpen() const { return serial_.isOpen(); }
    bool poll(std::vector<UltrasonicMeasurement>& measurements);

private:
    void parseLine(const std::string& line, std::vector<UltrasonicMeasurement>& measurements);

    UltrasonicConfig config_;
    SerialPort serial_;
    std::string line_buffer_;
};

} // namespace sa
