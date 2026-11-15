#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace sa {

struct WallpadConfig
{
    std::string port;
    int baud_rate = 115200;
    float max_range_m = 30.0f;
    bool enabled = true;
};

class WallpadDriver
{
public:
    WallpadDriver();
    explicit WallpadDriver(const WallpadConfig& config);
    ~WallpadDriver();

    WallpadDriver(const WallpadDriver&) = delete;
    WallpadDriver& operator=(const WallpadDriver&) = delete;

    bool open(std::string& error);
    void close();
    bool isOpen() const { return serial_.isOpen(); }

    // Reads sector estimates from a USB serial sweep-adapter bridge.
    bool poll(std::vector<PresenceZone>& zones, std::string& error);

    static bool parseZoneLine(const std::string& line,
                              float max_range_m,
                              PresenceZone& zone,
                              std::string& error);

private:
    WallpadConfig config_;
    SerialPort serial_;
    std::string line_buffer_;
};

} // namespace sa
