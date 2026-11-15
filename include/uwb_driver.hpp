#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace sa {

struct UwbConfig
{
    // A small MCU/firmware bridge owns the DWM3000 SPI radio and emits one
    // ranging result per line:
    //   tag_id,range_m,aoa_deg,confidence[,range_rate_mps[,is_fixed_anchor,anchor_x_m,anchor_y_m]]
    // Empty/nan AoA fields mean the optional AoA estimate is unavailable.
    std::string port;
    int baud_rate = 115200;
    float max_range_m = 30.0f;
    bool enabled = true;
};

class UwbDriver
{
public:
    UwbDriver();
    explicit UwbDriver(const UwbConfig& config);
    ~UwbDriver();

    bool open(std::string& error);
    void close();
    bool isOpen() const { return serial_.isOpen(); }
    bool poll(std::vector<UwbMeasurement>& measurements);

private:
    void parseLine(const std::string& line, std::vector<UwbMeasurement>& measurements);

    UwbConfig config_;
    SerialPort serial_;
    std::string line_buffer_;
};

} // namespace sa
