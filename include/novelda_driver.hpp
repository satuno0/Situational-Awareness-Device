#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace sa {

struct NoveldaConfig
{
    // Firmware bridge line protocol:
    //   NOV,range_m,motion,breathing,confidence[,bearing_deg,bearing_width_deg[,through_barrier]]
    // The XeThru module itself is not a primary tracker. It only produces
    // presence evidence for this device.
    std::string port;
    int baud_rate = 115200;
    bool enabled = true;
};

class NoveldaDriver
{
public:
    NoveldaDriver();
    explicit NoveldaDriver(const NoveldaConfig& config);
    ~NoveldaDriver();

    bool open(std::string& error);
    void close();
    bool isOpen() const { return serial_.isOpen(); }
    bool poll(std::vector<NoveldaMeasurement>& measurements);

private:
    void parseLine(const std::string& line, std::vector<NoveldaMeasurement>& measurements);

    NoveldaConfig config_;
    SerialPort serial_;
    std::string line_buffer_;
};

} // namespace sa
