#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <string>

namespace sa {

struct MarkerConfig
{
    // Optional camera bridge. The bridge owns the image sensor and emits one
    // result per line. Two formats are accepted:
    //   MARK,<bearing_deg>,<confidence>[,<area_ratio>]
    //   RED,<cx_px>,<cy_px>,<width_px>,<height_px>,<confidence>[,<hfov_deg>]
    // MARK bearing is positive to the right of the camera axis. RED carries a
    // red-cluster centroid and is converted to the same bearing.
    std::string port;
    int baud_rate = 115200;
    float horizontal_fov_deg = 60.0f;
    bool enabled = true;
};

class MarkerDriver
{
public:
    MarkerDriver();
    explicit MarkerDriver(const MarkerConfig& config);
    ~MarkerDriver();

    bool open(std::string& error);
    void close();
    bool isOpen() const { return serial_.isOpen(); }

    // Returns the newest complete observation. Never blocks.
    bool poll(MarkerObservation& observation);

private:
    void parseLine(const std::string& line, MarkerObservation& observation, bool& got);

    MarkerConfig config_;
    SerialPort serial_;
    std::string line_buffer_;
};

} // namespace sa
