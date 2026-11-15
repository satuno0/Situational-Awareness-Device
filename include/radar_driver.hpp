#pragma once

#include "serial_port.hpp"
#include "types.hpp"

#include <deque>
#include <string>
#include <vector>

namespace sa {

struct RadarConfig
{
    std::string data_port;
    std::string cfg_port;
    std::string config_file;
    int data_baud = 921600;
    bool configure_on_open = true;
    bool enabled = true;

    Vec3 mount_translation_m{0.0f, 0.0f, 0.0f};
    float yaw_offset_deg = 0.0f;
};

// TI IWR6843AOPEVM driver. The data port uses the TI mmWave demo packet format
// (point-cloud TLV 1 and side-info TLV 7). The optional config port sends a
// .cfg file before opening the data stream.
class RadarDriver
{
public:
    RadarDriver();
    explicit RadarDriver(const RadarConfig& config);
    ~RadarDriver();

    bool open(std::string& error);
    void close();
    bool isOpen() const { return data_.isOpen(); }

    // Returns one complete radar frame if available. Never blocks.
    bool poll(RadarFrame& frame);
    bool healthy() const { return data_.isOpen() && healthy_; }
    static void transformToBody(RadarFrame& frame, const RadarConfig& config);

private:
    bool configure(std::string& error);
    bool readAndParse(std::string& error);
    bool parsePacket(const uint8_t* packet, std::size_t size, RadarFrame& frame);

    RadarConfig config_;
    SerialPort data_;
    std::vector<uint8_t> buffer_;
    std::deque<RadarFrame> frames_;
    bool healthy_ = false;
};

} // namespace sa
