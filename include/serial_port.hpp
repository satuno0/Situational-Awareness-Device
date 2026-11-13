#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sa {

class SerialPort
{
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud_rate, std::string& error);
    void close();
    bool isOpen() const { return handle_ != -1; }

    // Non-blocking: appends every byte currently available.
    bool readAvailable(std::vector<uint8_t>& out, std::string& error);
    bool writeAll(const uint8_t* data, std::size_t size, std::string& error);
    bool writeAll(const std::string& text, std::string& error);

private:
    intptr_t handle_ = -1;
};

} // namespace sa
