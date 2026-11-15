#pragma once

#include <cstdint>
#include <string>

namespace sa {

struct GpioLineConfig
{
    std::string chip = "/dev/gpiochip3";
    uint32_t offset = 0;
    bool active_high = true;
};

class GpioLine
{
public:
    GpioLine() = default;
    ~GpioLine();

    GpioLine(const GpioLine&) = delete;
    GpioLine& operator=(const GpioLine&) = delete;

    bool open(const GpioLineConfig& config, bool initial_active, const char* consumer, std::string& error);
    bool set(bool active, std::string& error);
    void close();
    bool isOpen() const { return line_fd_ >= 0; }

private:
    int line_fd_ = -1;
};

} // namespace sa
