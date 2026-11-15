#include "gpio_line.hpp"

#include <cstring>

#if defined(__linux__)
#include <cerrno>
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace sa {

GpioLine::~GpioLine() { close(); }

bool GpioLine::open(const GpioLineConfig& config, bool initial_active, const char* consumer,
                    std::string& error)
{
    close();
#if defined(__linux__)
    const int chip_fd = ::open(config.chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0)
    {
        error = "cannot open GPIO chip " + config.chip + ": " + std::strerror(errno);
        return false;
    }

    gpio_v2_line_request request{};
    request.offsets[0] = config.offset;
    request.num_lines = 1;
    request.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    if (!config.active_high) request.config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
    request.config.num_attrs = 1;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values = initial_active ? 1u : 0u;
    request.config.attrs[0].mask = 1u;
    std::strncpy(request.consumer, consumer, sizeof(request.consumer) - 1);
    if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0)
    {
        error = "cannot request GPIO line " + std::to_string(config.offset) + " from " + config.chip +
                ": " + std::strerror(errno);
        ::close(chip_fd);
        return false;
    }
    ::close(chip_fd);
    line_fd_ = request.fd;
    return true;
#else
    (void)config;
    (void)initial_active;
    (void)consumer;
    error = "GPIO character devices are supported only on Linux";
    return false;
#endif
}

bool GpioLine::set(bool active, std::string& error)
{
#if defined(__linux__)
    if (line_fd_ < 0)
    {
        error = "GPIO line is not open";
        return false;
    }
    gpio_v2_line_values values{};
    values.mask = 1u;
    values.bits = active ? 1u : 0u;
    if (ioctl(line_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0)
    {
        error = std::string("GPIO line write failed: ") + std::strerror(errno);
        return false;
    }
    return true;
#else
    (void)active;
    error = "GPIO character devices are supported only on Linux";
    return false;
#endif
}

void GpioLine::close()
{
#if defined(__linux__)
    if (line_fd_ >= 0) ::close(line_fd_);
#endif
    line_fd_ = -1;
}

} // namespace sa
