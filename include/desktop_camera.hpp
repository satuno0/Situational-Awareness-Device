#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sa {

class DesktopCamera
{
public:
    DesktopCamera();
    ~DesktopCamera();

    DesktopCamera(const DesktopCamera&) = delete;
    DesktopCamera& operator=(const DesktopCamera&) = delete;

    bool open(std::string& error);
    bool readRgb(std::vector<uint8_t>& pixels, int& width, int& height, std::string& error);
    void close();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sa
