#include "sensor_mode.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <vector>

namespace sa {
namespace {

constexpr uint32_t kImu = static_cast<uint32_t>(SensorKind::Imu);
constexpr uint32_t kRadar = static_cast<uint32_t>(SensorKind::Radar);
constexpr uint32_t kUwb = static_cast<uint32_t>(SensorKind::Uwb);
constexpr uint32_t kNovelda = static_cast<uint32_t>(SensorKind::Novelda);
constexpr uint32_t kUltrasonic = static_cast<uint32_t>(SensorKind::Ultrasonic);
constexpr uint32_t kMarker = static_cast<uint32_t>(SensorKind::Marker);
constexpr uint32_t kLidar = static_cast<uint32_t>(SensorKind::Lidar);
constexpr uint32_t kTof = static_cast<uint32_t>(SensorKind::Tof);
constexpr uint32_t kWallpad = static_cast<uint32_t>(SensorKind::Wallpad);

std::string lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::string trim(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> splitList(const std::string& text)
{
    std::vector<std::string> parts;
    std::string current;
    for (char c : text)
    {
        if (c == ',' || c == '+')
        {
            const std::string token = trim(current);
            if (!token.empty()) parts.push_back(token);
            current.clear();
        }
        else
        {
            current.push_back(c);
        }
    }
    const std::string token = trim(current);
    if (!token.empty()) parts.push_back(token);
    return parts;
}

bool preset(const std::string& name, uint32_t& mask)
{
    // Ego-motion support is part of every perception preset. A custom list can
    // explicitly drop the IMU by omitting it.
    if (name == "mixed" || name == "all" || name == "auto")
        mask = kAllSensorMask;
    else if (name == "radar")
        mask = kImu | kRadar;
    else if (name == "lidar")
        mask = kImu | kLidar;
    else if (name == "tof")
        mask = kImu | kTof;
    else if (name == "uwb")
        mask = kImu | kUwb;
    else if (name == "novelda" || name == "xethru")
        mask = kImu | kNovelda;
    else if (name == "wallpad")
        mask = kImu | kWallpad;
    else if (name == "ultrasonic" || name == "sonar")
        mask = kImu | kUltrasonic;
    else if (name == "vision" || name == "marker" || name == "camera")
        mask = kImu | kMarker;
    else if (name == "imu")
        mask = kImu;
    else
        return false;
    return true;
}

bool namedSensor(const std::string& name, uint32_t& bit)
{
    if (name == "imu") bit = kImu;
    else if (name == "radar") bit = kRadar;
    else if (name == "lidar") bit = kLidar;
    else if (name == "tof") bit = kTof;
    else if (name == "uwb") bit = kUwb;
    else if (name == "novelda" || name == "xethru") bit = kNovelda;
    else if (name == "wallpad") bit = kWallpad;
    else if (name == "ultrasonic" || name == "sonar") bit = kUltrasonic;
    else if (name == "marker" || name == "vision" || name == "camera" || name == "rgb") bit = kMarker;
    else return false;
    return true;
}

} // namespace

bool parseSensorSelection(const std::string& text, SensorSelection& selection, std::string& error)
{
    const std::string normalized = lower(trim(text));
    if (normalized.empty())
    {
        error = "sensor selection is empty";
        return false;
    }

    uint32_t mask = 0;
    if (preset(normalized, mask))
    {
        selection.mask = mask;
        return true;
    }

    const std::vector<std::string> parts = splitList(normalized);
    if (parts.empty())
    {
        error = "sensor selection is empty";
        return false;
    }
    for (const auto& part : parts)
    {
        uint32_t bit = 0;
        if (!namedSensor(part, bit))
        {
            error = "unknown sensor '" + part +
                    "' (use mixed, radar, lidar, tof, uwb, novelda, wallpad, ultrasonic, vision, imu or a comma list)";
            return false;
        }
        mask |= bit;
    }
    if (mask == 0)
    {
        error = "sensor selection enables nothing";
        return false;
    }
    selection.mask = mask;
    return true;
}

std::string sensorSelectionName(const SensorSelection& selection)
{
    if (selection.mask == kAllSensorMask) return "mixed";
    if (selection.mask == (kImu | kRadar)) return "radar";
    if (selection.mask == (kImu | kLidar)) return "lidar";
    if (selection.mask == (kImu | kTof)) return "tof";
    if (selection.mask == (kImu | kUwb)) return "uwb";
    if (selection.mask == (kImu | kNovelda)) return "novelda";
    if (selection.mask == (kImu | kWallpad)) return "wallpad";
    if (selection.mask == (kImu | kUltrasonic)) return "ultrasonic";
    if (selection.mask == (kImu | kMarker)) return "vision";
    if (selection.mask == kImu) return "imu";

    std::string name;
    const auto append = [&name](const char* part) {
        if (!name.empty()) name += "+";
        name += part;
    };
    if (selection.has(SensorKind::Imu)) append("imu");
    if (selection.has(SensorKind::Radar)) append("radar");
    if (selection.has(SensorKind::Lidar)) append("lidar");
    if (selection.has(SensorKind::Tof)) append("tof");
    if (selection.has(SensorKind::Uwb)) append("uwb");
    if (selection.has(SensorKind::Novelda)) append("novelda");
    if (selection.has(SensorKind::Wallpad)) append("wallpad");
    if (selection.has(SensorKind::Ultrasonic)) append("ultrasonic");
    if (selection.has(SensorKind::Marker)) append("marker");
    return name.empty() ? "none" : name;
}

SensorFrame filterSensorFrame(const SensorFrame& frame, const SensorSelection& selection)
{
    SensorFrame out = frame;
    if (!selection.has(SensorKind::Imu)) out.imu.reset();
    if (!selection.has(SensorKind::Radar)) out.radar.reset();
    if (!selection.has(SensorKind::Lidar)) out.lidar.reset();
    if (!selection.has(SensorKind::Tof)) out.tof.reset();
    if (!selection.has(SensorKind::Uwb)) out.uwb.clear();
    if (!selection.has(SensorKind::Novelda)) out.novelda.clear();
    if (!selection.has(SensorKind::Wallpad)) out.wallpad_zones.clear();
    if (!selection.has(SensorKind::Ultrasonic)) out.ultrasonic.reset();
    if (!selection.has(SensorKind::Marker)) out.marker.reset();
    return out;
}

} // namespace sa
