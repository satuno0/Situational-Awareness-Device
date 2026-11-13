#pragma once

#include "types.hpp"

#include <cstdint>
#include <string>

namespace sa {

// Which sensor inputs are allowed to reach fusion. This is a perception switch,
// not a hardware switch: drivers still poll, and raw sensors are still recorded,
// so one session can be replayed under a different sensor selection.
enum class SensorKind : uint32_t
{
    Imu = 1u << 0,
    Radar = 1u << 1,
    Uwb = 1u << 2,
    Novelda = 1u << 3,
    Ultrasonic = 1u << 4,
    Marker = 1u << 5
};

constexpr uint32_t kAllSensorMask = (1u << 6) - 1u;

struct SensorSelection
{
    uint32_t mask = kAllSensorMask;

    bool has(SensorKind kind) const { return (mask & static_cast<uint32_t>(kind)) != 0; }
};

// Accepts either a preset (mixed, radar, uwb, novelda, ultrasonic, vision, imu)
// or an explicit comma/plus separated list (for example "radar,uwb,marker").
// Presets keep the IMU enabled because it supplies ego-motion, not target data.
bool parseSensorSelection(const std::string& text, SensorSelection& selection, std::string& error);

// Canonical name for logs and telemetry. Presets keep their short name; custom
// sets are written in a stable order, for example "radar+uwb".
std::string sensorSelectionName(const SensorSelection& selection);

// Returns a copy of the frame with disabled sensors removed. Timestamps and
// frame ids are preserved. The raw frame is what should be recorded.
SensorFrame filterSensorFrame(const SensorFrame& frame, const SensorSelection& selection);

} // namespace sa
