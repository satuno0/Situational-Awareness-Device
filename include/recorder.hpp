#pragma once

#include "types.hpp"

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>

namespace sa {

enum class RecordType : uint32_t
{
    Metadata = 0,
    Imu = 1,
    Radar = 2,
    Uwb = 3,
    Novelda = 4,
    Ultrasonic = 5,
    Tracks = 6,
    World = 7,
    FrameEnd = 8,
    Marker = 9,
    Schematic = 10
};

struct RecorderConfig
{
    bool write_world_snapshots = true;
    bool write_track_snapshots = true;
    float world_snapshot_hz = 1.0f;
    float track_snapshot_hz = 2.0f;
};

// Versioned little-endian session format. Every record carries a CRC32 and
// every processing tick ends with a FrameEnd record. Sensor records are the
// replay source; world/track records are debug references only.
class Recorder
{
public:
    Recorder();
    explicit Recorder(const RecorderConfig& config);
    ~Recorder();

    bool open(const std::string& path, std::string& error);
    void close();
    bool isOpen() const { return file_.is_open(); }

    bool recordFrame(const SensorFrame& frame,
                     const WorldModel* processed_world = nullptr,
                     bool force_snapshots = false);

    // Embeds a schematic as a replay input. The record is written at the point
    // it becomes active: once at session start and again after a runtime upload.
    // Replaying the session can therefore reconstruct the same structural
    // prior without external files.
    bool writeSchematicRecord(const std::string& text);

    uint64_t framesWritten() const { return frames_written_; }
    uint64_t bytesWritten() const { return bytes_written_; }
    const std::string& error() const { return error_; }

private:
    bool writeRecord(RecordType type, const std::vector<uint8_t>& payload);
    bool writeImuRecord(const ImuSample& sample, uint32_t frame_id);
    bool writeRadarRecord(const RadarFrame& frame, uint32_t frame_id);
    bool writeUwbRecord(const UwbMeasurement& sample, uint32_t frame_id);
    bool writeNoveldaRecord(const NoveldaMeasurement& sample, uint32_t frame_id);
    bool writeUltrasonicRecord(const UltrasonicMeasurement& sample, uint32_t frame_id);
    bool writeMarkerRecord(const MarkerObservation& sample, uint32_t frame_id);
    bool writeTracksRecord(const WorldModel& world, uint32_t frame_id, uint64_t timestamp_ms);
    bool writeWorldRecord(const WorldModel& world, uint32_t frame_id);
    bool writeFrameEnd(uint32_t frame_id, uint64_t timestamp_ms, uint32_t flags);

    RecorderConfig config_;
    std::ofstream file_;
    std::string error_;
    uint64_t frames_written_ = 0;
    uint64_t bytes_written_ = 0;
    std::optional<uint64_t> last_track_snapshot_ms_;
    std::optional<uint64_t> last_world_snapshot_ms_;
};

} // namespace sa
