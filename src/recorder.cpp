#include "recorder.hpp"

#include "serialization.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace sa {
namespace {

constexpr char kMagic[8] = {'S', 'A', 'D', 'A', 'S', 'E', 'S', 'S'};
constexpr uint32_t kHeaderSize = 64;
constexpr uint32_t kEndianMarker = 0x01020304u;

void appendU32(std::vector<uint8_t>& out, uint32_t value)
{
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFFu));
}

} // namespace

Recorder::Recorder() : Recorder(RecorderConfig{}) {}
Recorder::Recorder(const RecorderConfig& config) : config_(config) {}
Recorder::~Recorder() { close(); }

bool Recorder::open(const std::string& path, std::string& error)
{
    close();
    if (path.empty())
    {
        error = "recording path is empty";
        return false;
    }
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_)
    {
        error = "cannot open session for writing: " + path;
        return false;
    }
    error_.clear();
    frames_written_ = 0;
    bytes_written_ = 0;
    last_track_snapshot_ms_.reset();
    last_world_snapshot_ms_.reset();

    const uint64_t created = wallClockMs();
    BinaryWriter header;
    header.bytes(kMagic, sizeof(kMagic));
    header.u32(kSessionFormatVersion);
    header.u32(kHeaderSize);
    header.u32(kEndianMarker);
    header.u64(created);
    header.data().resize(kHeaderSize, 0);
    file_.write(reinterpret_cast<const char*>(header.data().data()), static_cast<std::streamsize>(header.size()));
    if (!file_)
    {
        error = "failed to write session header";
        close();
        return false;
    }
    bytes_written_ += header.size();

    BinaryWriter metadata;
    metadata.u64(created);
    metadata.string("sa_daemon");
    metadata.string(std::to_string(kSessionFormatVersion));
    writeRecord(RecordType::Metadata, metadata.data());
    return error_.empty();
}

void Recorder::close()
{
    if (file_.is_open())
    {
        file_.flush();
        file_.close();
    }
}

bool Recorder::writeRecord(RecordType type, const std::vector<uint8_t>& payload)
{
    if (!file_.is_open()) return false;
    if (payload.size() > 32u * 1024u * 1024u)
    {
        error_ = "record payload exceeds format limit";
        return false;
    }

    std::vector<uint8_t> header;
    header.reserve(16);
    appendU32(header, static_cast<uint32_t>(type));
    appendU32(header, static_cast<uint32_t>(payload.size()));
    appendU32(header, crc32(payload.data(), payload.size()));
    appendU32(header, 0);
    file_.write(reinterpret_cast<const char*>(header.data()), static_cast<std::streamsize>(header.size()));
    if (!payload.empty())
        file_.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    if (!file_)
    {
        error_ = "session write failed";
        return false;
    }
    bytes_written_ += header.size() + payload.size();
    return true;
}

bool Recorder::writeImuRecord(const ImuSample& sample, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(sample.timestamp_ms);
    writeImu(writer, sample);
    return writeRecord(RecordType::Imu, writer.data());
}

bool Recorder::writeRadarRecord(const RadarFrame& frame, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(frame.timestamp_ms);
    writeRadarFrame(writer, frame);
    return writeRecord(RecordType::Radar, writer.data());
}

bool Recorder::writeLidarRecord(const LidarScan& scan, uint32_t frame_id)
{
    if (scan.points.size() > kMaxLidarPointsPerScan)
    {
        error_ = "LiDAR scan exceeds the point limit";
        return false;
    }
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(scan.timestamp_ms);
    writeLidarScan(writer, scan);
    return writeRecord(RecordType::Lidar, writer.data());
}

bool Recorder::writeTofRecord(const TofScan& scan, uint32_t frame_id)
{
    if (scan.rays.size() > kMaxTofRaysPerScan)
    {
        error_ = "ToF scan exceeds the ray limit";
        return false;
    }
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(scan.timestamp_ms);
    writeTofScan(writer, scan);
    return writeRecord(RecordType::Tof, writer.data());
}

bool Recorder::writeUwbRecord(const UwbMeasurement& sample, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(sample.timestamp_ms);
    writeUwb(writer, sample);
    return writeRecord(RecordType::Uwb, writer.data());
}

bool Recorder::writeNoveldaRecord(const NoveldaMeasurement& sample, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(sample.timestamp_ms);
    writeNovelda(writer, sample);
    return writeRecord(RecordType::Novelda, writer.data());
}

bool Recorder::writeWallpadRecord(const PresenceZone& zone, uint32_t frame_id, uint64_t timestamp_ms)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(timestamp_ms);
    writeZone(writer, zone);
    return writeRecord(RecordType::Wallpad, writer.data());
}

bool Recorder::writeWristOrientationRecord(const WristOrientation& orientation, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(orientation.timestamp_ms);
    writer.f32(orientation.wrist_yaw_deg);
    writer.f32(orientation.wrist_pitch_deg);
    writer.u8(orientation.glove_packet ? 1 : 0);
    writer.u8(orientation.pointing_active ? 1 : 0);
    writer.u8(orientation.zero_heading_requested ? 1 : 0);
    writer.u8(orientation.glove_events);
    return writeRecord(RecordType::WristOrientation, writer.data());
}

bool Recorder::writeUltrasonicRecord(const UltrasonicMeasurement& sample, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(sample.timestamp_ms);
    writeUltrasonic(writer, sample);
    return writeRecord(RecordType::Ultrasonic, writer.data());
}

bool Recorder::writeMarkerRecord(const MarkerObservation& sample, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(sample.timestamp_ms);
    writeMarker(writer, sample);
    return writeRecord(RecordType::Marker, writer.data());
}

bool Recorder::writeSchematicRecord(const std::string& text)
{
    BinaryWriter writer;
    writer.string(text);
    return writeRecord(RecordType::Schematic, writer.data());
}

bool Recorder::writeTracksRecord(const WorldModel& world, uint32_t frame_id, uint64_t timestamp_ms)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(timestamp_ms);
    writeTracks(writer, world.tracks);
    return writeRecord(RecordType::Tracks, writer.data());
}

bool Recorder::writeWorldRecord(const WorldModel& world, uint32_t frame_id)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(world.timestamp_ms);
    writeWorld(writer, world);
    return writeRecord(RecordType::World, writer.data());
}

bool Recorder::writeFrameEnd(uint32_t frame_id, uint64_t timestamp_ms, uint32_t flags)
{
    BinaryWriter writer;
    writer.u32(frame_id);
    writer.u64(timestamp_ms);
    writer.u32(flags);
    writer.u32(0);
    return writeRecord(RecordType::FrameEnd, writer.data());
}

bool Recorder::recordFrame(const SensorFrame& frame, const WorldModel* processed_world, bool force_snapshots)
{
    if (!file_.is_open())
    {
        error_ = "recorder is not open";
        return false;
    }
    if (frame.lidar && frame.lidar->points.size() > kMaxLidarPointsPerScan)
    {
        error_ = "LiDAR scan exceeds the point limit";
        return false;
    }
    if (frame.tof && frame.tof->rays.size() > kMaxTofRaysPerScan)
    {
        error_ = "ToF scan exceeds the ray limit";
        return false;
    }
    if (frame.wallpad_zones.size() > kMaxWallpadZonesPerFrame)
    {
        error_ = "wall-pad frame exceeds the sector limit";
        return false;
    }

    if (frame.imu && !writeImuRecord(*frame.imu, frame.frame_id)) return false;
    if (frame.radar && !writeRadarRecord(*frame.radar, frame.frame_id)) return false;
    if (frame.lidar && !writeLidarRecord(*frame.lidar, frame.frame_id)) return false;
    if (frame.tof && !writeTofRecord(*frame.tof, frame.frame_id)) return false;
    for (const auto& sample : frame.uwb)
        if (!writeUwbRecord(sample, frame.frame_id)) return false;
    for (const auto& sample : frame.novelda)
        if (!writeNoveldaRecord(sample, frame.frame_id)) return false;
    for (const auto& zone : frame.wallpad_zones)
        if (!writeWallpadRecord(zone, frame.frame_id, frame.timestamp_ms)) return false;
    if (frame.ultrasonic && !writeUltrasonicRecord(*frame.ultrasonic, frame.frame_id)) return false;
    if (frame.marker && !writeMarkerRecord(*frame.marker, frame.frame_id)) return false;
    if (frame.wrist_orientation &&
        !writeWristOrientationRecord(*frame.wrist_orientation, frame.frame_id)) return false;

    uint32_t flags = 0;
    if (processed_world)
    {
        const uint64_t now_ms = processed_world->timestamp_ms;
        bool wrote_track = false;
        bool wrote_world = false;
        if (config_.write_track_snapshots)
        {
            const uint64_t interval = static_cast<uint64_t>(1000.0f / std::max(0.1f, config_.track_snapshot_hz));
            if (force_snapshots || !last_track_snapshot_ms_ || now_ms >= *last_track_snapshot_ms_ + interval)
            {
                if (!writeTracksRecord(*processed_world, frame.frame_id, now_ms)) return false;
                last_track_snapshot_ms_ = now_ms;
                wrote_track = true;
            }
        }
        if (config_.write_world_snapshots)
        {
            const uint64_t interval = static_cast<uint64_t>(1000.0f / std::max(0.1f, config_.world_snapshot_hz));
            if (force_snapshots || !last_world_snapshot_ms_ || now_ms >= *last_world_snapshot_ms_ + interval)
            {
                if (!writeWorldRecord(*processed_world, frame.frame_id)) return false;
                last_world_snapshot_ms_ = now_ms;
                wrote_world = true;
            }
        }
        if (wrote_track) flags |= 1u;
        if (wrote_world) flags |= 2u;
    }

    if (!writeFrameEnd(frame.frame_id, frame.timestamp_ms, flags)) return false;
    ++frames_written_;
    return true;
}

} // namespace sa
