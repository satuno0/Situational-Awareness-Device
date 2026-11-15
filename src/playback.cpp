#include "playback.hpp"

#include "serialization.hpp"

#include <cstring>

namespace sa {
namespace {

constexpr char kMagic[8] = {'S', 'A', 'D', 'A', 'S', 'E', 'S', 'S'};
constexpr uint32_t kEndianMarker = 0x01020304u;

} // namespace

Playback::Playback() = default;
Playback::Playback(const std::string& path) { std::string ignored; open(path, ignored); }
Playback::~Playback() { close(); }

bool Playback::open(const std::string& path, std::string& error)
{
    close();
    path_ = path;
    file_.open(path, std::ios::binary);
    if (!file_)
    {
        error = "cannot open session for replay: " + path;
        error_ = error;
        return false;
    }
    uint8_t header[64]{};
    file_.read(reinterpret_cast<char*>(header), sizeof(header));
    if (file_.gcount() != static_cast<std::streamsize>(sizeof(header)))
    {
        error = "session header is truncated";
        close();
        error_ = error;
        return false;
    }
    BinaryReader reader(header, sizeof(header));
    uint8_t magic[8]{};
    if (!reader.bytes(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(magic)) != 0)
    {
        error = "not a sa_daemon session file";
        close();
        error_ = error;
        return false;
    }
    const uint32_t version = reader.u32();
    const uint32_t header_size = reader.u32();
    const uint32_t endian = reader.u32();
    if (!reader.ok() || endian != kEndianMarker || header_size < 28 || header_size > 4096)
    {
        error = "unsupported session format or endianness";
        close();
        error_ = error;
        return false;
    }
    if (version == 1)
    {
        error = "session format v1 uses the legacy coordinate frame and cannot be replayed; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version == 2)
    {
        error = "session format v2 predates LiDAR records and cannot be replayed; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version == 3)
    {
        error = "session format v3 predates ToF records and cannot be replayed; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version == 4)
    {
        error = "session format v4 predates scan-burst and fixed-anchor fields; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version == 5)
    {
        error = "session format v5 uses the smaller occupancy-grid profile; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version == 6)
    {
        error = "session format v6 predates wall-pad presence records; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version == 7)
    {
        error = "session format v7 predates wrist orientation records; record a new v8 session";
        close();
        error_ = error;
        return false;
    }
    if (version != kSessionFormatVersion)
    {
        error = "unsupported session format version " + std::to_string(version);
        close();
        error_ = error;
        return false;
    }
    header_size_ = header_size;
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(header_size_), std::ios::beg);
    if (!file_)
    {
        error = "failed to seek past session header";
        close();
        error_ = error;
        return false;
    }

    stats_ = PlaybackStats{};
    eof_ = false;
    error_.clear();
    if (!readPreamble())
    {
        error = error_.empty() ? "failed to read session preamble" : error_;
        close();
        return false;
    }
    error.clear();
    return true;
}

void Playback::close()
{
    if (file_.is_open()) file_.close();
}

bool Playback::rewind(std::string& error)
{
    if (!file_.is_open())
    {
        error = "session is not open";
        return false;
    }
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(header_size_), std::ios::beg);
    if (!file_)
    {
        error = "failed to rewind session";
        return false;
    }
    stats_ = PlaybackStats{};
    eof_ = false;
    error_.clear();
    return true;
}

bool Playback::readExactly(void* data, std::size_t size)
{
    if (size == 0) return true;
    file_.read(static_cast<char*>(data), static_cast<std::streamsize>(size));
    return file_.gcount() == static_cast<std::streamsize>(size);
}

// Reads the leading metadata/schematic records once, then returns to the same
// position so normal frame iteration sees them again. This lets the caller set
// the structural prior before the first fusion tick without buffering the whole
// session.
bool Playback::readPreamble()
{
    schematic_text_.clear();
    const std::streampos start = file_.tellg();
    if (start == std::streampos(-1)) return false;

    while (true)
    {
        uint8_t header[16]{};
        file_.read(reinterpret_cast<char*>(header), sizeof(header));
        const std::streamsize got = file_.gcount();
        if (got == 0 && file_.eof()) break;
        if (got != static_cast<std::streamsize>(sizeof(header)))
        {
            file_.clear();
            break;
        }

        BinaryReader reader(header, sizeof(header));
        const uint32_t type = reader.u32();
        const uint32_t size = reader.u32();
        const uint32_t expected_crc = reader.u32();
        (void)reader.u32();
        if (!reader.ok() || size > 1024u * 1024u)
        {
            file_.clear();
            break;
        }

        std::vector<uint8_t> payload(size);
        if (!payload.empty())
        {
            file_.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
            if (file_.gcount() != static_cast<std::streamsize>(payload.size()))
            {
                file_.clear();
                break;
            }
        }
        if (crc32(payload.data(), payload.size()) != expected_crc)
        {
            file_.clear();
            break;
        }

        const RecordType record_type = static_cast<RecordType>(type);
        if (record_type == RecordType::Schematic)
        {
            BinaryReader record(payload);
            std::string text = record.string();
            if (record.ok()) schematic_text_ = std::move(text);
            continue;
        }
        if (record_type == RecordType::Metadata)
        {
            BinaryReader record(payload);
            (void)record.u64();
            (void)record.string();
            (void)record.string();
            if (record.ok() && record.remaining() > 0)
            {
                std::string text = record.string();
                if (record.ok()) schematic_text_ = std::move(text);
            }
            continue;
        }
        break;
    }

    file_.clear();
    file_.seekg(start);
    if (!file_)
    {
        error_ = "failed to seek past session preamble";
        return false;
    }
    eof_ = false;
    return true;
}

bool Playback::readHeader(uint32_t& type, uint32_t& size, uint32_t& crc, bool& clean_eof)
{
    uint8_t header[16]{};
    file_.read(reinterpret_cast<char*>(header), sizeof(header));
    const std::streamsize got = file_.gcount();
    if (got == 0 && file_.eof())
    {
        clean_eof = true;
        eof_ = true;
        return false;
    }
    clean_eof = false;
    if (got != static_cast<std::streamsize>(sizeof(header)))
    {
        stats_.truncated = true;
        error_ = "truncated record header";
        eof_ = true;
        return false;
    }
    BinaryReader reader(header, sizeof(header));
    type = reader.u32();
    size = reader.u32();
    crc = reader.u32();
    (void)reader.u32();
    return reader.ok();
}

bool Playback::next(PlaybackFrame& frame)
{
    frame = PlaybackFrame{};
    if (!file_.is_open() || eof_) return false;

    std::optional<WorldModel> pending_world;
    std::optional<std::vector<Track>> pending_tracks;
    std::optional<std::string> pending_schematic;
    uint32_t frame_id = 0;
    uint64_t frame_timestamp = 0;
    bool saw_sensor = false;

    while (true)
    {
        uint32_t type = 0;
        uint32_t size = 0;
        uint32_t expected_crc = 0;
        bool clean_eof = false;
        if (!readHeader(type, size, expected_crc, clean_eof))
        {
            if (!clean_eof && stats_.truncated) error_ = "truncated session";
            return false;
        }
        if (size > 64u * 1024u * 1024u)
        {
            error_ = "record is unreasonably large";
            eof_ = true;
            return false;
        }
        std::vector<uint8_t> payload(size);
        if (!payload.empty() && !readExactly(payload.data(), payload.size()))
        {
            stats_.truncated = true;
            error_ = "truncated record payload";
            eof_ = true;
            return false;
        }
        ++stats_.records;
        const uint32_t actual_crc = crc32(payload.data(), payload.size());
        if (actual_crc != expected_crc)
        {
            ++stats_.crc_errors;
            continue;
        }

        BinaryReader reader(payload);
        switch (static_cast<RecordType>(type))
        {
        case RecordType::Metadata:
            break;
        case RecordType::Imu:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            ImuSample sample = readImu(reader);
            if (!reader.ok()) continue;
            frame.sensors.imu = sample;
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Radar:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            RadarFrame radar = readRadarFrame(reader);
            if (!reader.ok()) continue;
            frame.sensors.radar = std::move(radar);
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Lidar:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            LidarScan scan = readLidarScan(reader);
            if (!reader.ok()) continue;
            frame.sensors.lidar = std::move(scan);
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Tof:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            TofScan scan = readTofScan(reader);
            if (!reader.ok()) continue;
            frame.sensors.tof = std::move(scan);
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Uwb:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            UwbMeasurement sample = readUwb(reader);
            if (!reader.ok()) continue;
            frame.sensors.uwb.push_back(sample);
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Novelda:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            NoveldaMeasurement sample = readNovelda(reader);
            if (!reader.ok()) continue;
            frame.sensors.novelda.push_back(sample);
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Wallpad:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            PresenceZone zone = readZone(reader);
            if (!reader.ok()) continue;
            frame.sensors.wallpad_zones.push_back(zone);
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::WristOrientation:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            WristOrientation orientation{};
            orientation.timestamp_ms = frame_timestamp;
            orientation.wrist_yaw_deg = reader.f32();
            orientation.wrist_pitch_deg = reader.f32();
            if (!reader.ok()) continue;
            frame.sensors.wrist_orientation = orientation;
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Ultrasonic:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            UltrasonicMeasurement sample = readUltrasonic(reader);
            if (!reader.ok()) continue;
            frame.sensors.ultrasonic = sample;
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Marker:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            MarkerObservation sample = readMarker(reader);
            if (!reader.ok()) continue;
            frame.sensors.marker = sample;
            saw_sensor = true;
            ++stats_.sensor_records;
            break;
        }
        case RecordType::Tracks:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            std::vector<Track> tracks = readTracks(reader);
            if (!reader.ok()) continue;
            pending_tracks = std::move(tracks);
            ++stats_.track_snapshots;
            break;
        }
        case RecordType::World:
        {
            frame_id = reader.u32();
            frame_timestamp = reader.u64();
            WorldModel world;
            const bool parsed = readWorld(reader, world);
            if (!parsed || !reader.ok()) continue;
            pending_world = std::move(world);
            ++stats_.world_snapshots;
            break;
        }
        case RecordType::FrameEnd:
        {
            const uint32_t end_frame_id = reader.u32();
            const uint64_t end_timestamp = reader.u64();
            (void)reader.u32();
            (void)reader.u32();
            if (!reader.ok()) continue;
            frame.sensors.frame_id = saw_sensor ? frame_id : end_frame_id;
            frame.sensors.timestamp_ms = frame_timestamp != 0 ? frame_timestamp : end_timestamp;
            frame.reference_world = std::move(pending_world);
            frame.reference_tracks = std::move(pending_tracks);
            frame.schematic = std::move(pending_schematic);
            ++stats_.frames;
            if (stats_.frames == 1) stats_.first_timestamp_ms = frame.sensors.timestamp_ms;
            stats_.last_timestamp_ms = frame.sensors.timestamp_ms;
            return true;
        }
        case RecordType::Schematic:
        {
            std::string text = reader.string();
            if (!reader.ok()) continue;
            pending_schematic = std::move(text);
            break;
        }
        default:
            break;
        }
    }
}

} // namespace sa
