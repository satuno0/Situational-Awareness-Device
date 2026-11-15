#include "serialization.hpp"

#include <bit>
#include <cmath>
#include <cstring>

namespace sa {
namespace {

template <typename T>
void appendLittle(std::vector<uint8_t>& out, T value)
{
    using U = std::make_unsigned_t<T>;
    U v = static_cast<U>(value);
    for (std::size_t i = 0; i < sizeof(T); ++i)
    {
        out.push_back(static_cast<uint8_t>(v & 0xFFu));
        v >>= 8;
    }
}

} // namespace

void BinaryWriter::u8(uint8_t value) { data_.push_back(value); }
void BinaryWriter::u16(uint16_t value) { appendLittle(data_, value); }
void BinaryWriter::u32(uint32_t value) { appendLittle(data_, value); }
void BinaryWriter::u64(uint64_t value) { appendLittle(data_, value); }
void BinaryWriter::i32(int32_t value) { appendLittle(data_, value); }
void BinaryWriter::f32(float value) { appendLittle(data_, std::bit_cast<uint32_t>(value)); }
void BinaryWriter::boolean(bool value) { u8(value ? 1u : 0u); }
void BinaryWriter::string(const std::string& value)
{
    u32(static_cast<uint32_t>(value.size()));
    bytes(value.data(), value.size());
}
void BinaryWriter::bytes(const void* data, std::size_t size)
{
    const auto* p = static_cast<const uint8_t*>(data);
    data_.insert(data_.end(), p, p + size);
}
void BinaryWriter::vec3(const Vec3& value) { f32(value.x); f32(value.y); f32(value.z); }
void BinaryWriter::quaternion(const Quaternion& value) { f32(value.w); f32(value.x); f32(value.y); f32(value.z); }
void BinaryWriter::pose(const Pose2D& value) { f32(value.x); f32(value.y); f32(value.yaw_deg); }

BinaryReader::BinaryReader(const uint8_t* data, std::size_t size) : data_(data), size_(size) {}
BinaryReader::BinaryReader(const std::vector<uint8_t>& data) : data_(data.data()), size_(data.size()) {}

bool BinaryReader::require(std::size_t count)
{
    if (!ok_ || offset_ + count > size_)
    {
        ok_ = false;
        return false;
    }
    return true;
}

uint8_t BinaryReader::u8()
{
    if (!require(1)) return 0;
    return data_[offset_++];
}

uint16_t BinaryReader::u16()
{
    if (!require(2)) return 0;
    uint16_t v = static_cast<uint16_t>(data_[offset_]) |
                 static_cast<uint16_t>(data_[offset_ + 1] << 8);
    offset_ += 2;
    return v;
}

uint32_t BinaryReader::u32()
{
    if (!require(4)) return 0;
    uint32_t v = static_cast<uint32_t>(data_[offset_]) |
                 (static_cast<uint32_t>(data_[offset_ + 1]) << 8) |
                 (static_cast<uint32_t>(data_[offset_ + 2]) << 16) |
                 (static_cast<uint32_t>(data_[offset_ + 3]) << 24);
    offset_ += 4;
    return v;
}

uint64_t BinaryReader::u64()
{
    if (!require(8)) return 0;
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(data_[offset_ + i]) << (8 * i);
    offset_ += 8;
    return v;
}

int32_t BinaryReader::i32() { return static_cast<int32_t>(u32()); }
float BinaryReader::f32() { return std::bit_cast<float>(u32()); }
bool BinaryReader::boolean() { return u8() != 0; }

std::string BinaryReader::string()
{
    const uint32_t length = u32();
    if (!ok_ || length > 1024u * 1024u || !require(length)) return {};
    std::string out(reinterpret_cast<const char*>(data_ + offset_), length);
    offset_ += length;
    return out;
}

bool BinaryReader::bytes(void* out, std::size_t size)
{
    if (!require(size)) return false;
    std::memcpy(out, data_ + offset_, size);
    offset_ += size;
    return true;
}

Vec3 BinaryReader::vec3() { return {f32(), f32(), f32()}; }
Quaternion BinaryReader::quaternion() { return {f32(), f32(), f32(), f32()}; }
Pose2D BinaryReader::pose() { return {f32(), f32(), f32()}; }

uint32_t crc32(const uint8_t* data, std::size_t size)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

void writeSensorHealth(BinaryWriter& writer, const SensorHealth& health)
{
    writer.boolean(health.imu);
    writer.boolean(health.radar);
    writer.boolean(health.lidar);
    writer.boolean(health.tof);
    writer.boolean(health.uwb);
    writer.boolean(health.novelda);
    writer.boolean(health.ultrasonic);
    writer.f32(health.health_score);
}

SensorHealth readSensorHealth(BinaryReader& reader)
{
    SensorHealth health{};
    health.imu = reader.boolean();
    health.radar = reader.boolean();
    health.lidar = reader.boolean();
    health.tof = reader.boolean();
    health.uwb = reader.boolean();
    health.novelda = reader.boolean();
    health.ultrasonic = reader.boolean();
    health.health_score = reader.f32();
    return health;
}

void writeDetection(BinaryWriter& writer, const Detection& detection)
{
    writer.vec3(detection.position);
    writer.vec3(detection.velocity);
    writer.f32(detection.snr);
    writer.f32(detection.range);
    writer.boolean(detection.from_radar);
}

Detection readDetection(BinaryReader& reader)
{
    Detection detection{};
    detection.position = reader.vec3();
    detection.velocity = reader.vec3();
    detection.snr = reader.f32();
    detection.range = reader.f32();
    detection.from_radar = reader.boolean();
    return detection;
}

void writeImu(BinaryWriter& writer, const ImuSample& sample)
{
    writer.u64(sample.timestamp_ms);
    writer.vec3(sample.accel_mps2);
    writer.vec3(sample.gyro_radps);
    writer.f32(sample.temperature_c);
}

ImuSample readImu(BinaryReader& reader)
{
    ImuSample sample{};
    sample.timestamp_ms = reader.u64();
    sample.accel_mps2 = reader.vec3();
    sample.gyro_radps = reader.vec3();
    sample.temperature_c = reader.f32();
    return sample;
}

void writeRadarFrame(BinaryWriter& writer, const RadarFrame& frame)
{
    writer.u64(frame.timestamp_ms);
    writer.u32(static_cast<uint32_t>(frame.detections.size()));
    for (const auto& detection : frame.detections) writeDetection(writer, detection);
}

RadarFrame readRadarFrame(BinaryReader& reader)
{
    RadarFrame frame{};
    frame.timestamp_ms = reader.u64();
    const uint32_t count = reader.u32();
    if (count > 100000u)
    {
        reader.invalidate();
        return frame;
    }
    frame.detections.reserve(count);
    for (uint32_t i = 0; i < count && reader.ok(); ++i) frame.detections.push_back(readDetection(reader));
    return frame;
}

void writeLidarScan(BinaryWriter& writer, const LidarScan& scan)
{
    writer.u64(scan.timestamp_ms);
    writer.vec3(scan.origin);
    writer.u32(static_cast<uint32_t>(scan.points.size()));
    for (const auto& point : scan.points) writer.vec3(point);
    writer.boolean(scan.burst_active);
    writer.boolean(scan.burst_complete);
}

LidarScan readLidarScan(BinaryReader& reader)
{
    LidarScan scan{};
    scan.timestamp_ms = reader.u64();
    scan.origin = reader.vec3();
    const uint32_t count = reader.u32();
    if (count > kMaxLidarPointsPerScan)
    {
        reader.invalidate();
        return scan;
    }
    scan.points.reserve(count);
    for (uint32_t i = 0; i < count && reader.ok(); ++i) scan.points.push_back(reader.vec3());
    scan.burst_active = reader.boolean();
    scan.burst_complete = reader.boolean();
    return scan;
}

void writeTofScan(BinaryWriter& writer, const TofScan& scan)
{
    writer.u64(scan.timestamp_ms);
    writer.u32(static_cast<uint32_t>(scan.rays.size()));
    for (const auto& ray : scan.rays)
    {
        writer.vec3(ray.origin);
        writer.vec3(ray.endpoint);
    }
}

TofScan readTofScan(BinaryReader& reader)
{
    TofScan scan{};
    scan.timestamp_ms = reader.u64();
    const uint32_t count = reader.u32();
    if (count > kMaxTofRaysPerScan)
    {
        reader.invalidate();
        return scan;
    }
    scan.rays.reserve(count);
    for (uint32_t i = 0; i < count && reader.ok(); ++i)
        scan.rays.push_back(SurfaceRay{reader.vec3(), reader.vec3()});
    return scan;
}

void writeUwb(BinaryWriter& writer, const UwbMeasurement& sample)
{
    writer.u64(sample.timestamp_ms);
    writer.u16(sample.tag_id);
    writer.f32(sample.range_m);
    writer.boolean(sample.has_aoa);
    writer.f32(sample.aoa_rad);
    writer.boolean(sample.has_range_rate);
    writer.f32(sample.range_rate_mps);
    writer.f32(sample.confidence);
    writer.boolean(sample.is_fixed_anchor);
    writer.f32(sample.anchor_x_m);
    writer.f32(sample.anchor_y_m);
}

UwbMeasurement readUwb(BinaryReader& reader)
{
    UwbMeasurement sample{};
    sample.timestamp_ms = reader.u64();
    sample.tag_id = reader.u16();
    sample.range_m = reader.f32();
    sample.has_aoa = reader.boolean();
    sample.aoa_rad = reader.f32();
    sample.has_range_rate = reader.boolean();
    sample.range_rate_mps = reader.f32();
    sample.confidence = reader.f32();
    sample.is_fixed_anchor = reader.boolean();
    sample.anchor_x_m = reader.f32();
    sample.anchor_y_m = reader.f32();
    return sample;
}

void writeNovelda(BinaryWriter& writer, const NoveldaMeasurement& sample)
{
    writer.u64(sample.timestamp_ms);
    writer.f32(sample.range_m);
    writer.f32(sample.bearing_center_rad);
    writer.f32(sample.bearing_width_rad);
    writer.f32(sample.motion);
    writer.f32(sample.breathing);
    writer.f32(sample.confidence);
    writer.boolean(sample.through_barrier);
}

NoveldaMeasurement readNovelda(BinaryReader& reader)
{
    NoveldaMeasurement sample{};
    sample.timestamp_ms = reader.u64();
    sample.range_m = reader.f32();
    sample.bearing_center_rad = reader.f32();
    sample.bearing_width_rad = reader.f32();
    sample.motion = reader.f32();
    sample.breathing = reader.f32();
    sample.confidence = reader.f32();
    sample.through_barrier = reader.boolean();
    return sample;
}

void writeUltrasonic(BinaryWriter& writer, const UltrasonicMeasurement& sample)
{
    writer.u64(sample.timestamp_ms);
    writer.f32(sample.range_m);
    writer.boolean(sample.valid);
}

UltrasonicMeasurement readUltrasonic(BinaryReader& reader)
{
    UltrasonicMeasurement sample{};
    sample.timestamp_ms = reader.u64();
    sample.range_m = reader.f32();
    sample.valid = reader.boolean();
    return sample;
}

void writeMarker(BinaryWriter& writer, const MarkerObservation& sample)
{
    writer.u64(sample.timestamp_ms);
    writer.f32(sample.bearing_rad);
    writer.f32(sample.confidence);
    writer.f32(sample.area_ratio);
    writer.boolean(sample.valid);
}

MarkerObservation readMarker(BinaryReader& reader)
{
    MarkerObservation sample{};
    sample.timestamp_ms = reader.u64();
    sample.bearing_rad = reader.f32();
    sample.confidence = reader.f32();
    sample.area_ratio = reader.f32();
    sample.valid = reader.boolean();
    return sample;
}

void writeZone(BinaryWriter& writer, const PresenceZone& zone)
{
    writer.f32(zone.range_min);
    writer.f32(zone.range_max);
    writer.f32(zone.bearing_center_rad);
    writer.f32(zone.bearing_width_rad);
    writer.f32(zone.confidence);
}

PresenceZone readZone(BinaryReader& reader)
{
    PresenceZone zone{};
    zone.range_min = reader.f32();
    zone.range_max = reader.f32();
    zone.bearing_center_rad = reader.f32();
    zone.bearing_width_rad = reader.f32();
    zone.confidence = reader.f32();
    return zone;
}

void writeTrack(BinaryWriter& writer, const Track& track)
{
    writer.u32(track.id);
    for (float value : track.filter.state()) writer.f32(value);
    for (float value : track.filter.covariance()) writer.f32(value);
    writer.u64(track.filter.lastUpdateMs());
    writer.boolean(track.identity.tagged_safe);
    writer.u16(track.identity.tag_id);
    writer.f32(track.identity.confidence);
    writer.u8(static_cast<uint8_t>(track.state));
    writer.f32(track.existence_confidence);
    writer.f32(track.position_confidence);
    writer.f32(track.position_sigma_m);
    writer.boolean(track.visible);
}

Track readTrack(BinaryReader& reader)
{
    Track track{};
    track.id = reader.u32();
    std::array<float, KalmanTrack::kStateSize> state{};
    std::array<float, KalmanTrack::kStateSize * KalmanTrack::kStateSize> covariance{};
    for (float& value : state) value = reader.f32();
    for (float& value : covariance) value = reader.f32();
    const uint64_t last_update = reader.u64();
    track.filter.setStateAndCovariance(state, covariance, last_update);
    track.identity.tagged_safe = reader.boolean();
    track.identity.tag_id = reader.u16();
    track.identity.confidence = reader.f32();
    const uint8_t state_value = reader.u8();
    track.state = state_value <= static_cast<uint8_t>(TrackState::EXPIRED)
                      ? static_cast<TrackState>(state_value)
                      : TrackState::EXPIRED;
    track.existence_confidence = reader.f32();
    track.position_confidence = reader.f32();
    track.position_sigma_m = reader.f32();
    track.visible = reader.boolean();
    return track;
}

void writeTracks(BinaryWriter& writer, const std::vector<Track>& tracks)
{
    writer.u32(static_cast<uint32_t>(tracks.size()));
    for (const auto& track : tracks) writeTrack(writer, track);
}

std::vector<Track> readTracks(BinaryReader& reader)
{
    const uint32_t count = reader.u32();
    if (count > 4096u)
    {
        reader.invalidate();
        return {};
    }
    std::vector<Track> tracks;
    tracks.reserve(count);
    for (uint32_t i = 0; i < count && reader.ok(); ++i) tracks.push_back(readTrack(reader));
    return tracks;
}

void writeGrid(BinaryWriter& writer, const OccupancyGrid& grid)
{
    const auto& config = grid.config();
    writer.u32(static_cast<uint32_t>(config.width));
    writer.u32(static_cast<uint32_t>(config.height));
    writer.f32(config.cell_size);
    writer.f32(config.range_m);
    const auto encoded = grid.encodedProbabilities();
    writer.u32(static_cast<uint32_t>(encoded.size()));
    for (uint16_t value : encoded) writer.u16(value);
}

bool readGrid(BinaryReader& reader, OccupancyGrid& grid)
{
    GridConfig config;
    config.width = static_cast<int>(reader.u32());
    config.height = static_cast<int>(reader.u32());
    config.cell_size = reader.f32();
    config.range_m = reader.f32();
    const uint32_t count = reader.u32();
    if (!reader.ok() || config.width <= 0 || config.height <= 0 || config.width > 4096 || config.height > 4096 ||
        config.cell_size <= 0.0f)
    {
        return false;
    }
    const std::size_t expected = static_cast<std::size_t>(config.width) * static_cast<std::size_t>(config.height);
    if (count != expected || count > 16u * 1024u * 1024u) return false;
    std::vector<uint16_t> values;
    values.reserve(count);
    for (uint32_t i = 0; i < count && reader.ok(); ++i) values.push_back(reader.u16());
    return reader.ok() && grid.loadEncodedProbabilities(config, values);
}

void writeWorld(BinaryWriter& writer, const WorldModel& world)
{
    writer.u64(world.timestamp_ms);
    writer.pose(world.pose);
    writer.f32(world.environment_confidence);
    writeSensorHealth(writer, world.health);
    writeGrid(writer, world.occupancy);
    writer.u32(static_cast<uint32_t>(world.static_objects.size()));
    for (const auto& object : world.static_objects)
    {
        writer.vec3(object.position);
        writer.f32(object.confidence);
    }
    writeTracks(writer, world.tracks);
    writer.u32(static_cast<uint32_t>(world.zones.size()));
    for (const auto& zone : world.zones) writeZone(writer, zone);
}

bool readWorld(BinaryReader& reader, WorldModel& world)
{
    world = WorldModel{};
    world.timestamp_ms = reader.u64();
    world.pose = reader.pose();
    world.environment_confidence = reader.f32();
    world.health = readSensorHealth(reader);
    if (!readGrid(reader, world.occupancy)) return false;
    const uint32_t object_count = reader.u32();
    if (object_count > 100000u) return false;
    world.static_objects.reserve(object_count);
    for (uint32_t i = 0; i < object_count && reader.ok(); ++i)
    {
        MapObject object{};
        object.position = reader.vec3();
        object.confidence = reader.f32();
        world.static_objects.push_back(object);
    }
    world.tracks = readTracks(reader);
    const uint32_t zone_count = reader.u32();
    if (zone_count > 10000u) return false;
    world.zones.reserve(zone_count);
    for (uint32_t i = 0; i < zone_count && reader.ok(); ++i) world.zones.push_back(readZone(reader));
    return reader.ok();
}

} // namespace sa
