#pragma once

#include "types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sa {

class BinaryWriter
{
public:
    void u8(uint8_t value);
    void u16(uint16_t value);
    void u32(uint32_t value);
    void u64(uint64_t value);
    void i32(int32_t value);
    void f32(float value);
    void boolean(bool value);
    void string(const std::string& value);
    void bytes(const void* data, std::size_t size);
    void vec3(const Vec3& value);
    void quaternion(const Quaternion& value);
    void pose(const Pose2D& value);

    const std::vector<uint8_t>& data() const { return data_; }
    std::vector<uint8_t>& data() { return data_; }
    std::size_t size() const { return data_.size(); }
    void clear() { data_.clear(); }

private:
    std::vector<uint8_t> data_;
};

class BinaryReader
{
public:
    BinaryReader(const uint8_t* data, std::size_t size);
    explicit BinaryReader(const std::vector<uint8_t>& data);

    bool ok() const { return ok_; }
    std::size_t remaining() const { return ok_ ? size_ - offset_ : 0; }
    std::size_t position() const { return offset_; }

    // Marks the reader as failed. Used when a length field is implausible even
    // though the payload buffer itself has not underflowed yet.
    void invalidate() { ok_ = false; }

    uint8_t u8();
    uint16_t u16();
    uint32_t u32();
    uint64_t u64();
    int32_t i32();
    float f32();
    bool boolean();
    std::string string();
    bool bytes(void* out, std::size_t size);
    Vec3 vec3();
    Quaternion quaternion();
    Pose2D pose();

private:
    bool require(std::size_t count);

    const uint8_t* data_;
    std::size_t size_;
    std::size_t offset_ = 0;
    bool ok_ = true;
};

uint32_t crc32(const uint8_t* data, std::size_t size);

void writeSensorHealth(BinaryWriter& writer, const SensorHealth& health);
SensorHealth readSensorHealth(BinaryReader& reader);

void writeDetection(BinaryWriter& writer, const Detection& detection);
Detection readDetection(BinaryReader& reader);

void writeImu(BinaryWriter& writer, const ImuSample& sample);
ImuSample readImu(BinaryReader& reader);

void writeRadarFrame(BinaryWriter& writer, const RadarFrame& frame);
RadarFrame readRadarFrame(BinaryReader& reader);

void writeUwb(BinaryWriter& writer, const UwbMeasurement& sample);
UwbMeasurement readUwb(BinaryReader& reader);

void writeNovelda(BinaryWriter& writer, const NoveldaMeasurement& sample);
NoveldaMeasurement readNovelda(BinaryReader& reader);

void writeUltrasonic(BinaryWriter& writer, const UltrasonicMeasurement& sample);
UltrasonicMeasurement readUltrasonic(BinaryReader& reader);

void writeMarker(BinaryWriter& writer, const MarkerObservation& sample);
MarkerObservation readMarker(BinaryReader& reader);

void writeZone(BinaryWriter& writer, const PresenceZone& zone);
PresenceZone readZone(BinaryReader& reader);

void writeTrack(BinaryWriter& writer, const Track& track);
Track readTrack(BinaryReader& reader);

void writeTracks(BinaryWriter& writer, const std::vector<Track>& tracks);
std::vector<Track> readTracks(BinaryReader& reader);

void writeGrid(BinaryWriter& writer, const OccupancyGrid& grid);
bool readGrid(BinaryReader& reader, OccupancyGrid& grid);

void writeWorld(BinaryWriter& writer, const WorldModel& world);
bool readWorld(BinaryReader& reader, WorldModel& world);

} // namespace sa
