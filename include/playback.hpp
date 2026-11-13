#pragma once

#include "recorder.hpp"
#include "types.hpp"

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace sa {

struct PlaybackStats
{
    uint64_t records = 0;
    uint64_t frames = 0;
    uint64_t sensor_records = 0;
    uint64_t world_snapshots = 0;
    uint64_t track_snapshots = 0;
    uint64_t crc_errors = 0;
    bool truncated = false;
    uint64_t first_timestamp_ms = 0;
    uint64_t last_timestamp_ms = 0;
};

struct PlaybackFrame
{
    SensorFrame sensors;
    std::optional<WorldModel> reference_world;
    std::optional<std::vector<Track>> reference_tracks;

    // Set when a schematic record becomes active in this frame (a mid-session
    // upload). The initial preface schematic is exposed by schematicText().
    std::optional<std::string> schematic;
};

class Playback
{
public:
    Playback();
    explicit Playback(const std::string& path);
    ~Playback();

    bool open(const std::string& path, std::string& error);
    void close();
    bool isOpen() const { return file_.is_open(); }
    bool rewind(std::string& error);

    // Emits the replay source of truth (raw sensors). Reference world/track
    // snapshots are attached when present and are never used to synthesize
    // outputs.
    bool next(PlaybackFrame& frame);
    bool eof() const { return eof_; }

    const PlaybackStats& stats() const { return stats_; }
    const std::string& error() const { return error_; }

    // Initial schematic embedded in the session preface, if any. Later changes
    // are delivered through PlaybackFrame::schematic.
    const std::string& schematicText() const { return schematic_text_; }

private:
    bool readExactly(void* data, std::size_t size);
    bool readHeader(uint32_t& type, uint32_t& size, uint32_t& crc, bool& clean_eof);
    bool readPreamble();

    std::string path_;
    std::ifstream file_;
    PlaybackStats stats_;
    std::string error_;
    std::string schematic_text_;
    std::uint32_t header_size_ = 64;
    bool eof_ = false;
};

} // namespace sa
