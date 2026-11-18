#include "glove_raycaster.hpp"

#include <cmath>

namespace sa {
namespace {

constexpr uint8_t kPacketMagic = 0x5B;
constexpr std::size_t kPacketSize = 10;

int16_t readInt16LittleEndian(const uint8_t* data)
{
    int32_t value = static_cast<int32_t>(data[0]) |
                    (static_cast<int32_t>(data[1]) << 8);
    if (value >= 0x8000) value -= 0x10000;
    return static_cast<int16_t>(value);
}

float cross2d(float ax, float ay, float bx, float by)
{
    return ax * by - ay * bx;
}

float nearestObstructionRange(const WorldModel& world, float ray_x, float ray_y)
{
    float nearest = GloveRaycaster::kMaxRangeM;
    for (const MapWall& wall : world.walls)
    {
        const float segment_x = wall.end.x - wall.start.x;
        const float segment_y = wall.end.y - wall.start.y;
        const float offset_x = wall.start.x - world.pose.x;
        const float offset_y = wall.start.y - world.pose.y;
        const float denominator = cross2d(ray_x, ray_y, segment_x, segment_y);
        if (std::abs(denominator) < 1.0e-6f) continue;

        const float range = cross2d(offset_x, offset_y, segment_x, segment_y) / denominator;
        const float segment_fraction = cross2d(offset_x, offset_y, ray_x, ray_y) / denominator;
        if (range > 0.0f && range < nearest &&
            segment_fraction >= 0.0f && segment_fraction <= 1.0f)
            nearest = range;
    }

    const GridConfig& grid = world.occupancy.config();
    const float step = grid.cell_size * 0.5f;
    if (std::isfinite(step) && step > 0.0f)
    {
        for (float range = step; range < nearest; range += step)
        {
            int cell_x = 0;
            int cell_y = 0;
            const float x = world.pose.x + ray_x * range;
            const float y = world.pose.y + ray_y * range;
            if (world.occupancy.worldToCell(x, y, cell_x, cell_y) &&
                world.occupancy.probability(cell_x, cell_y) >= 0.65f)
            {
                nearest = range;
                break;
            }
        }
    }
    return nearest;
}

} // namespace

bool parseGlovePosePacket(const uint8_t* data,
                          std::size_t size,
                          GlovePosePacket& packet,
                          std::string& error)
{
    error.clear();
    if (data == nullptr || size != kPacketSize)
    {
        error = "invalid glove packet size (expected 10 bytes)";
        return false;
    }

    uint8_t checksum = 0;
    for (std::size_t i = 0; i < kPacketSize - 1; ++i)
        checksum ^= data[i];
    if (data[0] != kPacketMagic || checksum != data[kPacketSize - 1])
    {
        error = "invalid glove packet magic or checksum";
        return false;
    }

    const int16_t yaw_cdeg = readInt16LittleEndian(data + 2);
    const int16_t pitch_cdeg = readInt16LittleEndian(data + 4);
    if (yaw_cdeg < -18000 || yaw_cdeg > 18000 ||
        pitch_cdeg < -9000 || pitch_cdeg > 9000)
    {
        error = "glove packet orientation is outside its valid range";
        return false;
    }

    packet.sequence = data[1];
    packet.yaw_deg = static_cast<float>(yaw_cdeg) / 100.0f;
    packet.pitch_deg = static_cast<float>(pitch_cdeg) / 100.0f;
    packet.trigger_held = (data[6] & 0x01) != 0;
    packet.zero_heading_requested = (data[7] & 0x01) != 0;
    packet.events = data[8];
    return true;
}

GloveRayState GloveRaycaster::update(const WorldModel& world,
                                     const std::optional<GlovePosePacket>& packet,
                                     uint64_t timestamp_ms)
{
    if (packet)
    {
        const bool previous_packet_stale =
            !latest_packet_ || timestamp_ms < latest_packet_timestamp_ms_ ||
            timestamp_ms - latest_packet_timestamp_ms_ > kPacketTimeoutMs;
        if (previous_packet_stale) button_was_pressed_ = false;
        if (packet->trigger_held && !button_was_pressed_)
            scan_enabled_ = !scan_enabled_;
        button_was_pressed_ = packet->trigger_held;
        latest_packet_ = packet;
        latest_packet_timestamp_ms_ = timestamp_ms;
        if (packet->zero_heading_requested)
            zero_heading_offset_deg_ = wrapDeg(-packet->yaw_deg);
    }

    GloveRayState state{};
    if (!latest_packet_ || !scan_enabled_ ||
        timestamp_ms < latest_packet_timestamp_ms_ ||
        timestamp_ms - latest_packet_timestamp_ms_ > kPacketTimeoutMs ||
        !std::isfinite(latest_packet_->yaw_deg) ||
        !std::isfinite(world.pose.yaw_deg) ||
        !std::isfinite(world.pose.x) || !std::isfinite(world.pose.y))
        return state;

    state.active = true;
    state.yaw_deg = wrapDeg(world.pose.yaw_deg + latest_packet_->yaw_deg +
                            zero_heading_offset_deg_);

    const float yaw_rad = degToRad(state.yaw_deg);
    const float ray_x = std::sin(yaw_rad);
    const float ray_y = std::cos(yaw_rad);
    const float obstruction_range = nearestObstructionRange(world, ray_x, ray_y);
    float nearest_range = kMaxRangeM;
    for (const Track& track : world.tracks)
    {
        if (!track.visible || track.state != TrackState::VISIBLE) continue;

        const Vec3 position = track.filter.position();
        const float dx = position.x - world.pose.x;
        const float dy = position.y - world.pose.y;
        const float range = std::sqrt(dx * dx + dy * dy);
        if (!std::isfinite(range) || range <= 1.0e-4f || range > kMaxRangeM ||
            track.state != TrackState::VISIBLE ||
            range > obstruction_range + 0.15f)
            continue;

        const float bearing_deg = radToDeg(std::atan2(dx, dy));
        const float error_deg = std::abs(wrapDeg(bearing_deg - state.yaw_deg));
        if (error_deg > kHalfAngleDeg || range >= nearest_range) continue;

        nearest_range = range;
        state.locked_target_id = track.id;
        state.locked_dist_m = range;
    }
    return state;
}

void GloveRaycaster::reset()
{
    latest_packet_.reset();
    latest_packet_timestamp_ms_ = 0;
    zero_heading_offset_deg_ = 0.0f;
    scan_enabled_ = false;
    button_was_pressed_ = false;
}

void GloveRaycaster::disableScan()
{
    scan_enabled_ = false;
    button_was_pressed_ = false;
}

} // namespace sa
