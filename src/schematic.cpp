#include "schematic.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace sa {
namespace {

constexpr std::size_t kMaxWalls = 4096;
constexpr std::size_t kMaxRooms = 1024;
constexpr std::size_t kMaxMarks = 1024;

std::string lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

bool readFloat(std::istringstream& stream, float& value)
{
    return static_cast<bool>(stream >> value) && std::isfinite(value);
}

} // namespace

void Schematic::clear()
{
    origin_x_ = 0.0f;
    origin_y_ = 0.0f;
    yaw_deg_ = 0.0f;
    scale_ = 1.0f;
    raw_walls_.clear();
    raw_rooms_.clear();
    raw_marks_.clear();
    walls_.clear();
    doors_.clear();
    rooms_.clear();
    marks_.clear();
}

bool Schematic::loadFile(const std::string& path, std::string& error)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        error = "cannot open schematic: " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return loadText(buffer.str(), error);
}

bool Schematic::loadText(const std::string& text, std::string& error)
{
    clear();

    std::istringstream input(text);
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) line.resize(comment);

        std::istringstream stream(line);
        std::string command;
        if (!(stream >> command)) continue;
        command = lower(command);

        auto fail = [&](const std::string& message) {
            error = "schematic line " + std::to_string(line_number) + ": " + message;
            clear();
            return false;
        };

        if (command == "origin")
        {
            float x = 0.0f;
            float y = 0.0f;
            if (!readFloat(stream, x) || !readFloat(stream, y)) return fail("origin needs x y");
            origin_x_ = x;
            origin_y_ = y;
        }
        else if (command == "yaw")
        {
            float yaw = 0.0f;
            if (!readFloat(stream, yaw)) return fail("yaw needs degrees");
            yaw_deg_ = yaw;
        }
        else if (command == "scale")
        {
            float scale = 0.0f;
            if (!readFloat(stream, scale) || scale <= 0.0f || scale > 1000.0f)
                return fail("scale must be in (0, 1000]");
            scale_ = scale;
        }
        else if (command == "wall" || command == "door")
        {
            RawWall wall;
            if (!readFloat(stream, wall.a.x) || !readFloat(stream, wall.a.y) ||
                !readFloat(stream, wall.b.x) || !readFloat(stream, wall.b.y))
                return fail("wall needs x1 y1 x2 y2 [height]");
            float height = 0.0f;
            if (stream >> height)
            {
                if (!std::isfinite(height) || height <= 0.0f) return fail("wall height must be positive");
                wall.height = height;
            }
            wall.door = command == "door";
            if (wall.door) wall.height = std::min(wall.height, 2.10f);
            raw_walls_.push_back(wall);
            if (raw_walls_.size() > kMaxWalls) return fail("too many wall segments");
        }
        else if (command == "room")
        {
            RawRoom room;
            if (!(stream >> room.name)) return fail("room needs a name");
            if (!readFloat(stream, room.min_x) || !readFloat(stream, room.min_y) ||
                !readFloat(stream, room.max_x) || !readFloat(stream, room.max_y))
                return fail("room needs name x1 y1 x2 y2");
            raw_rooms_.push_back(room);
            if (raw_rooms_.size() > kMaxRooms) return fail("too many rooms");
        }
        else if (command == "label" || command == "mark" || command == "landmark")
        {
            RawMark mark;
            if (!(stream >> mark.name)) return fail("label needs a name");
            if (!readFloat(stream, mark.position.x) || !readFloat(stream, mark.position.y))
                return fail("label needs name x y");
            raw_marks_.push_back(mark);
            if (raw_marks_.size() > kMaxMarks) return fail("too many labels");
        }
        else
        {
            return fail("unknown command '" + command + "'");
        }
    }

    if (raw_walls_.empty() && raw_rooms_.empty() && raw_marks_.empty())
    {
        error = "schematic contains no walls, rooms or labels";
        clear();
        return false;
    }

    for (const auto& raw : raw_walls_)
    {
        MapWall wall{};
        transformPoint(raw.a, wall.start);
        transformPoint(raw.b, wall.end);
        wall.height = raw.height;
        if (raw.door) doors_.push_back(wall);
        else walls_.push_back(wall);
    }

    for (const auto& raw : raw_rooms_)
    {
        const RawPoint corners[4] = {
            {raw.min_x, raw.min_y},
            {raw.max_x, raw.min_y},
            {raw.max_x, raw.max_y},
            {raw.min_x, raw.max_y},
        };
        MapRoom room{};
        room.name = raw.name;
        room.min_x = std::numeric_limits<float>::infinity();
        room.min_y = std::numeric_limits<float>::infinity();
        room.max_x = -std::numeric_limits<float>::infinity();
        room.max_y = -std::numeric_limits<float>::infinity();
        for (const auto& corner : corners)
        {
            Vec3 world{};
            transformPoint(corner, world);
            room.min_x = std::min(room.min_x, world.x);
            room.min_y = std::min(room.min_y, world.y);
            room.max_x = std::max(room.max_x, world.x);
            room.max_y = std::max(room.max_y, world.y);
        }
        rooms_.push_back(room);
    }

    for (const auto& raw : raw_marks_)
    {
        SchematicMark mark{};
        mark.name = raw.name;
        transformPoint(raw.position, mark.position);
        marks_.push_back(mark);
    }

    return true;
}

void Schematic::transformPoint(const RawPoint& raw, Vec3& world) const
{
    const float yaw_rad = degToRad(yaw_deg_);
    const float c = std::cos(yaw_rad);
    const float s = std::sin(yaw_rad);
    world.x = origin_x_ + scale_ * (c * raw.x - s * raw.y);
    world.y = origin_y_ + scale_ * (s * raw.x + c * raw.y);
    world.z = 0.0f;
}

void Schematic::applyToGrid(OccupancyGrid& grid, float wall_log_odds, float door_log_odds) const
{
    const float step = std::max(0.05f, grid.config().cell_size * 0.5f);
    const auto stamp = [&](const MapWall& wall, bool occupied) {
        const float dx = wall.end.x - wall.start.x;
        const float dy = wall.end.y - wall.start.y;
        const float distance = std::sqrt(dx * dx + dy * dy);
        const int steps = std::max(1, static_cast<int>(std::ceil(distance / step)));
        for (int i = 0; i <= steps; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(steps);
            const Vec3 point{wall.start.x + dx * t, wall.start.y + dy * t, 0.0f};
            if (occupied) grid.markOccupiedWorld(point, wall_log_odds);
            else grid.markFreeWorld(point, door_log_odds);
        }
    };
    for (const auto& wall : walls_) stamp(wall, true);
    for (const auto& door : doors_) stamp(door, false);
}

} // namespace sa
