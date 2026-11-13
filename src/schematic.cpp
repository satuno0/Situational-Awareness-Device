#include "schematic.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

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

std::string trim(std::string text)
{
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::string upper(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

bool parseFloat(const std::string& text, float& value)
{
    std::istringstream stream(text);
    if (!readFloat(stream, value)) return false;
    stream >> std::ws;
    return stream.eof();
}

bool parseInt(const std::string& text, int& value)
{
    std::istringstream stream(text);
    if (!(stream >> value)) return false;
    stream >> std::ws;
    return stream.eof();
}

float dxfUnitScale(int units)
{
    switch (units)
    {
    case 0: return 1.0f;
    case 1: return 0.0254f;
    case 2: return 0.3048f;
    case 3: return 1609.344f;
    case 4: return 0.001f;
    case 5: return 0.01f;
    case 6: return 1.0f;
    case 7: return 1000.0f;
    case 8: return 2.54e-8f;
    case 9: return 2.54e-5f;
    case 10: return 0.9144f;
    case 11: return 1.0e-10f;
    case 12: return 1.0e-9f;
    case 13: return 1.0e-6f;
    case 14: return 0.1f;
    case 15: return 10.0f;
    case 16: return 100.0f;
    case 17: return 1.0e9f;
    case 18: return 149597870700.0f;
    case 19: return 9.460730472e15f;
    case 20: return 3.085677581e16f;
    case 21: return 1200.0f / 3937.0f;
    case 22: return 1.0f / 12.0f * (1200.0f / 3937.0f);
    case 23: return 3.0f * (1200.0f / 3937.0f);
    case 24: return 5280.0f * (1200.0f / 3937.0f);
    default: return 0.0f;
    }
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
    const std::size_t extension = path.find_last_of('.');
    if (extension != std::string::npos &&
        lower(path.substr(extension)) == ".dxf")
        return loadDxfText(buffer.str(), error);
    return loadText(buffer.str(), error);
}

bool Schematic::loadDxfText(const std::string& text, std::string& error)
{
    struct Pair
    {
        int code;
        std::string value;
        std::size_t line;
    };

    clear();
    error.clear();

    auto fail = [&](std::size_t line, const std::string& message) {
        error = "DXF line " + std::to_string(line) + ": " + message;
        clear();
        return false;
    };

    std::istringstream input(text);
    std::vector<Pair> pairs;
    std::string code_line;
    std::string value_line;
    std::size_t line_number = 0;
    while (std::getline(input, code_line))
    {
        ++line_number;
        if (line_number == 1 && code_line.size() >= 3 &&
            static_cast<unsigned char>(code_line[0]) == 0xEF &&
            static_cast<unsigned char>(code_line[1]) == 0xBB &&
            static_cast<unsigned char>(code_line[2]) == 0xBF)
            code_line.erase(0, 3);

        if (!std::getline(input, value_line))
            return fail(line_number, "missing value for group code");
        ++line_number;

        int code = 0;
        if (!parseInt(trim(code_line), code))
            return fail(line_number - 1, "invalid group code");
        pairs.push_back({code, trim(value_line), line_number});
    }

    int units = 0;
    std::string section;
    for (std::size_t i = 0; i < pairs.size(); ++i)
    {
        const Pair& pair = pairs[i];
        if (pair.code == 0 && upper(pair.value) == "SECTION" &&
            i + 1 < pairs.size() && pairs[i + 1].code == 2)
        {
            section = upper(pairs[++i].value);
            continue;
        }
        if (pair.code == 0 && upper(pair.value) == "ENDSEC")
        {
            section.clear();
            continue;
        }
        if (section == "HEADER" && pair.code == 9 &&
            upper(pair.value) == "$INSUNITS")
        {
            if (i + 1 >= pairs.size() || pairs[i + 1].code != 70 ||
                !parseInt(pairs[i + 1].value, units))
                return fail(pair.line, "invalid $INSUNITS value");
        }
    }

    scale_ = dxfUnitScale(units);
    if (scale_ <= 0.0f)
        return fail(1, "unsupported $INSUNITS value " + std::to_string(units));

    bool found_entities_section = false;
    section.clear();
    for (std::size_t i = 0; i < pairs.size(); ++i)
    {
        const Pair& pair = pairs[i];
        if (pair.code == 0 && upper(pair.value) == "SECTION" &&
            i + 1 < pairs.size() && pairs[i + 1].code == 2)
        {
            section = upper(pairs[++i].value);
            if (section == "ENTITIES") found_entities_section = true;
            continue;
        }
        if (pair.code == 0 && upper(pair.value) == "ENDSEC")
        {
            section.clear();
            continue;
        }
        if (section != "ENTITIES" || pair.code != 0) continue;

        const std::string entity = upper(pair.value);
        const std::size_t data_begin = i + 1;
        std::size_t data_end = data_begin;
        while (data_end < pairs.size() && pairs[data_end].code != 0) ++data_end;

        if (entity == "LINE")
        {
            RawWall wall;
            bool has_x1 = false;
            bool has_y1 = false;
            bool has_x2 = false;
            bool has_y2 = false;
            for (std::size_t j = data_begin; j < data_end; ++j)
            {
                float value = 0.0f;
                if (pairs[j].code == 10 || pairs[j].code == 20 ||
                    pairs[j].code == 11 || pairs[j].code == 21)
                {
                    if (!parseFloat(pairs[j].value, value))
                        return fail(pairs[j].line, "invalid LINE coordinate");
                    if (pairs[j].code == 10) { wall.a.x = value; has_x1 = true; }
                    if (pairs[j].code == 20) { wall.a.y = value; has_y1 = true; }
                    if (pairs[j].code == 11) { wall.b.x = value; has_x2 = true; }
                    if (pairs[j].code == 21) { wall.b.y = value; has_y2 = true; }
                }
            }
            if (!has_x1 || !has_y1 || !has_x2 || !has_y2)
                return fail(pair.line, "LINE needs x1 y1 x2 y2");
            if (raw_walls_.size() >= kMaxWalls)
                return fail(pair.line, "too many DXF wall segments");
            raw_walls_.push_back(wall);
        }
        else if (entity == "LWPOLYLINE")
        {
            std::vector<RawPoint> vertices;
            int flags = 0;
            int declared_vertices = -1;
            for (std::size_t j = data_begin; j < data_end; ++j)
            {
                if (pairs[j].code == 70 && !parseInt(pairs[j].value, flags))
                    return fail(pairs[j].line, "invalid LWPOLYLINE flags");
                if (pairs[j].code == 90)
                {
                    if (!parseInt(pairs[j].value, declared_vertices) ||
                        declared_vertices < 0 ||
                        static_cast<std::size_t>(declared_vertices) > kMaxWalls + 1)
                        return fail(pairs[j].line, "invalid LWPOLYLINE vertex count");
                }
                if (pairs[j].code == 42)
                {
                    float bulge = 0.0f;
                    if (!parseFloat(pairs[j].value, bulge))
                        return fail(pairs[j].line, "invalid LWPOLYLINE bulge");
                    if (bulge != 0.0f)
                        return fail(pairs[j].line, "curved LWPOLYLINE segments are unsupported");
                }
                if (pairs[j].code != 10) continue;

                RawPoint vertex;
                if (!parseFloat(pairs[j].value, vertex.x) ||
                    j + 1 >= data_end || pairs[j + 1].code != 20 ||
                    !parseFloat(pairs[j + 1].value, vertex.y))
                    return fail(pairs[j].line, "invalid LWPOLYLINE vertex");
                if (vertices.size() >= kMaxWalls + 1)
                    return fail(pairs[j].line, "too many LWPOLYLINE vertices");
                vertices.push_back(vertex);
                ++j;
            }

            if (declared_vertices >= 0 &&
                static_cast<std::size_t>(declared_vertices) != vertices.size())
                return fail(pair.line, "LWPOLYLINE vertex count does not match");
            if (vertices.size() < 2)
                return fail(pair.line, "LWPOLYLINE needs at least two vertices");

            const bool closed = (flags & 1) != 0;
            const std::size_t segment_count = vertices.size() - 1 + (closed ? 1 : 0);
            if (segment_count > kMaxWalls - raw_walls_.size())
                return fail(pair.line, "too many DXF wall segments");
            for (std::size_t vertex = 0; vertex < segment_count; ++vertex)
            {
                RawWall wall;
                wall.a = vertices[vertex];
                wall.b = vertices[(vertex + 1) % vertices.size()];
                raw_walls_.push_back(wall);
            }
        }

        if (raw_walls_.size() > kMaxWalls)
            return fail(pair.line, "too many DXF wall segments");
        i = data_end - 1;
    }

    if (!found_entities_section)
        return fail(1, "ENTITIES section not found");
    if (raw_walls_.empty())
        return fail(1, "DXF contains no supported line or polyline geometry");

    for (const auto& raw : raw_walls_)
    {
        MapWall wall{};
        transformPoint(raw.a, wall.start);
        transformPoint(raw.b, wall.end);
        wall.height = raw.height;
        walls_.push_back(wall);
    }
    return true;
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
