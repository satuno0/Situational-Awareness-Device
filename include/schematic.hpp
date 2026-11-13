#pragma once

#include "occupancy_grid.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace sa {

struct SchematicMark
{
    Vec3 position;
    std::string name;
};

// Operator-supplied floor plan. The text format is deliberately small and
// debuggable; PDF, DXF and raster images are not parsed. Convert those to this
// format (or export a vector floor plan) before loading.
//
//   # comment
//   origin 0 0            schematic origin in world metres
//   yaw 0                 schematic rotation in degrees
//   scale 1.0             schematic units -> metres
//   wall  -4 -3  4 -3     wall segment x1 y1 x2 y2 [height_m]
//   door   1 -3  1.8 -3   door segment; carves free space in the grid
//   room   Kitchen -4 -3 0 2
//   label  Ladder 2.5 1.0
class Schematic
{
public:
    bool loadFile(const std::string& path, std::string& error);
    bool loadText(const std::string& text, std::string& error);
    void clear();

    bool empty() const { return walls_.empty() && rooms_.empty() && marks_.empty(); }
    const std::vector<MapWall>& walls() const { return walls_; }
    const std::vector<MapWall>& doors() const { return doors_; }
    const std::vector<MapRoom>& rooms() const { return rooms_; }
    const std::vector<SchematicMark>& marks() const { return marks_; }

    // Re-asserts the schematic structure in world coordinates. Called every
    // fusion tick because probabilistic decay is mandatory, and a one-shot
    // prior would erode back to unknown.
    void applyToGrid(OccupancyGrid& grid, float wall_log_odds = 1.10f, float door_log_odds = -0.80f) const;

private:
    struct RawPoint
    {
        float x = 0.0f;
        float y = 0.0f;
    };

    struct RawWall
    {
        RawPoint a;
        RawPoint b;
        float height = 2.70f;
        bool door = false;
    };

    struct RawRoom
    {
        std::string name;
        float min_x = 0.0f;
        float min_y = 0.0f;
        float max_x = 0.0f;
        float max_y = 0.0f;
    };

    struct RawMark
    {
        std::string name;
        RawPoint position;
    };

    void transformPoint(const RawPoint& raw, Vec3& world) const;

    float origin_x_ = 0.0f;
    float origin_y_ = 0.0f;
    float yaw_deg_ = 0.0f;
    float scale_ = 1.0f;

    std::vector<RawWall> raw_walls_;
    std::vector<RawRoom> raw_rooms_;
    std::vector<RawMark> raw_marks_;

    std::vector<MapWall> walls_;
    std::vector<MapWall> doors_;
    std::vector<MapRoom> rooms_;
    std::vector<SchematicMark> marks_;
};

} // namespace sa
