#include "schematic.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool near(float a, float b, float epsilon)
{
    return std::fabs(a - b) <= epsilon;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: sa_dxf_tests <file.dxf>\n";
        return 2;
    }

    sa::Schematic schematic;
    std::string error;
    if (!schematic.loadFile(argv[1], error))
    {
        std::cerr << "DXF parse failed: " << error << "\n";
        return 1;
    }

    if (schematic.walls().size() != 4)
    {
        std::cerr << "expected four walls, got " << schematic.walls().size() << "\n";
        return 1;
    }

    const sa::MapWall& first = schematic.walls().front();
    if (!near(first.start.x, 0.03f, 1.0e-5f) ||
        !near(first.start.y, 0.02f, 1.0e-5f) ||
        !near(first.end.x, -0.03f, 1.0e-5f) ||
        !near(first.end.y, 0.02f, 1.0e-5f))
    {
        std::cerr << "DXF centimeter units or first polyline segment parsed incorrectly\n";
        return 1;
    }

    return 0;
}
