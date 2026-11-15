#pragma once

#include "math.hpp"

#include <cstdint>
#include <vector>

namespace sa {

struct OccupancyCell
{
    float occupancy_prob;
};

struct MapObject
{
    Vec3 position;

    float confidence;
};

struct GridConfig
{
    float cell_size = 0.25f;
    int width = 280;
    int height = 280;
    float range_m = 35.0f;
};

// Bayesian 2-D occupancy grid in a right-handed world frame: +X is right and
// +Y is forward. The grid is centered on the origin and spans [-range_m,
// +range_m] on both axes when the default 280 x 280 / 0.25 m configuration is
// used.
class OccupancyGrid
{
public:
    OccupancyGrid();
    explicit OccupancyGrid(const GridConfig& config);

    void reset();
    const GridConfig& config() const { return config_; }

    bool worldToCell(float x, float y, int& cx, int& cy) const;
    Vec3 cellCenter(int cx, int cy) const;
    int cellIndex(int cx, int cy) const;

    float probability(int cx, int cy) const;
    void setProbability(int cx, int cy, float probability);

    bool integrateRay(const Vec3& sensor_origin,
                      const Vec3& endpoint,
                      float occupied_log_odds = 0.85f,
                      float free_log_odds = -0.35f,
                      float max_range_m = 35.0f);

    void markOccupiedWorld(const Vec3& position, float log_odds = 0.85f);
    void markFreeWorld(const Vec3& position, float log_odds = -0.35f);

    // Mandatory stale-map suppression. p <- 0.5 + (p - 0.5) * factor.
    void decay(float factor = 0.999f);

    std::vector<MapObject> extractMapObjects(float occupied_threshold = 0.65f,
                                             int min_cells = 3,
                                             std::size_t max_objects = 64) const;

    int occupiedCellCount(float threshold = 0.65f) const;
    int knownCellCount() const;
    int totalCellCount() const { return config_.width * config_.height; }

    std::vector<uint16_t> encodedProbabilities() const;
    bool loadEncodedProbabilities(const GridConfig& config, const std::vector<uint16_t>& values);
    const std::vector<OccupancyCell>& cells() const { return cells_; }

private:
    GridConfig config_;
    std::vector<OccupancyCell> cells_;
};

} // namespace sa
