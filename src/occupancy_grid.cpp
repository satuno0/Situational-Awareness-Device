#include "occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace sa {
namespace {

constexpr float kMinProbability = 0.02f;
constexpr float kMaxProbability = 0.98f;

float logOdds(float probability)
{
    const float p = clampf(probability, 0.001f, 0.999f);
    return std::log(p / (1.0f - p));
}

float probabilityFromLogOdds(float log_odds)
{
    return 1.0f - 1.0f / (1.0f + std::exp(log_odds));
}

} // namespace

OccupancyGrid::OccupancyGrid() : OccupancyGrid(GridConfig{}) {}

OccupancyGrid::OccupancyGrid(const GridConfig& config) : config_(config)
{
    if (config_.width <= 0) config_.width = 160;
    if (config_.height <= 0) config_.height = 160;
    if (!(config_.cell_size > 0.0f)) config_.cell_size = 0.25f;
    reset();
}

void OccupancyGrid::reset()
{
    cells_.assign(static_cast<std::size_t>(config_.width * config_.height), OccupancyCell{0.5f});
}

int OccupancyGrid::cellIndex(int cx, int cy) const
{
    if (cx < 0 || cy < 0 || cx >= config_.width || cy >= config_.height) return -1;
    return cy * config_.width + cx;
}

bool OccupancyGrid::worldToCell(float x, float y, int& cx, int& cy) const
{
    cx = static_cast<int>(std::floor(x / config_.cell_size)) + config_.width / 2;
    cy = static_cast<int>(std::floor(y / config_.cell_size)) + config_.height / 2;
    return cx >= 0 && cy >= 0 && cx < config_.width && cy < config_.height;
}

Vec3 OccupancyGrid::cellCenter(int cx, int cy) const
{
    return {
        (static_cast<float>(cx - config_.width / 2) + 0.5f) * config_.cell_size,
        (static_cast<float>(cy - config_.height / 2) + 0.5f) * config_.cell_size,
        0.0f};
}

float OccupancyGrid::probability(int cx, int cy) const
{
    const int idx = cellIndex(cx, cy);
    return idx < 0 ? 0.5f : cells_[static_cast<std::size_t>(idx)].occupancy_prob;
}

void OccupancyGrid::setProbability(int cx, int cy, float probability)
{
    const int idx = cellIndex(cx, cy);
    if (idx >= 0) cells_[static_cast<std::size_t>(idx)].occupancy_prob = clampf(probability, 0.0f, 1.0f);
}

void OccupancyGrid::markOccupiedWorld(const Vec3& position, float log_odds)
{
    int cx = 0;
    int cy = 0;
    if (!worldToCell(position.x, position.y, cx, cy)) return;
    auto& cell = cells_[static_cast<std::size_t>(cellIndex(cx, cy))];
    cell.occupancy_prob = clampf(probabilityFromLogOdds(logOdds(cell.occupancy_prob) + log_odds),
                                 kMinProbability, kMaxProbability);
}

void OccupancyGrid::markFreeWorld(const Vec3& position, float log_odds)
{
    int cx = 0;
    int cy = 0;
    if (!worldToCell(position.x, position.y, cx, cy)) return;
    auto& cell = cells_[static_cast<std::size_t>(cellIndex(cx, cy))];
    cell.occupancy_prob = clampf(probabilityFromLogOdds(logOdds(cell.occupancy_prob) + log_odds),
                                 kMinProbability, kMaxProbability);
}

bool OccupancyGrid::integrateRay(const Vec3& sensor_origin,
                                 const Vec3& endpoint,
                                 float occupied_log_odds,
                                 float free_log_odds,
                                 float max_range_m)
{
    const Vec3 delta{endpoint.x - sensor_origin.x, endpoint.y - sensor_origin.y, 0.0f};
    const float distance = norm(delta);
    if (!(distance > 1.0e-4f) || distance > max_range_m) return false;

    int end_cx = 0;
    int end_cy = 0;
    if (!worldToCell(endpoint.x, endpoint.y, end_cx, end_cy)) return false;

    const float step = std::max(0.04f, config_.cell_size * 0.4f);
    const int steps = std::max(1, static_cast<int>(distance / step));
    int last_idx = -1;
    for (int s = 0; s <= steps; ++s)
    {
        const float t = static_cast<float>(s) / static_cast<float>(steps);
        const Vec3 p = sensor_origin + delta * t;
        int cx = 0;
        int cy = 0;
        if (!worldToCell(p.x, p.y, cx, cy)) continue;
        const int idx = cellIndex(cx, cy);
        if (idx < 0 || idx == last_idx) continue;
        last_idx = idx;
        // The endpoint is applied exactly once after the traversal, so skip it
        // here instead of stamping occupied evidence twice.
        if (cx == end_cx && cy == end_cy) continue;
        auto& cell = cells_[static_cast<std::size_t>(idx)];
        cell.occupancy_prob = clampf(probabilityFromLogOdds(logOdds(cell.occupancy_prob) + free_log_odds),
                                     kMinProbability, kMaxProbability);
    }

    int cx = 0;
    int cy = 0;
    if (worldToCell(endpoint.x, endpoint.y, cx, cy))
    {
        const int idx = cellIndex(cx, cy);
        if (idx >= 0)
        {
            auto& cell = cells_[static_cast<std::size_t>(idx)];
            cell.occupancy_prob = clampf(probabilityFromLogOdds(logOdds(cell.occupancy_prob) + occupied_log_odds),
                                         kMinProbability, kMaxProbability);
        }
    }
    return true;
}

void OccupancyGrid::decay(float factor)
{
    const float f = clampf(factor, 0.0f, 1.0f);
    for (auto& cell : cells_) cell.occupancy_prob = 0.5f + (cell.occupancy_prob - 0.5f) * f;
}

std::vector<MapObject> OccupancyGrid::extractMapObjects(float occupied_threshold,
                                                        int min_cells,
                                                        std::size_t max_objects) const
{
    struct Cluster
    {
        int cells = 0;
        double sx = 0.0;
        double sy = 0.0;
        double sp = 0.0;
    };

    std::vector<MapObject> objects;
    if (cells_.empty()) return objects;

    std::vector<uint8_t> visited(cells_.size(), 0);
    std::vector<int> queue;
    queue.reserve(256);

    for (int cy = 0; cy < config_.height; ++cy)
    {
        for (int cx = 0; cx < config_.width; ++cx)
        {
            const int start = cellIndex(cx, cy);
            if (start < 0 || visited[static_cast<std::size_t>(start)]) continue;
            if (cells_[static_cast<std::size_t>(start)].occupancy_prob < occupied_threshold) continue;

            Cluster cluster;
            queue.clear();
            queue.push_back(start);
            visited[static_cast<std::size_t>(start)] = 1;

            for (std::size_t q = 0; q < queue.size(); ++q)
            {
                const int idx = queue[q];
                const int x = idx % config_.width;
                const int y = idx / config_.width;
                const auto& cell = cells_[static_cast<std::size_t>(idx)];
                cluster.cells += 1;
                const Vec3 center = cellCenter(x, y);
                cluster.sx += center.x;
                cluster.sy += center.y;
                cluster.sp += cell.occupancy_prob;

                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = x + dx;
                        const int ny = y + dy;
                        const int nidx = cellIndex(nx, ny);
                        if (nidx < 0 || visited[static_cast<std::size_t>(nidx)]) continue;
                        if (cells_[static_cast<std::size_t>(nidx)].occupancy_prob < occupied_threshold) continue;
                        visited[static_cast<std::size_t>(nidx)] = 1;
                        queue.push_back(nidx);
                    }
                }
            }

            if (cluster.cells >= min_cells)
            {
                MapObject object;
                object.position = {
                    static_cast<float>(cluster.sx / cluster.cells),
                    static_cast<float>(cluster.sy / cluster.cells),
                    0.0f};
                object.confidence = static_cast<float>(cluster.sp / cluster.cells);
                objects.push_back(object);
            }
        }
    }

    std::sort(objects.begin(), objects.end(), [](const MapObject& a, const MapObject& b) {
        if (a.confidence != b.confidence) return a.confidence > b.confidence;
        if (a.position.y != b.position.y) return a.position.y < b.position.y;
        return a.position.x < b.position.x;
    });
    if (objects.size() > max_objects) objects.resize(max_objects);
    return objects;
}

int OccupancyGrid::occupiedCellCount(float threshold) const
{
    int count = 0;
    for (const auto& cell : cells_)
        if (cell.occupancy_prob >= threshold) ++count;
    return count;
}

int OccupancyGrid::knownCellCount() const
{
    int count = 0;
    for (const auto& cell : cells_)
        if (std::abs(cell.occupancy_prob - 0.5f) > 0.05f) ++count;
    return count;
}

std::vector<uint16_t> OccupancyGrid::encodedProbabilities() const
{
    std::vector<uint16_t> out;
    out.reserve(cells_.size());
    for (const auto& cell : cells_)
    {
        const float p = clampf(cell.occupancy_prob, 0.0f, 1.0f);
        out.push_back(static_cast<uint16_t>(std::lround(p * 65535.0f)));
    }
    return out;
}

bool OccupancyGrid::loadEncodedProbabilities(const GridConfig& config,
                                             const std::vector<uint16_t>& values)
{
    if (config.width <= 0 || config.height <= 0) return false;
    if (values.size() != static_cast<std::size_t>(config.width * config.height)) return false;
    config_ = config;
    cells_.resize(values.size());
    for (std::size_t i = 0; i < values.size(); ++i)
        cells_[i].occupancy_prob = static_cast<float>(values[i]) / 65535.0f;
    return true;
}

} // namespace sa
