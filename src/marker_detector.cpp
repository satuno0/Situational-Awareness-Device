#include "marker_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace sa {
namespace {

struct Component
{
    int count = 0;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_redness = 0.0f;
};

bool isRedPixel(const uint8_t* pixel, const RedMarkerConfig& config, float& redness)
{
    const float r = static_cast<float>(pixel[0]) / 255.0f;
    const float g = static_cast<float>(pixel[1]) / 255.0f;
    const float b = static_cast<float>(pixel[2]) / 255.0f;
    const float max_gb = std::max(g, b);
    const float max_channel = std::max(r, max_gb);
    const float min_channel = std::min(r, std::min(g, b));
    if (r < config.min_red) return false;
    if (r - max_gb < config.min_red_dominance) return false;
    if (max_channel <= 1.0e-5f) return false;
    if ((max_channel - min_channel) / max_channel < config.min_saturation) return false;
    redness = clampf((r - max_gb) / std::max(r, 1.0e-3f), 0.0f, 1.0f);
    return true;
}

} // namespace

RedMarkerDetector::RedMarkerDetector() : RedMarkerDetector(RedMarkerConfig{}) {}
RedMarkerDetector::RedMarkerDetector(const RedMarkerConfig& config) : config_(config) {}

MarkerDetection RedMarkerDetector::detect(const RgbFrameView& frame) const
{
    MarkerDetection detection{};
    if (frame.data == nullptr || frame.width < 3 || frame.height < 3 || config_.min_cluster_pixels <= 0)
        return detection;

    const int stride = frame.stride > 0 ? frame.stride : frame.width * 3;
    if (stride < frame.width * 3) return detection;
    const std::size_t pixels = static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);

    std::vector<uint8_t> mask(pixels, 0);
    std::vector<float> redness(pixels, 0.0f);
    for (int y = 0; y < frame.height; ++y)
    {
        const uint8_t* row = frame.data + static_cast<std::size_t>(y) * static_cast<std::size_t>(stride);
        for (int x = 0; x < frame.width; ++x)
        {
            const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) +
                                      static_cast<std::size_t>(x);
            float value = 0.0f;
            if (isRedPixel(row + x * 3, config_, value))
            {
                mask[index] = 1;
                redness[index] = value;
            }
        }
    }

    Component best{};
    std::vector<int> queue;
    queue.reserve(static_cast<std::size_t>(frame.width * frame.height / 4 + 1));
    for (int y = 0; y < frame.height; ++y)
    {
        for (int x = 0; x < frame.width; ++x)
        {
            const std::size_t start = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) +
                                      static_cast<std::size_t>(x);
            if (mask[start] != 1) continue;

            Component component{};
            queue.clear();
            queue.push_back(static_cast<int>(start));
            mask[start] = 2;
            while (!queue.empty())
            {
                const int current = queue.back();
                queue.pop_back();
                const int cy = current / frame.width;
                const int cx = current % frame.width;
                ++component.count;
                component.sum_x += static_cast<float>(cx) + 0.5f;
                component.sum_y += static_cast<float>(cy) + 0.5f;
                component.sum_redness += redness[static_cast<std::size_t>(current)];

                const int neighbours[4][2] = {{cx - 1, cy}, {cx + 1, cy}, {cx, cy - 1}, {cx, cy + 1}};
                for (const auto& neighbour : neighbours)
                {
                    const int nx = neighbour[0];
                    const int ny = neighbour[1];
                    if (nx < 0 || ny < 0 || nx >= frame.width || ny >= frame.height) continue;
                    const std::size_t index = static_cast<std::size_t>(ny) * static_cast<std::size_t>(frame.width) +
                                              static_cast<std::size_t>(nx);
                    if (mask[index] != 1) continue;
                    mask[index] = 2;
                    queue.push_back(static_cast<int>(index));
                }
            }

            if (component.count > best.count) best = component;
        }
    }

    if (best.count < config_.min_cluster_pixels) return detection;

    const float inv_count = 1.0f / static_cast<float>(best.count);
    const float centroid_x = best.sum_x * inv_count;
    const float focal = (static_cast<float>(frame.width) * 0.5f) /
                        std::tan(std::max(0.1f, config_.horizontal_fov_rad) * 0.5f);
    detection.bearing_rad = std::atan2(-(centroid_x - static_cast<float>(frame.width) * 0.5f), focal);

    const float color_score = clampf((best.sum_redness * inv_count) / 0.60f, 0.0f, 1.0f);
    const float size_score = std::sqrt(clampf(
        static_cast<float>(best.count) / static_cast<float>(std::max(1, config_.min_cluster_pixels) * 6), 0.0f, 1.0f));
    detection.confidence = clampf(0.42f + 0.38f * color_score + 0.20f * size_score, 0.0f, 1.0f);
    detection.area_ratio = static_cast<float>(best.count) / static_cast<float>(pixels);
    detection.pixel_count = best.count;
    detection.valid = true;
    return detection;
}

} // namespace sa
