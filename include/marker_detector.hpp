#pragma once

#include "types.hpp"

#include <cstdint>
#include <vector>

namespace sa {

// A view over an 8-bit RGB888 image. Callers own the pixel buffer; the
// detector only reads it. Any camera backend can produce this view, which
// keeps the red-marker algorithm independent of the capture API.
struct RgbFrameView
{
    const uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    int stride = 0; // bytes per row; 0 means width * 3
};

struct RedMarkerConfig
{
    float min_red = 0.55f;
    float min_red_dominance = 0.20f;
    float min_saturation = 0.40f;
    int min_cluster_pixels = 24;
    float horizontal_fov_rad = degToRad(60.0f);
};

struct MarkerDetection
{
    bool valid = false;
    float bearing_rad = 0.0f; // camera frame, positive = left
    float confidence = 0.0f;
    float area_ratio = 0.0f;
    int pixel_count = 0;
};

// Deterministic red-dominance connected-component detector. It selects the
// largest qualifying red cluster and returns its centroid bearing. It does not
// identify people, read text, or recognize faces.
class RedMarkerDetector
{
public:
    RedMarkerDetector();
    explicit RedMarkerDetector(const RedMarkerConfig& config);

    MarkerDetection detect(const RgbFrameView& frame) const;

private:
    RedMarkerConfig config_;
};

} // namespace sa
