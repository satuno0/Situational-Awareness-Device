#pragma once

#include "types.hpp"

#include <cstdint>

namespace sa {

struct SimulationConfig
{
    uint32_t seed = 0x5A17u;
    double duration_s = 60.0;
    double frame_period_s = 0.05;
    uint64_t start_timestamp_ms = 1000;

    // Emit a synthetic red marker so the wrist-guidance path can be exercised
    // without a camera. This is a sensor measurement only; it never creates a
    // track or affects person classification.
    bool marker_enabled = true;
};

// Deterministic synthetic handheld session used for development, replay tests
// and the phone visualization. It injects measurements only; it never writes
// directly into tracks or the world model.
class Simulation
{
public:
    Simulation();
    explicit Simulation(const SimulationConfig& config);

    void reset();
    bool next(SensorFrame& frame);
    bool finished() const;
    Pose2D truthPose() const;
    double elapsedSeconds() const { return elapsed_s_; }

private:
    struct Person
    {
        Vec3 position;
        Vec3 velocity;
        bool safe_tagged = false;
        uint16_t tag_id = 0;
    };

    struct Box
    {
        float min_x;
        float min_y;
        float max_x;
        float max_y;
    };

    Pose2D poseAt(double t) const;
    Vec3 personPosition(int index, double t) const;
    Vec3 personVelocity(int index, double t) const;
    Vec3 markerPositionAt(double t) const;
    Person personAt(int index, double t) const;
    float randomUniform();
    float gaussian(float sigma);
    bool lineOfSight(const Vec3& origin, const Vec3& target, double& blocked_distance) const;
    void addStaticReturns(RadarFrame& frame, const Pose2D& pose, double t);
    void addPersonReturns(RadarFrame& frame, const Pose2D& pose, double t);
    void addDetection(RadarFrame& frame, const Pose2D& pose, const Vec3& world_point, const Vec3& world_velocity,
                      float snr);
    float ultrasonicForward(const Pose2D& pose, double t) const;

    SimulationConfig config_;
    uint32_t rng_state_;
    uint64_t tick_ = 0;
    double elapsed_s_ = 0.0;
    bool finished_ = false;
};

} // namespace sa
