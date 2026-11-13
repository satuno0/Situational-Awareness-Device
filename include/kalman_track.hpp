#pragma once

#include "math.hpp"

#include <array>
#include <cstdint>

namespace sa {

struct Detection;

// Constant-velocity Kalman filter over [x, y, z, vx, vy, vz].
// The implementation is deliberately dependency-free and deterministic.
class KalmanTrack
{
public:
    static constexpr int kStateSize = 6;

    KalmanTrack();
    explicit KalmanTrack(const Detection& detection, uint64_t timestamp_ms);

    void predict(float dt);
    void update(const Detection& detection);

    // Coasting tracks must not translate forever on stale velocity. This is
    // applied by the track manager when a frame contains no matching return.
    void scaleVelocity(float factor);

    void reset(const Vec3& position, const Vec3& velocity, uint64_t timestamp_ms);

    Vec3 position() const;
    Vec3 velocity() const;

    // RMS 3-D position uncertainty derived from the position block of the
    // covariance. This is never hand-tuned at the track layer.
    float positionSigma() const;

    uint64_t lastUpdateMs() const { return last_update_ms_; }
    void setLastUpdateMs(uint64_t timestamp_ms) { last_update_ms_ = timestamp_ms; }

    const std::array<float, kStateSize>& state() const { return state_; }
    const std::array<float, kStateSize * kStateSize>& covariance() const { return covariance_; }
    void setStateAndCovariance(const std::array<float, kStateSize>& state,
                               const std::array<float, kStateSize * kStateSize>& covariance,
                               uint64_t last_update_ms);

private:
    void initialize(const Detection& detection, uint64_t timestamp_ms);
    void symmetrizeCovariance();

    std::array<float, kStateSize> state_{};
    std::array<float, kStateSize * kStateSize> covariance_{};
    uint64_t last_update_ms_ = 0;
};

} // namespace sa
