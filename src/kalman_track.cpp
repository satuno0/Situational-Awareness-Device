#include "kalman_track.hpp"

#include "types.hpp"

#include <algorithm>
#include <cmath>

namespace sa {
namespace {

using Mat6 = std::array<float, 36>;

Mat6 multiply(const Mat6& a, const Mat6& b)
{
    Mat6 out{};
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) sum += a[r * 6 + k] * b[k * 6 + c];
            out[r * 6 + c] = sum;
        }
    }
    return out;
}

Mat6 transpose(const Mat6& a)
{
    Mat6 out{};
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) out[c * 6 + r] = a[r * 6 + c];
    return out;
}

bool invert3(const float m[9], float inv[9])
{
    const float a = m[0], b = m[1], c = m[2];
    const float d = m[3], e = m[4], f = m[5];
    const float g = m[6], h = m[7], i = m[8];
    const float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (std::abs(det) < 1.0e-8f) return false;
    const float id = 1.0f / det;
    inv[0] = (e * i - f * h) * id;
    inv[1] = (c * h - b * i) * id;
    inv[2] = (b * f - c * e) * id;
    inv[3] = (f * g - d * i) * id;
    inv[4] = (a * i - c * g) * id;
    inv[5] = (c * d - a * f) * id;
    inv[6] = (d * h - e * g) * id;
    inv[7] = (b * g - a * h) * id;
    inv[8] = (a * e - b * d) * id;
    return true;
}

} // namespace

KalmanTrack::KalmanTrack()
{
    state_.fill(0.0f);
    covariance_.fill(0.0f);
    for (int i = 0; i < 6; ++i) covariance_[i * 6 + i] = (i < 3) ? 0.25f : 4.0f;
}

KalmanTrack::KalmanTrack(const Detection& detection, uint64_t timestamp_ms)
{
    initialize(detection, timestamp_ms);
}

void KalmanTrack::initialize(const Detection& detection, uint64_t timestamp_ms)
{
    state_.fill(0.0f);
    covariance_.fill(0.0f);
    state_[0] = detection.position.x;
    state_[1] = detection.position.y;
    state_[2] = detection.position.z;
    state_[3] = detection.velocity.x;
    state_[4] = detection.velocity.y;
    state_[5] = detection.velocity.z;

    // A new return is uncertain. The first few measurement updates tighten
    // this quickly; occlusion grows it again.
    covariance_[0 * 6 + 0] = 0.30f;
    covariance_[1 * 6 + 1] = 0.30f;
    covariance_[2 * 6 + 2] = 0.60f;
    covariance_[3 * 6 + 3] = 6.0f;
    covariance_[4 * 6 + 4] = 6.0f;
    covariance_[5 * 6 + 5] = 4.0f;
    last_update_ms_ = timestamp_ms;
}

void KalmanTrack::reset(const Vec3& position, const Vec3& velocity, uint64_t timestamp_ms)
{
    state_.fill(0.0f);
    covariance_.fill(0.0f);
    state_[0] = position.x;
    state_[1] = position.y;
    state_[2] = position.z;
    state_[3] = velocity.x;
    state_[4] = velocity.y;
    state_[5] = velocity.z;
    covariance_[0 * 6 + 0] = 0.25f;
    covariance_[1 * 6 + 1] = 0.25f;
    covariance_[2 * 6 + 2] = 0.50f;
    covariance_[3 * 6 + 3] = 4.0f;
    covariance_[4 * 6 + 4] = 4.0f;
    covariance_[5 * 6 + 5] = 2.0f;
    last_update_ms_ = timestamp_ms;
}

void KalmanTrack::predict(float dt)
{
    if (!(dt > 0.0f)) return;
    dt = std::min(dt, 2.0f);

    Mat6 f{};
    for (int i = 0; i < 6; ++i) f[i * 6 + i] = 1.0f;
    f[0 * 6 + 3] = dt;
    f[1 * 6 + 4] = dt;
    f[2 * 6 + 5] = dt;

    Mat6 p{};
    std::copy(covariance_.begin(), covariance_.end(), p.begin());
    Mat6 next = multiply(multiply(f, p), transpose(f));

    const float q_xy = 1.2f;
    const float q_z = 0.35f;
    const float axes[3] = {q_xy, q_xy, q_z};
    for (int axis = 0; axis < 3; ++axis)
    {
        const int pos = axis;
        const int vel = axis + 3;
        const float q = axes[axis];
        next[pos * 6 + pos] += q * dt * dt * dt / 3.0f;
        next[pos * 6 + vel] += q * dt * dt / 2.0f;
        next[vel * 6 + pos] += q * dt * dt / 2.0f;
        next[vel * 6 + vel] += q * dt;
    }

    std::copy(next.begin(), next.end(), covariance_.begin());
    symmetrizeCovariance();

    for (int i = 0; i < 6; ++i)
    {
        if (covariance_[i * 6 + i] < 1.0e-5f) covariance_[i * 6 + i] = 1.0e-5f;
    }
}

void KalmanTrack::update(const Detection& detection)
{
    const float range = std::max(0.2f, detection.range);
    const float snr = std::max(1.0f, detection.snr);
    const float snr_scale = 1.0f / std::sqrt(std::min(snr, 30.0f));

    // Radar range accuracy and angular spreading both worsen with range.
    // These values describe the measurement, not track health.
    const float xy_var = (0.008f + 0.00035f * range * range) * snr_scale;
    const float z_var = (0.025f + 0.0010f * range * range) * snr_scale;
    float r[9] = {
        std::max(0.002f, xy_var), 0.0f, 0.0f,
        0.0f, std::max(0.002f, xy_var), 0.0f,
        0.0f, 0.0f, std::max(0.005f, z_var)};

    float ph[18]{};
    for (int row = 0; row < 6; ++row)
    {
        ph[row * 3 + 0] = covariance_[row * 6 + 0];
        ph[row * 3 + 1] = covariance_[row * 6 + 1];
        ph[row * 3 + 2] = covariance_[row * 6 + 2];
    }

    float s[9]{};
    for (int rr = 0; rr < 3; ++rr)
    {
        for (int cc = 0; cc < 3; ++cc)
        {
            float sum = r[rr * 3 + cc];
            for (int k = 0; k < 6; ++k) sum += ph[k * 3 + rr] * covariance_[k * 6 + cc];
            s[rr * 3 + cc] = sum;
        }
    }

    float s_inv[9]{};
    if (!invert3(s, s_inv))
    {
        for (int i = 0; i < 3; ++i) s_inv[i * 3 + i] = 1.0f / std::max(0.01f, s[i * 3 + i]);
    }

    float gain[18]{};
    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) sum += ph[row * 3 + k] * s_inv[k * 3 + col];
            gain[row * 3 + col] = sum;
        }
    }

    const float innovation[3] = {
        detection.position.x - state_[0],
        detection.position.y - state_[1],
        detection.position.z - state_[2]};
    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 3; ++col) state_[row] += gain[row * 3 + col] * innovation[col];
    }

    // Joseph form: (I-KH)P(I-KH)^T + K R K^T.
    Mat6 ikh{};
    for (int i = 0; i < 6; ++i) ikh[i * 6 + i] = 1.0f;
    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 3; ++col) ikh[row * 6 + col] -= gain[row * 3 + col];

    Mat6 p{};
    std::copy(covariance_.begin(), covariance_.end(), p.begin());
    Mat6 next = multiply(multiply(ikh, p), transpose(ikh));

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            float krk = 0.0f;
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    krk += gain[row * 3 + a] * r[a * 3 + b] * gain[col * 3 + b];
            next[row * 6 + col] += krk;
        }
    }

    std::copy(next.begin(), next.end(), covariance_.begin());
    symmetrizeCovariance();
    for (int i = 0; i < 6; ++i)
    {
        if (covariance_[i * 6 + i] < 1.0e-5f) covariance_[i * 6 + i] = 1.0e-5f;
    }
}

void KalmanTrack::scaleVelocity(float factor)
{
    const float f = clampf(factor, 0.0f, 1.0f);
    state_[3] *= f;
    state_[4] *= f;
    state_[5] *= f;
}

void KalmanTrack::setStateAndCovariance(const std::array<float, 6>& state,
                                        const std::array<float, 36>& covariance,
                                        uint64_t last_update_ms)
{
    state_ = state;
    covariance_ = covariance;
    last_update_ms_ = last_update_ms;
    symmetrizeCovariance();
}

Vec3 KalmanTrack::position() const { return {state_[0], state_[1], state_[2]}; }
Vec3 KalmanTrack::velocity() const { return {state_[3], state_[4], state_[5]}; }

float KalmanTrack::positionSigma() const
{
    // Largest eigenvalue of the symmetric 3x3 position covariance. Power
    // iteration is deterministic and more honest than a hand-picked sigma.
    const float a[9] = {
        covariance_[0], covariance_[1], covariance_[2],
        covariance_[6], covariance_[7], covariance_[8],
        covariance_[12], covariance_[13], covariance_[14]};
    float v[3] = {0.57735026f, 0.57735026f, 0.57735026f};
    float lambda = 0.0f;
    for (int iter = 0; iter < 12; ++iter)
    {
        float nv[3];
        for (int r = 0; r < 3; ++r) nv[r] = a[r * 3] * v[0] + a[r * 3 + 1] * v[1] + a[r * 3 + 2] * v[2];
        const float n = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
        if (n < 1.0e-9f) break;
        v[0] = nv[0] / n;
        v[1] = nv[1] / n;
        v[2] = nv[2] / n;
        lambda = a[0] * v[0] * v[0] + a[4] * v[1] * v[1] + a[8] * v[2] * v[2] +
                 2.0f * (a[1] * v[0] * v[1] + a[2] * v[0] * v[2] + a[5] * v[1] * v[2]);
    }
    return std::sqrt(std::max(0.0f, lambda));
}

void KalmanTrack::symmetrizeCovariance()
{
    for (int r = 0; r < 6; ++r)
    {
        for (int c = r + 1; c < 6; ++c)
        {
            const float v = 0.5f * (covariance_[r * 6 + c] + covariance_[c * 6 + r]);
            covariance_[r * 6 + c] = v;
            covariance_[c * 6 + r] = v;
        }
    }
}

} // namespace sa
