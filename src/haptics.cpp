#include "haptics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

#if defined(__linux__)
#include <cerrno>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace sa {
namespace {

struct Step
{
    bool on;
    int duration_ms;
};

std::vector<Step> timeline(HapticPattern pattern)
{
    switch (pattern)
    {
    case HapticPattern::SAFE_TAGGED:
        return {{true, 80}, {false, 80}, {true, 180}};
    case HapticPattern::UNKNOWN_PERSON:
        return {{true, 180}};
    case HapticPattern::LOST_TRACK:
        return {{true, 60}, {false, 70}, {true, 60}, {false, 70}, {true, 60}};
    case HapticPattern::PRESENCE_BARRIER:
        return {{true, 250}};
    case HapticPattern::COLLISION:
    case HapticPattern::NONE:
        break;
    }
    return {};
}

int timelineDuration(const std::vector<Step>& steps)
{
    int total = 0;
    for (const auto& step : steps) total += step.duration_ms;
    return total;
}

} // namespace

const char* hapticPatternName(HapticPattern pattern)
{
    switch (pattern)
    {
    case HapticPattern::NONE: return "NONE";
    case HapticPattern::COLLISION: return "COLLISION";
    case HapticPattern::SAFE_TAGGED: return "SAFE_TAGGED";
    case HapticPattern::UNKNOWN_PERSON: return "UNKNOWN_PERSON";
    case HapticPattern::LOST_TRACK: return "LOST_TRACK";
    case HapticPattern::PRESENCE_BARRIER: return "PRESENCE_BARRIER";
    }
    return "NONE";
}

std::optional<float> closeRangeCollisionRange(const WorldModel& world,
                                              const UltrasonicMeasurement* ultrasonic)
{
    float closest = std::numeric_limits<float>::infinity();
    if (ultrasonic && ultrasonic->valid)
    {
        closest = ultrasonic->range_m;
    }
    else
    {
        for (const auto& track : world.tracks)
        {
            const Vec3 body = poseInverseApply(world.pose, track.filter.position());
            if (body.y <= 0.15f) continue;
            const float distance = std::sqrt(body.x * body.x + body.y * body.y);
            if (distance > 1.5f) continue;
            const float angle = std::atan2(std::abs(body.x), std::max(0.01f, body.y));
            if (angle <= degToRad(30.0f)) closest = std::min(closest, distance);
        }
        for (const auto& object : world.static_objects)
        {
            const Vec3 body = poseInverseApply(world.pose, object.position);
            if (body.y <= 0.15f) continue;
            const float distance = std::sqrt(body.x * body.x + body.y * body.y);
            if (distance > 1.5f) continue;
            const float angle = std::atan2(std::abs(body.x), std::max(0.01f, body.y));
            if (angle <= degToRad(30.0f)) closest = std::min(closest, distance);
        }
    }
    if (std::isfinite(closest) && closest <= 1.5f) return closest;
    return std::nullopt;
}

Haptics::Haptics() : Haptics(HapticConfig{}) {}
Haptics::Haptics(const HapticConfig& config) : config_(config) {}
Haptics::~Haptics() { stop(); }

bool Haptics::start(std::string& error)
{
    stop();
    if (!config_.enabled) return true;
    if (!config_.device_path.empty())
    {
        device_.open(config_.device_path, std::ios::out | std::ios::binary);
        if (!device_)
        {
            error = "cannot open haptic device " + config_.device_path;
            return false;
        }
    }
#if defined(__linux__)
    if (!config_.drv2605_i2c_device.empty())
    {
        drv2605_fd_ = ::open(config_.drv2605_i2c_device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (drv2605_fd_ >= 0 &&
            ioctl(drv2605_fd_, I2C_SLAVE, static_cast<unsigned long>(config_.drv2605_i2c_address)) == 0)
        {
            std::string output_error;
            drv2605_active_ = writeDrv2605(0x1A, 0xB6, output_error) &&
                              writeDrv2605(0x01, 0x05, output_error) &&
                              writeDrv2605(0x02, 0x00, output_error);
            if (!drv2605_active_)
                std::cerr << "[haptics] DRV2605L setup failed: " << output_error << "\n";
        }
        else
        {
            const std::string output_error = std::strerror(errno);
            std::cerr << "[haptics] DRV2605L unavailable: " << output_error << "\n";
        }
        if (!drv2605_active_)
        {
            if (drv2605_fd_ >= 0) ::close(drv2605_fd_);
            drv2605_fd_ = -1;
            if (!device_.is_open())
            {
                error = "DRV2605L setup failed and no PWM fallback is available";
                return false;
            }
            std::cerr << "[haptics] falling back to the configured PWM output\n";
        }
    }
#else
    if (!config_.drv2605_i2c_device.empty())
    {
        if (!device_.is_open())
        {
            error = "DRV2605L I2C hardware is supported only on Linux and no PWM fallback is available";
            return false;
        }
        std::cerr << "[haptics] DRV2605L unavailable; falling back to the configured PWM output\n";
    }
#endif
    stop_ = false;
    running_ = true;
    resetTracking();
    worker_ = std::thread(&Haptics::workerLoop, this);
    return true;
}

void Haptics::stop()
{
    if (running_.load() || worker_.joinable())
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            running_ = false;
        }
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
    if (device_.is_open()) device_.close();
#if defined(__linux__)
    if (drv2605_fd_ >= 0) ::close(drv2605_fd_);
#endif
    drv2605_fd_ = -1;
    drv2605_active_ = false;
}

void Haptics::resetTracking()
{
    std::lock_guard<std::mutex> lock(mutex_);
    previous_visible_.clear();
    previous_tagged_.clear();
    previous_presence_ = false;
    collision_range_.reset();
    presence_active_ = false;
    event_queue_.clear();
    active_pattern_ = HapticPattern::NONE;
}

bool Haptics::writeDrv2605(uint8_t reg, uint8_t value, std::string& error)
{
#if defined(__linux__)
    if (drv2605_fd_ < 0)
    {
        error = "DRV2605L I2C device is not open";
        return false;
    }
    const uint8_t data[2] = {reg, value};
    const ssize_t written = ::write(drv2605_fd_, data, sizeof(data));
    if (written != static_cast<ssize_t>(sizeof(data)))
    {
        error = "DRV2605L register write failed: " + std::string(std::strerror(errno));
        return false;
    }
    return true;
#else
    (void)reg;
    (void)value;
    error = "DRV2605L I2C hardware is supported only on Linux";
    return false;
#endif
}

bool Haptics::setMotor(bool on)
{
    if (drv2605_active_)
    {
        std::string error;
        if (writeDrv2605(0x02, on ? 96 : 0, error)) return true;
        std::cerr << "[haptics] DRV2605L output failed: " << error << "\n";
        drv2605_active_ = false;
#if defined(__linux__)
        if (drv2605_fd_ >= 0) ::close(drv2605_fd_);
#endif
        drv2605_fd_ = -1;
    }
    if (device_.is_open())
    {
        device_.seekp(0);
        device_ << (on ? config_.active_value : config_.inactive_value);
        device_.flush();
        return static_cast<bool>(device_);
    }
    return config_.device_path.empty();
}

void Haptics::trigger(HapticPattern pattern)
{
    if (pattern == HapticPattern::NONE) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (event_queue_.size() >= 8) event_queue_.pop_front();
        event_queue_.push_back(pattern);
    }
    cv_.notify_one();
}

HapticPattern Haptics::activePattern() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return active_pattern_;
}

void Haptics::workerLoop()
{
    using Clock = std::chrono::steady_clock;
    auto last_tick = Clock::now();
    auto collision_start = last_tick;
    auto event_start = last_tick;
    auto presence_start = last_tick;
    bool collision_was_active = false;
    bool event_active = false;
    bool presence_was_active = false;
    HapticPattern event_pattern = HapticPattern::NONE;
    std::vector<Step> event_steps;
    int event_duration = 0;
    bool motor_on = false;

    while (true)
    {
        bool collision = false;
        float collision_range = 10.0f;
        bool presence = false;
        HapticPattern next_event = HapticPattern::NONE;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) break;
            collision = collision_range_.has_value();
            if (collision) collision_range = *collision_range_;
            presence = presence_active_;
            if (!event_active && !event_queue_.empty())
            {
                next_event = event_queue_.front();
                event_queue_.pop_front();
            }
        }

        const auto now = Clock::now();
        const float dt = std::chrono::duration<float>(now - last_tick).count();
        (void)dt;
        last_tick = now;

        if (next_event != HapticPattern::NONE)
        {
            event_pattern = next_event;
            event_steps = timeline(next_event);
            event_duration = timelineDuration(event_steps);
            event_start = now;
            event_active = event_duration > 0;
        }

        bool desired_on = false;
        HapticPattern active = HapticPattern::NONE;

        if (collision)
        {
            if (!collision_was_active)
            {
                collision_start = now;
                collision_was_active = true;
            }
            active = HapticPattern::COLLISION;
            if (collision_range <= 0.25f)
            {
                desired_on = true;
            }
            else
            {
                const float clamped = clampf(collision_range, 0.25f, 1.5f);
                const float t = (clamped - 0.25f) / 1.25f;
                const float period_s = lerpf(0.16f, 1.0f, t);
                const float duty = lerpf(0.55f, 0.28f, t);
                const float elapsed = std::chrono::duration<float>(now - collision_start).count();
                const float phase = std::fmod(elapsed, period_s);
                desired_on = phase < period_s * duty;
            }
        }
        else
        {
            collision_was_active = false;
        }

        if (!collision)
        {
            if (event_active)
            {
                const float elapsed_ms = std::chrono::duration<float, std::milli>(now - event_start).count();
                if (elapsed_ms >= static_cast<float>(event_duration))
                {
                    event_active = false;
                    event_pattern = HapticPattern::NONE;
                    event_steps.clear();
                }
                else
                {
                    active = event_pattern;
                    int cursor = 0;
                    for (const auto& step : event_steps)
                    {
                        cursor += step.duration_ms;
                        if (elapsed_ms < static_cast<float>(cursor))
                        {
                            desired_on = step.on;
                            break;
                        }
                    }
                }
            }
            else if (presence)
            {
                if (!presence_was_active)
                {
                    presence_start = now;
                    presence_was_active = true;
                }
                active = HapticPattern::PRESENCE_BARRIER;
                const float elapsed = std::chrono::duration<float>(now - presence_start).count();
                desired_on = std::fmod(elapsed, 1.25f) < 0.25f;
            }
            else
            {
                presence_was_active = false;
            }
        }

        if (desired_on != motor_on)
        {
            if (!setMotor(desired_on))
                std::cerr << "[haptics] PWM output write failed\n";
            motor_on = desired_on;
            if (config_.verbose && desired_on)
                std::clog << "[haptics] " << hapticPatternName(active) << " pulse\n";
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (active_pattern_ != active) active_pattern_ = active;
        }

        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(10), [this] { return stop_; });
        if (stop_) break;
    }

    if (!setMotor(false)) std::cerr << "[haptics] failed to stop haptic output\n";
}

void Haptics::update(const WorldModel& world, const UltrasonicMeasurement* ultrasonic)
{
    if (!running_.load()) return;

    const std::optional<float> collision_range = closeRangeCollisionRange(world, ultrasonic);

    std::vector<HapticPattern> events;
    std::unordered_map<uint32_t, bool> current_visible;
    std::unordered_map<uint32_t, bool> current_tagged;
    bool presence = false;

    for (const auto& track : world.tracks)
    {
        current_visible[track.id] = track.visible;
        current_tagged[track.id] = track.identity.tagged_safe;
        if (!track.visible && track.state == TrackState::PRESENCE_SUPPORTED) presence = true;

        const bool was_visible = previous_visible_.count(track.id) && previous_visible_[track.id];
        if (track.visible && !was_visible)
            events.push_back(track.identity.tagged_safe ? HapticPattern::SAFE_TAGGED : HapticPattern::UNKNOWN_PERSON);
        else if (!track.visible && was_visible)
            events.push_back(HapticPattern::LOST_TRACK);

        // A tag that becomes known while the track was already visible gets
        // its own confirmation cue. The newly-visible case above would
        // otherwise emit SAFE_TAGGED twice for the same transition.
        const bool was_tagged = previous_tagged_.count(track.id) && previous_tagged_[track.id];
        if (track.visible && was_visible && track.identity.tagged_safe && !was_tagged)
            events.push_back(HapticPattern::SAFE_TAGGED);
    }
    for (const auto& previous : previous_visible_)
    {
        if (previous.second && current_visible.find(previous.first) == current_visible.end())
            events.push_back(HapticPattern::LOST_TRACK);
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (collision_range)
            collision_range_ = *collision_range;
        else
            collision_range_.reset();
        presence_active_ = presence;
        for (HapticPattern pattern : events)
        {
            if (event_queue_.size() >= 8) event_queue_.pop_front();
            event_queue_.push_back(pattern);
        }
    }
    previous_visible_ = std::move(current_visible);
    previous_tagged_ = std::move(current_tagged);
    previous_presence_ = presence;
    cv_.notify_all();
}

} // namespace sa
