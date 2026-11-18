#include "wrist_haptics.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace sa {
namespace {

std::string trim(std::string value)
{
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

bool parseFloat(const std::string& text, float& value)
{
    if (text.empty()) return false;
    char* end = nullptr;
    value = std::strtof(text.c_str(), &end);
    return end == text.c_str() + text.size() && std::isfinite(value);
}

bool sameOutput(const WristHapticState& a, const WristHapticState& b)
{
    return a.pattern == b.pattern && a.duty_pct == b.duty_pct && a.frequency_hz == b.frequency_hz;
}

} // namespace

const char* wristHapticPatternName(WristHapticPattern pattern)
{
    switch (pattern)
    {
    case WristHapticPattern::SILENT: return "SILENT";
    case WristHapticPattern::COLLISION: return "COLLISION";
    case WristHapticPattern::UNKNOWN_PERSON: return "UNKNOWN_PERSON";
    case WristHapticPattern::SAFE_TAGGED: return "SAFE_TAGGED";
    case WristHapticPattern::PRESENCE_ONLY: return "PRESENCE_ONLY";
    }
    return "SILENT";
}

WristGuidance::WristGuidance() : WristGuidance(WristConfig{}) {}
WristGuidance::WristGuidance(const WristConfig& config) : config_(config) {}

void WristGuidance::reset() { aimed_ = false; }

WristState WristGuidance::update(const std::optional<MarkerObservation>& marker, uint64_t now_ms)
{
    WristState state{};
    if (!config_.enabled || !marker || !marker->valid)
    {
        aimed_ = false;
        return state;
    }

    state.marker_detected = true;
    state.confidence = clampf(marker->confidence, 0.0f, 1.0f);
    state.bearing_deg = radToDeg(wrapRad(marker->bearing_rad + degToRad(config_.camera_yaw_offset_deg)));

    const uint64_t age_ms = now_ms >= marker->timestamp_ms ? now_ms - marker->timestamp_ms : 0;
    if (age_ms > config_.max_age_ms || marker->confidence < config_.min_confidence)
    {
        aimed_ = false;
        return state;
    }

    const float magnitude = std::abs(state.bearing_deg);
    aimed_ = aimed_ ? magnitude <= config_.release_deg : magnitude <= config_.engage_deg;
    state.aimed = aimed_;
    return state;
}

WristVibrator::WristVibrator() : WristVibrator(WristConfig{}) {}
WristVibrator::WristVibrator(const WristConfig& config) : config_(config) {}
WristVibrator::~WristVibrator() { stop(); }

bool WristVibrator::start(std::string& error)
{
    stop();
    if (!config_.enabled) return true;
    if (!config_.device_path.empty())
    {
        device_.open(config_.device_path, std::ios::out | std::ios::binary);
        if (!device_)
        {
            error = "cannot open wrist device " + config_.device_path;
            return false;
        }
    }
    running_ = true;
    on_ = false;
    if (device_.is_open())
    {
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            stop_output_ = false;
            pending_output_ = false;
            has_pending_output_ = true;
        }
        output_thread_ = std::thread(&WristVibrator::outputLoop, this);
        output_ready_.notify_one();
    }
    return true;
}

void WristVibrator::setOutput(bool on)
{
    if (!device_.is_open()) return;
    device_.seekp(0);
    device_ << (on ? config_.active_value : config_.inactive_value);
    device_.flush();
    if (!device_) std::cerr << "[wrist] failed to write actuator state\n";
}

void WristVibrator::outputLoop()
{
    while (true)
    {
        bool output = false;
        {
            std::unique_lock<std::mutex> lock(output_mutex_);
            output_ready_.wait(lock, [this] { return has_pending_output_ || stop_output_; });
            if (!has_pending_output_ && stop_output_) return;
            output = pending_output_;
            has_pending_output_ = false;
        }
        setOutput(output);
    }
}

void WristVibrator::update(uint64_t now_ms, bool aimed)
{
    (void)now_ms;
    if (!running_ || aimed == on_) return;
    on_ = aimed;
    if (device_.is_open())
    {
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            pending_output_ = aimed;
            has_pending_output_ = true;
        }
        output_ready_.notify_one();
    }
    if (config_.verbose)
        std::clog << "[wrist] " << (aimed ? "marker aimed: vibrate" : "marker lost: stop") << "\n";
}

void WristVibrator::stop()
{
    running_ = false;
    if (output_thread_.joinable())
    {
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            pending_output_ = false;
            has_pending_output_ = true;
            stop_output_ = true;
        }
        output_ready_.notify_one();
        output_thread_.join();
    }
    on_ = false;
    if (device_.is_open()) device_.close();
}

WristHaptics::WristHaptics() : WristHaptics(WristConfig{}) {}
WristHaptics::WristHaptics(const WristConfig& config)
    : config_(config), calibrated_yaw_offset_deg_(config.wrist_yaw_offset_deg)
{
}

WristHaptics::~WristHaptics() { stop(); }

bool WristHaptics::start(std::string& error)
{
    stop();
    reset();
    if (!config_.enabled) return true;
    if (!config_.module_port.empty())
    {
        if (!module_.open(config_.module_port, config_.module_baud_rate, error)) return false;
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            stop_output_ = false;
            pending_output_ = WristHapticState{};
            has_pending_output_ = true;
        }
        running_ = true;
        output_thread_ = std::thread(&WristHaptics::outputLoop, this);
        output_ready_.notify_one();
        return true;
    }
    running_ = true;
    return true;
}

void WristHaptics::stop()
{
    running_ = false;
    if (output_thread_.joinable())
    {
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            pending_output_ = WristHapticState{};
            has_pending_output_ = true;
            stop_output_ = true;
        }
        output_ready_.notify_one();
        output_thread_.join();
    }
    module_.close();
    line_buffer_.clear();
    glove_packet_bytes_ = 0;
}

void WristHaptics::reset()
{
    line_buffer_.clear();
    glove_packet_bytes_ = 0;
    latest_orientation_.reset();
    calibrated_ = false;
    calibrated_yaw_offset_deg_ = config_.wrist_yaw_offset_deg;
    last_state_ = WristHapticState{};
    glove_raycaster_.reset();
    glove_ray_state_ = GloveRayState{};
}

void WristHaptics::disableGloveScan()
{
    glove_raycaster_.disableScan();
    glove_ray_state_ = GloveRayState{};
}

bool WristHaptics::parseOrientationLine(const std::string& line,
                                        float& yaw_deg,
                                        float& pitch_deg,
                                        std::string& error)
{
    std::istringstream input(trim(line));
    std::string prefix;
    std::string yaw_text;
    std::string pitch_text;
    std::string extra;
    if (!std::getline(input, prefix, ',') ||
        !std::getline(input, yaw_text, ',') ||
        !std::getline(input, pitch_text, ',') ||
        std::getline(input, extra, ',') ||
        trim(prefix) != "WRIST" ||
        !parseFloat(trim(yaw_text), yaw_deg) ||
        !parseFloat(trim(pitch_text), pitch_deg) ||
        std::abs(yaw_deg) > 720.0f ||
        pitch_deg < -90.0f || pitch_deg > 90.0f)
    {
        error = "invalid wrist orientation (expected WRIST,yaw_deg,pitch_deg)";
        return false;
    }
    error.clear();
    return true;
}

bool WristHaptics::poll(uint64_t timestamp_ms, WristOrientation& orientation, std::string& error)
{
    error.clear();
    if (!module_.isOpen()) return false;

    std::vector<uint8_t> bytes;
    if (!module_.readAvailable(bytes, error)) return false;

    bool received = false;
    std::string parse_error;
    for (const uint8_t byte : bytes)
    {
        if (glove_packet_bytes_ > 0 || byte == 0x5B)
        {
            glove_packet_buffer_[glove_packet_bytes_++] = byte;
            if (glove_packet_bytes_ == glove_packet_buffer_.size())
            {
                GlovePosePacket packet{};
                if (parseGlovePosePacket(glove_packet_buffer_.data(),
                                         glove_packet_buffer_.size(),
                                         packet, parse_error))
                {
                    WristOrientation sample{};
                    sample.timestamp_ms = timestamp_ms;
                    sample.wrist_yaw_deg = packet.yaw_deg;
                    sample.wrist_pitch_deg = packet.pitch_deg;
                    sample.glove_packet = true;
                    sample.pointing_active = packet.trigger_held;
                    sample.zero_heading_requested = packet.zero_heading_requested;
                    sample.glove_events = packet.events;
                    orientation = sample;
                    latest_orientation_ = sample;
                    received = true;
                    parse_error.clear();
                }
                glove_packet_bytes_ = 0;
            }
            continue;
        }

        if (byte == '\n')
        {
            const std::string line = trim(line_buffer_);
            line_buffer_.clear();
            if (line.empty()) continue;

            WristOrientation sample{};
            sample.timestamp_ms = timestamp_ms;
            if (parseOrientationLine(line, sample.wrist_yaw_deg, sample.wrist_pitch_deg, parse_error))
            {
                orientation = sample;
                latest_orientation_ = sample;
                received = true;
                parse_error.clear();
            }
            continue;
        }

        if (line_buffer_.size() >= 256)
        {
            line_buffer_.clear();
            parse_error = "wrist orientation line exceeds 256 bytes";
            continue;
        }
        line_buffer_.push_back(static_cast<char>(byte));
    }

    error = parse_error;
    return received;
}

WristHapticState WristHaptics::update(const WorldModel& world,
                                     const std::optional<WristOrientation>& orientation,
                                     uint64_t timestamp_ms,
                                     bool collision_warning)
{
    if (orientation && std::isfinite(orientation->wrist_yaw_deg) &&
        std::isfinite(orientation->wrist_pitch_deg))
        latest_orientation_ = orientation;

    WristHapticState state{};
    const bool orientation_fresh = latest_orientation_ &&
        timestamp_ms >= latest_orientation_->timestamp_ms &&
        timestamp_ms - latest_orientation_->timestamp_ms <= config_.orientation_timeout_ms;
    if (orientation_fresh)
    {
        if (!calibrated_)
        {
            calibrated_yaw_offset_deg_ = wrapDeg(config_.wrist_yaw_offset_deg -
                                                 latest_orientation_->wrist_yaw_deg);
            calibrated_ = true;
        }
        state.orientation_valid = true;
        state.wrist_yaw_deg = wrapDeg(latest_orientation_->wrist_yaw_deg + calibrated_yaw_offset_deg_);
        state.wrist_pitch_deg = latest_orientation_->wrist_pitch_deg;
    }

    std::optional<GlovePosePacket> new_glove_packet;
    if (orientation && orientation->glove_packet)
    {
        GlovePosePacket packet{};
        packet.yaw_deg = orientation->wrist_yaw_deg;
        packet.pitch_deg = orientation->wrist_pitch_deg;
        packet.trigger_held = orientation->pointing_active;
        packet.zero_heading_requested = orientation->zero_heading_requested;
        new_glove_packet = packet;
    }
    glove_ray_state_ = glove_raycaster_.update(world, new_glove_packet, timestamp_ms);

    if (collision_warning)
    {
        state.pattern = WristHapticPattern::COLLISION;
        state.duty_pct = 100.0f;
        state.frequency_hz = 20.0f;
    }
    else if (orientation_fresh && latest_orientation_->glove_packet)
    {
        if (glove_ray_state_.active && glove_ray_state_.locked_target_id != 0)
        {
            const auto target = std::find_if(
                world.tracks.begin(), world.tracks.end(),
                [this](const Track& track) {
                    return track.id == glove_ray_state_.locked_target_id;
                });
            if (target != world.tracks.end())
            {
                state.pointing = true;
                state.wrist_yaw_deg = wrapDeg(glove_ray_state_.yaw_deg - world.pose.yaw_deg);
                state.target_bearing_deg = state.wrist_yaw_deg;
                state.target_range_m = glove_ray_state_.locked_dist_m;
                const PersonClass person_class = classifyTrack(*target);
                if (person_class == PersonClass::SAFE_TAGGED)
                {
                    state.pattern = WristHapticPattern::SAFE_TAGGED;
                    state.duty_pct = 22.0f;
                    state.frequency_hz = 4.0f;
                }
                else
                {
                    state.pattern = WristHapticPattern::UNKNOWN_PERSON;
                    state.frequency_hz = clampf(16.0f - glove_ray_state_.locked_dist_m, 4.0f, 16.0f);
                    state.duty_pct = clampf(50.0f + 5.0f * (8.0f - glove_ray_state_.locked_dist_m),
                                            40.0f, 90.0f);
                }
            }
        }
    }
    else if (state.orientation_valid && config_.enabled)
    {
        bool found = false;
        float best_error = config_.pointing_cone_deg + 1.0f;
        bool best_unknown = false;
        for (const Track& track : world.tracks)
        {
            if (track.state == TrackState::EXPIRED) continue;

            const Vec3 position = track.filter.position();
            const float dx = position.x - world.pose.x;
            const float dy = position.y - world.pose.y;
            const float range = std::sqrt(dx * dx + dy * dy);
            if (!std::isfinite(range) || range <= 1.0e-4f) continue;

            const float target_bearing = wrapDeg(radToDeg(std::atan2(dx, dy)) - world.pose.yaw_deg);
            const float angular_error = wrapDeg(target_bearing - state.wrist_yaw_deg);
            const float error_magnitude = std::abs(angular_error);
            if (error_magnitude > config_.pointing_cone_deg) continue;

            const PersonClass person_class = classifyTrack(track);
            const bool unknown = person_class == PersonClass::UNKNOWN_PERSON;
            if (found && (error_magnitude > best_error ||
                          (error_magnitude == best_error && (!unknown || best_unknown))))
                continue;

            found = true;
            best_error = error_magnitude;
            best_unknown = unknown;
            state.pointing = true;
            state.target_bearing_deg = target_bearing;
            state.angular_error_deg = angular_error;
            state.target_range_m = range;

            const float alignment = clampf(1.0f - error_magnitude / config_.pointing_cone_deg,
                                           0.0f, 1.0f);
            if (person_class == PersonClass::SAFE_TAGGED)
            {
                state.pattern = WristHapticPattern::SAFE_TAGGED;
                state.duty_pct = 22.0f;
                state.frequency_hz = 4.0f;
            }
            else if (person_class == PersonClass::PRESENCE_ONLY)
            {
                state.pattern = WristHapticPattern::PRESENCE_ONLY;
                state.duty_pct = 18.0f;
                state.frequency_hz = 2.0f;
            }
            else
            {
                const float proximity = clampf((3.0f - range) / 3.0f, 0.0f, 1.0f);
                state.pattern = WristHapticPattern::UNKNOWN_PERSON;
                state.frequency_hz = 4.0f + 16.0f * alignment;
                state.duty_pct = clampf(40.0f + 50.0f * alignment + 10.0f * proximity,
                                        0.0f, 100.0f);
            }
        }
    }

    if (!sameOutput(state, last_state_)) queueOutput(state);
    last_state_ = state;
    return state;
}

void WristHaptics::queueOutput(const WristHapticState& state)
{
    if (!running_ || !module_.isOpen() || !output_thread_.joinable()) return;
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        pending_output_ = state;
        has_pending_output_ = true;
    }
    output_ready_.notify_one();
}

void WristHaptics::outputLoop()
{
    while (true)
    {
        WristHapticState state{};
        {
            std::unique_lock<std::mutex> lock(output_mutex_);
            output_ready_.wait(lock, [this] { return has_pending_output_ || stop_output_; });
            if (!has_pending_output_ && stop_output_) return;
            state = pending_output_;
            has_pending_output_ = false;
        }

        std::ostringstream frame;
        frame << "HAPTIC," << std::fixed << std::setprecision(1)
              << state.duty_pct << ',' << state.frequency_hz << ','
              << wristHapticPatternName(state.pattern) << '\n';
        std::string error;
        if (!module_.writeAll(frame.str(), error))
            std::cerr << "[wrist] asynchronous haptic write failed: " << error << "\n";
    }
}

} // namespace sa
