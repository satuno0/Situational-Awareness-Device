#include "announcer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <objbase.h>
#include <sapi.h>
#endif

namespace sa {
namespace {

std::string countPhrase(int count, const char* singular, const char* plural)
{
    return std::to_string(count) + " " + (count == 1 ? singular : plural);
}

std::string distancePhrase(float range_m)
{
    int meters = static_cast<int>(std::lround(std::max(0.0f, range_m)));
    if (meters < 1) meters = 1;
    return std::to_string(meters) + (meters == 1 ? " meter" : " meters");
}

// Positive bearing is right of the device forward axis.
std::string directionPhrase(float bearing_deg)
{
    const float magnitude = std::abs(bearing_deg);
    if (magnitude <= 22.5f) return "ahead";
    if (magnitude <= 67.5f) return bearing_deg > 0.0f ? "to the right" : "to the left";
    if (magnitude <= 112.5f) return bearing_deg > 0.0f ? "on the right" : "on the left";
    if (magnitude <= 157.5f) return bearing_deg > 0.0f ? "behind and to the right" : "behind and to the left";
    return "behind";
}

const char* classLabel(PersonClass cls)
{
    return personClassName(cls);
}

struct Entry
{
    float range_m = 0.0f;
    float bearing_deg = 0.0f;
    PersonClass cls = PersonClass::UNKNOWN_PERSON;
    uint32_t id = 0;
};

} // namespace

struct Announcer::SpeechBackend
{
#ifdef _WIN32
    ISpVoice* voice = nullptr;
    bool owns_com = false;

    ~SpeechBackend()
    {
        if (voice)
        {
            voice->Speak(L"", SPF_PURGEBEFORESPEAK, nullptr);
            voice->Release();
        }
        if (owns_com) CoUninitialize();
    }

    bool start(std::string& error)
    {
        HRESULT result = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
        if (FAILED(result))
        {
            error = "CoInitializeEx failed for Windows speech";
            return false;
        }
        owns_com = true;

        result = CoCreateInstance(CLSID_SpVoice, nullptr, CLSCTX_ALL, IID_ISpVoice,
                                  reinterpret_cast<void**>(&voice));
        if (FAILED(result))
        {
            std::ostringstream message;
            message << "cannot create Windows speech voice (HRESULT 0x" << std::hex
                    << static_cast<unsigned long>(result) << ")";
            error = message.str();
            return false;
        }
        return true;
    }

    bool speak(const std::string& text, std::string& error)
    {
        const std::wstring wide_text(text.begin(), text.end());
        const HRESULT result = voice->Speak(wide_text.c_str(), SPF_ASYNC, nullptr);
        if (SUCCEEDED(result)) return true;

        std::ostringstream message;
        message << "Windows speech failed (HRESULT 0x" << std::hex
                << static_cast<unsigned long>(result) << ")";
        error = message.str();
        return false;
    }
#else
    bool start(std::string& error)
    {
        error = "Windows text-to-speech is available only on Windows";
        return false;
    }

    bool speak(const std::string&, std::string&) { return false; }
#endif
};

std::string buildAnnouncement(const WorldModel& world, const AnnounceConfig& config)
{
    std::vector<Entry> entries;
    entries.reserve(world.tracks.size());
    for (const auto& track : world.tracks)
    {
        if (track.state == TrackState::EXPIRED) continue;
        if (track.existence_confidence < config.min_existence) continue;

        const Vec3 body = poseInverseApply(world.pose, track.filter.position());
        const float range = std::sqrt(body.x * body.x + body.y * body.y);
        if (!std::isfinite(range) || range < config.min_report_range_m || range > config.max_range_m) continue;

        Entry entry;
        entry.range_m = range;
        entry.bearing_deg = radToDeg(std::atan2(body.x, body.y));
        entry.cls = classifyTrack(track);
        entry.id = track.id;
        entries.push_back(entry);
    }

    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
        if (a.range_m != b.range_m) return a.range_m < b.range_m;
        return a.id < b.id;
    });

    if (entries.empty())
    {
        if (!world.zones.empty()) return "PRESENCE_ONLY detected; no person track.";
        return "Area clear.";
    }

    int unknown_people = 0;
    int safe_tagged = 0;
    int presence_only = 0;
    for (const auto& entry : entries)
    {
        if (entry.cls == PersonClass::SAFE_TAGGED) ++safe_tagged;
        else if (entry.cls == PersonClass::PRESENCE_ONLY) ++presence_only;
        else ++unknown_people;
    }

    std::ostringstream out;
    const int total = static_cast<int>(entries.size());
    if (total == 1)
    {
        const auto& entry = entries.front();
        out << classLabel(entry.cls) << " " << distancePhrase(entry.range_m) << " "
            << directionPhrase(entry.bearing_deg) << ".";
        return out.str();
    }

    out << total << " people:";
    bool first = true;
    const auto appendCount = [&](int count, const char* singular, const char* plural) {
        if (count <= 0) return;
        out << (first ? " " : ", ") << countPhrase(count, singular, plural);
        first = false;
    };
    appendCount(unknown_people, "UNKNOWN_PERSON", "UNKNOWN_PERSON");
    appendCount(safe_tagged, "SAFE_TAGGED", "SAFE_TAGGED");
    appendCount(presence_only, "PRESENCE_ONLY", "PRESENCE_ONLY");
    out << ".";

    const std::size_t limit = std::min(entries.size(), std::max<std::size_t>(1, config.max_entries));
    for (std::size_t i = 0; i < limit; ++i)
    {
        const auto& entry = entries[i];
        out << " " << classLabel(entry.cls) << " " << distancePhrase(entry.range_m) << " "
            << directionPhrase(entry.bearing_deg) << ".";
    }
    return out.str();
}

Announcer::Announcer() : Announcer(AnnounceConfig{}) {}
Announcer::Announcer(const AnnounceConfig& config) : config_(config) {}
Announcer::~Announcer() { stop(); }

bool Announcer::start(std::string& error)
{
    stop();
    if (!config_.enabled) return true;
    if (!config_.device_path.empty())
    {
        device_.open(config_.device_path, std::ios::out | std::ios::app);
        if (!device_)
        {
            error = "cannot open announcement device " + config_.device_path;
            return false;
        }
    }
    if (config_.windows_speech)
    {
        speech_ = std::make_unique<SpeechBackend>();
        if (!speech_->start(error))
        {
            speech_.reset();
            if (device_.is_open()) device_.close();
            return false;
        }
    }
    running_ = true;
    last_emit_ms_.reset();
    last_text_.clear();
    if (device_.is_open())
    {
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            device_queue_.clear();
            stop_device_thread_ = false;
        }
        device_thread_ = std::thread(&Announcer::deviceLoop, this);
    }
    return true;
}

void Announcer::stop()
{
    running_ = false;
    if (device_thread_.joinable())
    {
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            stop_device_thread_ = true;
        }
        device_ready_.notify_one();
        device_thread_.join();
    }
    if (device_.is_open()) device_.close();
    speech_.reset();
    std::lock_guard<std::mutex> lock(device_mutex_);
    device_queue_.clear();
    stop_device_thread_ = false;
}

void Announcer::deviceLoop()
{
    while (true)
    {
        std::string line;
        {
            std::unique_lock<std::mutex> lock(device_mutex_);
            device_ready_.wait(lock, [this] { return !device_queue_.empty() || stop_device_thread_; });
            if (device_queue_.empty() && stop_device_thread_) return;
            line = std::move(device_queue_.front());
            device_queue_.pop_front();
        }
        device_ << line << "\n";
        device_.flush();
        if (!device_) std::cerr << "[announce] failed to write speaker bridge output\n";
    }
}

void Announcer::emit(const std::string& text, uint64_t now_ms)
{
    if (device_.is_open())
    {
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            device_queue_.push_back("SAY " + text);
        }
        device_ready_.notify_one();
    }
    if (speech_)
    {
        std::string error;
        if (!speech_->speak(text, error))
            std::cerr << "[announce] " << error << "\n";
    }
    last_emit_ms_ = now_ms;
    last_text_ = text;
    if (config_.verbose) std::clog << "[announce] " << text << "\n";
}

void Announcer::update(const WorldModel& world, uint64_t now_ms)
{
    announceText(buildAnnouncement(world, config_), now_ms);
}

void Announcer::announceText(const std::string& text, uint64_t now_ms)
{
    if (!config_.enabled || !running_) return;
    if (text.empty()) return;

    uint64_t required_ms = static_cast<uint64_t>(
        (text != last_text_ ? config_.min_interval_s : config_.repeat_interval_s) * 1000.0);
    if (!last_emit_ms_ || now_ms < *last_emit_ms_)
    {
        emit(text, now_ms);
        return;
    }
    if (now_ms - *last_emit_ms_ >= required_ms) emit(text, now_ms);
}

} // namespace sa
