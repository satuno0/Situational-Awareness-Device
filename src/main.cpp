#include "announcer.hpp"
#include "fusion_engine.hpp"
#include "haptics.hpp"
#include "imu_driver.hpp"
#include "marker_driver.hpp"
#include "novelda_driver.hpp"
#include "playback.hpp"
#include "radar_driver.hpp"
#include "recorder.hpp"
#include "schematic.hpp"
#include "sensor_mode.hpp"
#include "simulation.hpp"
#include "telemetry.hpp"
#include "ultrasonic_driver.hpp"
#include "uwb_driver.hpp"
#include "wrist_haptics.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace sa;

std::atomic<bool> g_running{true};

void handleSignal(int) { g_running = false; }

struct Options
{
    bool show_help = false;
    bool simulate = false;
    bool replay = false;
    std::string replay_path;

    bool record = false;
    std::string record_path = "session.bin";

    double duration_s = 0.0;
    bool verify = false;

    bool telemetry = true;
    std::string bind_address = "127.0.0.1";
    uint16_t telemetry_port = 8080;
    std::string web_root = "web";

    bool haptics = true;
    std::string haptic_device;

    bool wrist = true;
    std::string wrist_device;
    std::string marker_port;
    float marker_offset_deg = 0.0f;
    float marker_engage_deg = 8.0f;
    float marker_release_deg = 14.0f;
    bool simulate_marker = true;

    SensorSelection sensors;

    bool announce = false;
    std::string announce_device;
    double announce_interval_s = 1.5;
    double announce_repeat_s = 8.0;

    std::string map_path;

    bool verbose = false;
    bool realtime_sim = false;
    double loop_rate_hz = 20.0;
    uint32_t seed = 0x5A17u;

    std::string radar_data_port;
    std::string radar_cli_port;
    std::string radar_config_file;
    std::string imu_device = "/dev/i2c-1";
    std::string uwb_port;
    std::string novelda_port;
    std::string ultrasonic_port;
};

void printUsage(const char* program)
{
    std::cout
        << "sa_daemon - handheld situational awareness daemon\n\n"
        << "Usage:\n"
        << "  " << program << " [options]\n\n"
        << "Modes:\n"
        << "  (default)              Run from hardware serial/I2C devices\n"
        << "  --simulate             Run deterministic synthetic sensors\n"
        << "  --replay <session.bin> Re-run fusion from a recorded raw session\n\n"
        << "Session options:\n"
        << "  --record[=path]        Record raw sensors to session.bin (default path)\n"
        << "  --duration <seconds>   Stop after N seconds (simulation default 60)\n"
        << "  --verify               During replay, compare regenerated outputs with\n"
        << "                         recorded reference snapshots\n"
        << "  --seed <uint32>        Simulation RNG seed\n"
        << "  --realtime             Pace simulation to wall-clock time (for live\n"
        << "                         telemetry); otherwise it runs as fast as possible\n"
        << "  --rate <hz>            Hardware poll/process rate (default 20)\n\n"
        << "Telemetry:\n"
        << "  --port <port>          HTTP port, 0 picks an ephemeral port (default 8080)\n"
        << "  --bind <address>       Bind address (default 127.0.0.1; use 0.0.0.0 for phone)\n"
        << "  --web-root <dir>       Directory containing index.html (default web)\n"
        << "  --no-telemetry         Disable the HTTP server\n\n"
        << "Haptics:\n"
        << "  --no-haptics           Disable the haptic worker\n"
        << "  --haptic-device <path> Write motor state to this device/file\n"
        << "  --verbose              Log haptic events and progress\n\n"
        << "Red marker / wrist guidance:\n"
        << "  --no-wrist             Disable wrist guidance\n"
        << "  --wrist-device <path>  Write wrist motor state (1/0) to this device/file\n"
        << "  --marker <dev>         Camera-bridge serial port (MARK/RED lines)\n"
        << "  --marker-offset <deg>  Camera yaw offset relative to device forward\n"
        << "  --marker-engage <deg>  Aim cone that starts vibration (default 8)\n"
        << "  --marker-release <deg> Aim cone that stops vibration (default 14)\n"
        << "  --no-marker-sim        Do not inject the synthetic marker in simulation\n\n"
        << "Perception / audio / map:\n"
        << "  --sensors <mode>       mixed, radar, uwb, novelda, ultrasonic, vision,\n"
        << "                         imu, or a comma list such as radar,uwb\n"
        << "  --announce             Speak track summaries through an earpiece/speaker\n"
        << "  --announce-device <p>  Line-oriented speaker bridge (SAY <text> lines)\n"
        << "  --announce-interval <s>  Debounce for changed scenes (default 1.5)\n"
        << "  --announce-repeat <s>    Repeat unchanged summaries (default 8)\n"
        << "  --map <file>           Load a text floor plan / schematic as a prior\n\n"
        << "Hardware ports:\n"
        << "  --radar-data <dev>     IWR6843 data port\n"
        << "  --radar-cli <dev>      IWR6843 CLI port (for .cfg upload)\n"
        << "  --radar-cfg <file>     mmWave .cfg file\n"
        << "  --imu <dev>            ICM-42688 I2C device (default /dev/i2c-1)\n"
        << "  --uwb <dev>            DWM3000 bridge serial port\n"
        << "  --novelda <dev>        XeThru bridge serial port\n"
        << "  --ultrasonic <dev>     MaxBotix serial port\n\n"
        << "  --help                 Show this help\n";
}

bool startsWith(const std::string& text, const std::string& prefix)
{
    return text.rfind(prefix, 0) == 0;
}

bool takeValue(const std::string& arg, int& index, int argc, char** argv, const std::string& name,
               std::string& value, std::string& error)
{
    if (arg == name)
    {
        if (index + 1 >= argc)
        {
            error = "missing value for " + name;
            return false;
        }
        value = argv[++index];
        return true;
    }
    const std::string prefix = name + "=";
    if (startsWith(arg, prefix))
    {
        value = arg.substr(prefix.size());
        return true;
    }
    return false;
}

bool matches(const std::string& arg, const std::string& name)
{
    return arg == name || startsWith(arg, name + "=");
}

bool parseDouble(const std::string& text, double& out)
{
    if (text.empty()) return false;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) return false;
    out = value;
    return true;
}

bool parseUInt(const std::string& text, uint32_t& out)
{
    if (text.empty()) return false;
    char* end = nullptr;
    const unsigned long value = std::strtoul(text.c_str(), &end, 0);
    if (end == text.c_str() || *end != '\0' || value > 0xFFFFFFFFul) return false;
    out = static_cast<uint32_t>(value);
    return true;
}

bool parseFiniteFloat(const std::string& text, float& out)
{
    double value = 0.0;
    if (!parseDouble(text, value)) return false;
    if (std::fabs(value) > 1.0e6) return false;
    out = static_cast<float>(value);
    return true;
}

bool parseOptions(int argc, char** argv, Options& options, std::string& error)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") { options.show_help = true; continue; }
        if (arg == "--simulate") { options.simulate = true; continue; }
        if (arg == "--verify") { options.verify = true; continue; }
        if (arg == "--verbose") { options.verbose = true; continue; }
        if (arg == "--realtime") { options.realtime_sim = true; continue; }
        if (arg == "--telemetry") { options.telemetry = true; continue; }
        if (arg == "--no-telemetry") { options.telemetry = false; continue; }
        if (arg == "--haptics") { options.haptics = true; continue; }
        if (arg == "--no-haptics") { options.haptics = false; continue; }
        if (arg == "--wrist") { options.wrist = true; continue; }
        if (arg == "--no-wrist") { options.wrist = false; continue; }
        if (arg == "--no-marker-sim") { options.simulate_marker = false; continue; }
        if (arg == "--announce") { options.announce = true; continue; }
        if (arg == "--no-announce") { options.announce = false; continue; }

        if (arg == "--record")
        {
            options.record = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') options.record_path = argv[++i];
            continue;
        }
        if (startsWith(arg, "--record="))
        {
            options.record = true;
            options.record_path = arg.substr(std::string("--record=").size());
            continue;
        }

        if (matches(arg, "--replay"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--replay", value, error)) return false;
            if (value.empty())
            {
                error = "--replay requires a session file";
                return false;
            }
            options.replay = true;
            options.replay_path = value;
            continue;
        }
        if (matches(arg, "--duration"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--duration", value, error)) return false;
            if (!parseDouble(value, options.duration_s) || options.duration_s < 0.0)
            {
                error = "invalid --duration: " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--seed"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--seed", value, error)) return false;
            if (!parseUInt(value, options.seed))
            {
                error = "invalid --seed: " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--rate"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--rate", value, error)) return false;
            if (!parseDouble(value, options.loop_rate_hz) || options.loop_rate_hz < 1.0 ||
                options.loop_rate_hz > 200.0)
            {
                error = "invalid --rate (expected 1..200): " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--port"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--port", value, error)) return false;
            uint32_t port = 0;
            if (!parseUInt(value, port) || port > 65535u)
            {
                error = "invalid --port: " + value;
                return false;
            }
            options.telemetry_port = static_cast<uint16_t>(port);
            continue;
        }
        if (matches(arg, "--bind"))
        {
            if (!takeValue(arg, i, argc, argv, "--bind", options.bind_address, error)) return false;
            continue;
        }
        if (matches(arg, "--web-root"))
        {
            if (!takeValue(arg, i, argc, argv, "--web-root", options.web_root, error)) return false;
            continue;
        }
        if (matches(arg, "--haptic-device"))
        {
            if (!takeValue(arg, i, argc, argv, "--haptic-device", options.haptic_device, error)) return false;
            continue;
        }
        if (matches(arg, "--wrist-device"))
        {
            if (!takeValue(arg, i, argc, argv, "--wrist-device", options.wrist_device, error)) return false;
            continue;
        }
        if (matches(arg, "--sensors"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--sensors", value, error)) return false;
            if (!parseSensorSelection(value, options.sensors, error)) return false;
            continue;
        }
        if (matches(arg, "--announce-device"))
        {
            if (!takeValue(arg, i, argc, argv, "--announce-device", options.announce_device, error)) return false;
            continue;
        }
        if (matches(arg, "--announce-interval"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--announce-interval", value, error)) return false;
            if (!parseDouble(value, options.announce_interval_s) || options.announce_interval_s < 0.0)
            {
                error = "invalid --announce-interval: " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--announce-repeat"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--announce-repeat", value, error)) return false;
            if (!parseDouble(value, options.announce_repeat_s) || options.announce_repeat_s < 0.0)
            {
                error = "invalid --announce-repeat: " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--map"))
        {
            if (!takeValue(arg, i, argc, argv, "--map", options.map_path, error)) return false;
            continue;
        }
        if (matches(arg, "--marker"))
        {
            if (!takeValue(arg, i, argc, argv, "--marker", options.marker_port, error)) return false;
            continue;
        }
        if (matches(arg, "--marker-offset"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--marker-offset", value, error)) return false;
            if (!parseFiniteFloat(value, options.marker_offset_deg) ||
                options.marker_offset_deg < -180.0f || options.marker_offset_deg > 180.0f)
            {
                error = "invalid --marker-offset (expected -180..180): " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--marker-engage"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--marker-engage", value, error)) return false;
            if (!parseFiniteFloat(value, options.marker_engage_deg) ||
                options.marker_engage_deg < 0.0f || options.marker_engage_deg > 90.0f)
            {
                error = "invalid --marker-engage (expected 0..90): " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--marker-release"))
        {
            std::string value;
            if (!takeValue(arg, i, argc, argv, "--marker-release", value, error)) return false;
            if (!parseFiniteFloat(value, options.marker_release_deg) ||
                options.marker_release_deg < 0.0f || options.marker_release_deg > 90.0f)
            {
                error = "invalid --marker-release (expected 0..90): " + value;
                return false;
            }
            continue;
        }
        if (matches(arg, "--radar-data"))
        {
            if (!takeValue(arg, i, argc, argv, "--radar-data", options.radar_data_port, error)) return false;
            continue;
        }
        if (matches(arg, "--radar-cli"))
        {
            if (!takeValue(arg, i, argc, argv, "--radar-cli", options.radar_cli_port, error)) return false;
            continue;
        }
        if (matches(arg, "--radar-cfg"))
        {
            if (!takeValue(arg, i, argc, argv, "--radar-cfg", options.radar_config_file, error)) return false;
            continue;
        }
        if (matches(arg, "--imu"))
        {
            if (!takeValue(arg, i, argc, argv, "--imu", options.imu_device, error)) return false;
            continue;
        }
        if (matches(arg, "--uwb"))
        {
            if (!takeValue(arg, i, argc, argv, "--uwb", options.uwb_port, error)) return false;
            continue;
        }
        if (matches(arg, "--novelda"))
        {
            if (!takeValue(arg, i, argc, argv, "--novelda", options.novelda_port, error)) return false;
            continue;
        }
        if (matches(arg, "--ultrasonic"))
        {
            if (!takeValue(arg, i, argc, argv, "--ultrasonic", options.ultrasonic_port, error)) return false;
            continue;
        }

        error = "unknown option: " + arg;
        return false;
    }

    if (options.simulate && options.replay)
    {
        error = "--simulate and --replay are mutually exclusive";
        return false;
    }
    if (options.replay && options.record)
    {
        error = "--record is not supported together with --replay";
        return false;
    }
    if (options.record && options.record_path.empty())
    {
        error = "--record path is empty";
        return false;
    }
    if (options.marker_release_deg < options.marker_engage_deg)
    {
        error = "--marker-release must be greater than or equal to --marker-engage";
        return false;
    }
    return true;
}

bool closeEnough(float a, float b, float epsilon)
{
    if (!std::isfinite(a) || !std::isfinite(b)) return a == b;
    return std::fabs(a - b) <= epsilon;
}

struct VerifyReport
{
    uint64_t compared_worlds = 0;
    uint64_t compared_track_sets = 0;
    uint64_t failures = 0;
    std::vector<std::string> messages;

    void fail(const std::string& message)
    {
        ++failures;
        if (messages.size() < 12) messages.push_back(message);
    }
};

void compareTracks(const std::vector<Track>& expected, const std::vector<Track>& actual,
                   uint64_t timestamp_ms, VerifyReport& report)
{
    if (expected.size() != actual.size())
    {
        report.fail("timestamp " + std::to_string(timestamp_ms) + ": track count " +
                    std::to_string(expected.size()) + " != " + std::to_string(actual.size()));
        return;
    }
    for (std::size_t i = 0; i < expected.size(); ++i)
    {
        const Track& e = expected[i];
        const Track& a = actual[i];
        if (e.id != a.id)
        {
            report.fail("timestamp " + std::to_string(timestamp_ms) + ": track id mismatch");
            continue;
        }
        const Vec3 ep = e.filter.position();
        const Vec3 ap = a.filter.position();
        const Vec3 ev = e.filter.velocity();
        const Vec3 av = a.filter.velocity();
        if (!closeEnough(ep.x, ap.x, 1.0e-4f) || !closeEnough(ep.y, ap.y, 1.0e-4f) ||
            !closeEnough(ep.z, ap.z, 1.0e-4f))
        {
            report.fail("timestamp " + std::to_string(timestamp_ms) + ": track " + std::to_string(e.id) +
                        " position mismatch");
        }
        if (!closeEnough(ev.x, av.x, 1.0e-4f) || !closeEnough(ev.y, av.y, 1.0e-4f) ||
            !closeEnough(ev.z, av.z, 1.0e-4f))
        {
            report.fail("timestamp " + std::to_string(timestamp_ms) + ": track " + std::to_string(e.id) +
                        " velocity mismatch");
        }
        if (!closeEnough(e.existence_confidence, a.existence_confidence, 1.0e-4f))
        {
            report.fail("timestamp " + std::to_string(timestamp_ms) + ": track " + std::to_string(e.id) +
                        " existence mismatch");
        }
        if (e.identity.tagged_safe != a.identity.tagged_safe || e.identity.tag_id != a.identity.tag_id)
        {
            report.fail("timestamp " + std::to_string(timestamp_ms) + ": track " + std::to_string(e.id) +
                        " identity mismatch");
        }
        if (e.state != a.state || e.visible != a.visible)
        {
            report.fail("timestamp " + std::to_string(timestamp_ms) + ": track " + std::to_string(e.id) +
                        " state mismatch");
        }
    }
}

void compareWorld(const WorldModel& expected, const WorldModel& actual, VerifyReport& report)
{
    ++report.compared_worlds;
    if (expected.timestamp_ms != actual.timestamp_ms)
    {
        report.fail("world timestamp " + std::to_string(expected.timestamp_ms) + " != " +
                    std::to_string(actual.timestamp_ms));
    }
    if (!closeEnough(expected.pose.x, actual.pose.x, 1.0e-4f) ||
        !closeEnough(expected.pose.y, actual.pose.y, 1.0e-4f) ||
        !closeEnough(expected.pose.yaw_deg, actual.pose.yaw_deg, 1.0e-3f))
    {
        report.fail("timestamp " + std::to_string(expected.timestamp_ms) + ": pose mismatch");
    }
    if (!closeEnough(expected.environment_confidence, actual.environment_confidence, 1.0e-4f))
    {
        report.fail("timestamp " + std::to_string(expected.timestamp_ms) + ": environment confidence mismatch");
    }
    compareTracks(expected.tracks, actual.tracks, expected.timestamp_ms, report);
    if (expected.zones.size() != actual.zones.size())
    {
        report.fail("timestamp " + std::to_string(expected.timestamp_ms) + ": presence zone count mismatch");
    }

    const auto expected_grid = expected.occupancy.encodedProbabilities();
    const auto actual_grid = actual.occupancy.encodedProbabilities();
    if (expected_grid.size() != actual_grid.size())
    {
        report.fail("timestamp " + std::to_string(expected.timestamp_ms) + ": grid size mismatch");
    }
    else
    {
        uint64_t different = 0;
        for (std::size_t i = 0; i < expected_grid.size(); ++i)
        {
            if (expected_grid[i] != actual_grid[i]) ++different;
        }
        if (different != 0)
        {
            report.fail("timestamp " + std::to_string(expected.timestamp_ms) + ": " +
                        std::to_string(different) + " occupancy cells differ");
        }
    }
}

void logWorld(const WorldModel& world, bool verbose)
{
    if (!verbose) return;
    std::size_t visible = 0;
    std::size_t tagged = 0;
    for (const auto& track : world.tracks)
    {
        if (track.visible) ++visible;
        if (track.identity.tagged_safe) ++tagged;
    }
    std::clog << "[sa] t=" << world.timestamp_ms << "ms tracks=" << world.tracks.size()
              << " visible=" << visible << " tagged=" << tagged
              << " env=" << std::fixed << std::setprecision(3) << world.environment_confidence
              << " occupied=" << world.occupancy.occupiedCellCount()
              << " zones=" << world.zones.size() << "\n";
}

TelemetryConfig telemetryConfig(const Options& options)
{
    TelemetryConfig config;
    config.bind_address = options.bind_address;
    config.port = options.telemetry_port;
    config.web_root = options.web_root;
    config.enabled = options.telemetry;
    config.sensor_mode = sensorSelectionName(options.sensors);
    return config;
}

HapticConfig hapticConfig(const Options& options)
{
    HapticConfig config;
    config.enabled = options.haptics;
    config.verbose = options.verbose;
    config.device_path = options.haptic_device;
    return config;
}

WristConfig wristConfig(const Options& options)
{
    WristConfig config;
    config.enabled = options.wrist;
    config.device_path = options.wrist_device;
    config.camera_yaw_offset_deg = options.marker_offset_deg;
    config.engage_deg = options.marker_engage_deg;
    config.release_deg = options.marker_release_deg;
    config.verbose = options.verbose;
    return config;
}

// Retains the newest marker across frames. The marker bridge and the synthetic
// source emit at a lower rate than the fusion loop, and replay frames may not
// contain a marker record at all. Staleness is enforced by WristGuidance.
struct MarkerRuntime
{
    std::optional<MarkerObservation> latest;

    void observe(const SensorFrame& frame)
    {
        if (frame.marker) latest = frame.marker;
    }
};

WristTelemetry wristTelemetry(const WristState& state, const WristVibrator& vibrator, const WristConfig& config)
{
    WristTelemetry out;
    out.enabled = config.enabled;
    out.aimed = state.aimed;
    out.vibrating = vibrator.outputOn();
    out.bearing_deg = state.bearing_deg;
    out.confidence = state.confidence;
    out.engage_deg = config.engage_deg;
    out.release_deg = config.release_deg;
    return out;
}

AnnounceConfig announceConfig(const Options& options)
{
    AnnounceConfig config;
    config.enabled = options.announce;
    config.device_path = options.announce_device;
    config.min_interval_s = options.announce_interval_s;
    config.repeat_interval_s = options.announce_repeat_s;
    config.verbose = options.verbose;
    return config;
}

// One struct keeps the per-run guidance, speech and schematic state together.
struct RuntimeContext
{
    RuntimeContext(const WristConfig& wrist_config, const AnnounceConfig& announce_config)
        : guidance(wrist_config), vibrator(wrist_config), announcer(announce_config)
    {
    }

    MarkerRuntime markers;
    WristGuidance guidance;
    WristVibrator vibrator;
    Announcer announcer;
    Schematic schematic;
    std::string schematic_text;
};

bool startTelemetry(const Options& options, Telemetry& telemetry)
{
    std::string error;
    if (!telemetry.start(error))
    {
        if (options.telemetry) std::cerr << "[sa] telemetry warning: " << error << "\n";
        return false;
    }
    std::cout << "[sa] telemetry http://" << options.bind_address << ":" << telemetry.port()
              << " (JSON: /api/state, /api/grid)\n";
    return true;
}

bool startHaptics(Haptics& haptics)
{
    std::string error;
    if (!haptics.start(error))
    {
        std::cerr << "[sa] haptics warning: " << error << "\n";
        return false;
    }
    return true;
}

bool startWrist(WristVibrator& vibrator, const Options& options)
{
    std::string error;
    if (!vibrator.start(error))
    {
        std::cerr << "[sa] wrist warning: " << error << "\n";
        return false;
    }
    if (options.wrist && !options.wrist_device.empty())
        std::cout << "[sa] wrist output -> " << options.wrist_device << "\n";
    return true;
}

bool startAnnouncer(Announcer& announcer, const Options& options)
{
    std::string error;
    if (!announcer.start(error))
    {
        std::cerr << "[sa] announcement warning: " << error << "\n";
        return false;
    }
    if (options.announce)
        std::cout << "[sa] announcements enabled"
                  << (options.announce_device.empty() ? " (telemetry only)" : " -> " + options.announce_device)
                  << "\n";
    return true;
}

void publishAll(const WorldModel& world, const SensorFrame& frame, Telemetry& telemetry, Haptics& haptics,
                bool telemetry_enabled, RuntimeContext& runtime)
{
    runtime.markers.observe(frame);
    const WristState wrist = runtime.guidance.update(runtime.markers.latest, frame.timestamp_ms);
    runtime.vibrator.update(frame.timestamp_ms, wrist.aimed);
    runtime.announcer.update(world, frame.timestamp_ms);
    if (telemetry_enabled)
    {
        telemetry.publish(world, runtime.markers.latest,
                          wristTelemetry(wrist, runtime.vibrator, runtime.guidance.config()),
                          runtime.announcer.lastText());
    }
    haptics.update(world, frame.ultrasonic ? &*frame.ultrasonic : nullptr);
}

// Parses and activates a schematic. `origin` is only used for logging.
bool applySchematicText(const std::string& text, const std::string& origin, FusionEngine& fusion,
                        RuntimeContext& runtime)
{
    Schematic candidate;
    std::string error;
    if (!candidate.loadText(text, error))
    {
        std::cerr << "[sa] schematic " << origin << " rejected: " << error << "\n";
        return false;
    }
    runtime.schematic = candidate;
    runtime.schematic_text = text;
    fusion.setSchematic(runtime.schematic);
    std::cout << "[sa] schematic " << origin
              << " walls=" << (runtime.schematic.walls().size() + runtime.schematic.doors().size())
              << " rooms=" << runtime.schematic.rooms().size()
              << " labels=" << runtime.schematic.marks().size() << "\n";
    return true;
}

// Applies at most one uploaded schematic per fusion tick. Uploads arrive on the
// telemetry thread and are parsed here on the processing thread, so the fusion
// loop remains deterministic and single-threaded. Returns true when a new
// schematic became active.
bool applyMapUpload(Telemetry& telemetry, bool telemetry_ok, FusionEngine& fusion, RuntimeContext& runtime)
{
    if (!telemetry_ok) return false;
    std::string text;
    if (!telemetry.takeMapUpload(text)) return false;
    return applySchematicText(text, "upload", fusion, runtime);
}

// Writes the active schematic immediately after recorder.open(), before any
// frame records. Playback uses this preface to restore the structural prior.
void recordActiveSchematic(Recorder& recorder, bool& recording, const RuntimeContext& runtime)
{
    if (!recording || runtime.schematic_text.empty()) return;
    if (!recorder.writeSchematicRecord(runtime.schematic_text))
    {
        std::cerr << "[sa] failed to record schematic: " << recorder.error() << "\n";
        recording = false;
    }
}

bool openRecorder(Recorder& recorder, const std::string& path)
{
    std::string error;
    if (!recorder.open(path, error))
    {
        std::cerr << "[sa] recording disabled: " << error << "\n";
        return false;
    }
    std::cout << "[sa] recording raw session to " << path << "\n";
    return true;
}

bool loadInitialSchematic(const Options& options, FusionEngine& fusion, RuntimeContext& runtime)
{
    if (options.map_path.empty()) return true;

    std::ifstream file(options.map_path, std::ios::binary);
    if (!file)
    {
        std::cerr << "[sa] map error: cannot open schematic: " << options.map_path << "\n";
        return false;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return applySchematicText(buffer.str(), options.map_path, fusion, runtime);
}

int runSimulation(const Options& options, Telemetry& telemetry, Haptics& haptics, bool telemetry_ok,
                  RuntimeContext& runtime)
{
    SimulationConfig config;
    config.seed = options.seed;
    config.duration_s = options.duration_s > 0.0 ? options.duration_s : 60.0;
    config.marker_enabled = options.simulate_marker;
    Simulation simulation(config);

    FusionEngine fusion;
    if (!loadInitialSchematic(options, fusion, runtime)) return 1;

    Recorder recorder;
    bool recording = false;
    if (options.record) recording = openRecorder(recorder, options.record_path);
    recordActiveSchematic(recorder, recording, runtime);

    std::cout << "[sa] simulation seed=0x" << std::hex << options.seed << std::dec
              << " duration=" << config.duration_s << "s"
              << " sensors=" << sensorSelectionName(options.sensors) << "\n";

    uint64_t frames = 0;
    SensorFrame frame;
    WorldModel last_world{};
    const auto realtime_start = std::chrono::steady_clock::now();
    uint64_t realtime_origin_ms = 0;
    bool have_realtime_origin = false;
    while (g_running.load() && simulation.next(frame))
    {
        if (applyMapUpload(telemetry, telemetry_ok, fusion, runtime))
            recordActiveSchematic(recorder, recording, runtime);

        // The raw frame is recorded; only the selected sensors reach fusion.
        const SensorFrame filtered = filterSensorFrame(frame, options.sensors);
        WorldModel world = fusion.process(filtered);
        last_world = world;
        if (recording && !recorder.recordFrame(frame, &world))
        {
            std::cerr << "[sa] recording stopped: " << recorder.error() << "\n";
            recording = false;
        }
        publishAll(world, filtered, telemetry, haptics, telemetry_ok, runtime);
        ++frames;
        logWorld(world, options.verbose && (frames % 20 == 0));

        if (options.realtime_sim)
        {
            if (!have_realtime_origin)
            {
                realtime_origin_ms = frame.timestamp_ms;
                have_realtime_origin = true;
            }
            const auto target = realtime_start + std::chrono::milliseconds(frame.timestamp_ms - realtime_origin_ms);
            std::this_thread::sleep_until(target);
        }
    }

    std::cout << "[sa] simulation finished: frames=" << frames
              << " tracks=" << last_world.tracks.size()
              << " occupied=" << last_world.occupancy.occupiedCellCount() << "\n";
    if (recording)
    {
        std::cout << "[sa] recorded " << recorder.framesWritten() << " frames, " << recorder.bytesWritten()
                  << " bytes\n";
        recorder.close();
    }
    return 0;
}

int runReplay(const Options& options, Telemetry& telemetry, Haptics& haptics, bool telemetry_ok,
              RuntimeContext& runtime)
{
    Playback playback;
    std::string error;
    if (!playback.open(options.replay_path, error))
    {
        std::cerr << "[sa] replay error: " << error << "\n";
        return 1;
    }

    FusionEngine fusion;
    if (!loadInitialSchematic(options, fusion, runtime)) return 1;
    if (options.map_path.empty() && !playback.schematicText().empty())
        applySchematicText(playback.schematicText(), "recorded session", fusion, runtime);

    VerifyReport verify;
    // Reference snapshots were produced by the session's own schematic timeline.
    // A sensor filter, or a command-line map that differs from the embedded one,
    // changes the regenerated world; say so instead of reporting false failures.
    const bool schematic_matches = runtime.schematic_text == playback.schematicText();
    bool verify_active = options.verify && options.sensors.mask == kAllSensorMask && schematic_matches;
    if (options.verify && !verify_active)
        std::cerr << "[sa] verify warning: sensor filtering or a changed schematic alters the regenerated "
                     "world; reference comparison skipped\n";
    uint64_t frames = 0;
    uint64_t first_timestamp = 0;
    WorldModel last_world{};

    std::cout << "[sa] replaying " << options.replay_path
              << " sensors=" << sensorSelectionName(options.sensors) << "\n";

    PlaybackFrame playback_frame;
    bool first_frame = true;
    while (g_running.load() && playback.next(playback_frame))
    {
        // The preface record was already applied before the first tick; only
        // re-apply when the session changes schematics mid-stream. `--map`
        // replaces the initial prior, but later recorded uploads remain part of
        // the session timeline.
        if (playback_frame.schematic && *playback_frame.schematic != runtime.schematic_text &&
            !(first_frame && *playback_frame.schematic == playback.schematicText()))
            applySchematicText(*playback_frame.schematic, "recorded session", fusion, runtime);
        first_frame = false;

        // A live upload during replay is not part of the recorded session, so it
        // cannot be compared against the recorded reference snapshots.
        if (applyMapUpload(telemetry, telemetry_ok, fusion, runtime)) verify_active = false;

        const SensorFrame filtered = filterSensorFrame(playback_frame.sensors, options.sensors);
        WorldModel world = fusion.process(filtered);
        last_world = world;
        publishAll(world, filtered, telemetry, haptics, telemetry_ok, runtime);
        ++frames;

        if (verify_active)
        {
            if (playback_frame.reference_world) compareWorld(*playback_frame.reference_world, world, verify);
            if (playback_frame.reference_tracks)
            {
                ++verify.compared_track_sets;
                compareTracks(*playback_frame.reference_tracks, world.tracks, world.timestamp_ms, verify);
            }
        }

        if (frames == 1) first_timestamp = world.timestamp_ms;
        if (options.verbose && frames % 20 == 0) logWorld(world, true);

        if (options.duration_s > 0.0 && first_timestamp != 0 &&
            world.timestamp_ms >= first_timestamp + static_cast<uint64_t>(options.duration_s * 1000.0))
        {
            break;
        }
    }

    const PlaybackStats& stats = playback.stats();
    std::cout << "[sa] replay finished: frames=" << frames << " records=" << stats.records
              << " sensor_records=" << stats.sensor_records
              << " track_snapshots=" << stats.track_snapshots
              << " world_snapshots=" << stats.world_snapshots
              << " crc_errors=" << stats.crc_errors
              << " truncated=" << (stats.truncated ? "yes" : "no") << "\n";
    std::cout << "[sa] regenerated final world: tracks=" << last_world.tracks.size()
              << " occupied=" << last_world.occupancy.occupiedCellCount()
              << " env=" << std::fixed << std::setprecision(3) << last_world.environment_confidence << "\n";

    if (!playback.error().empty() && !playback.eof())
        std::cerr << "[sa] replay warning: " << playback.error() << "\n";

    if (verify_active)
    {
        if (verify.compared_worlds == 0 && verify.compared_track_sets == 0)
        {
            std::cerr << "[sa] verification failed: session contains no reference snapshots\n";
            return 1;
        }
        std::cout << "[sa] verification: worlds=" << verify.compared_worlds
                  << " track_sets=" << verify.compared_track_sets
                  << " failures=" << verify.failures << "\n";
        for (const auto& message : verify.messages) std::cerr << "[sa] verify: " << message << "\n";
        if (verify.failures != 0 || stats.crc_errors != 0 || stats.truncated) return 1;
    }
    return 0;
}

struct HardwareRuntime
{
    // Drivers own serial/I2C handles and are intentionally non-copyable; keep
    // them behind unique_ptr so the runtime struct itself stays simple.
    std::unique_ptr<ImuDriver> imu;
    std::unique_ptr<RadarDriver> radar;
    std::unique_ptr<UwbDriver> uwb;
    std::unique_ptr<NoveldaDriver> novelda;
    std::unique_ptr<UltrasonicDriver> ultrasonic;
    std::unique_ptr<MarkerDriver> marker;
    bool imu_ok = false;
    bool radar_ok = false;
    bool uwb_ok = false;
    bool novelda_ok = false;
    bool ultrasonic_ok = false;
    bool marker_ok = false;
};

bool openHardware(const Options& options, HardwareRuntime& runtime)
{
    std::string error;

    ImuConfig imu_config;
    imu_config.i2c_device = options.imu_device;
    imu_config.enabled = true;
    runtime.imu = std::make_unique<ImuDriver>(imu_config);
    runtime.imu_ok = runtime.imu->open(error);
    if (!runtime.imu_ok) std::cerr << "[sa] IMU unavailable: " << error << "\n";

    if (!options.radar_data_port.empty())
    {
        RadarConfig radar_config;
        radar_config.data_port = options.radar_data_port;
        radar_config.cli_port = options.radar_cli_port;
        radar_config.config_file = options.radar_config_file;
        radar_config.configure_on_open = !options.radar_config_file.empty();
        runtime.radar = std::make_unique<RadarDriver>(radar_config);
        runtime.radar_ok = runtime.radar->open(error);
        if (!runtime.radar_ok) std::cerr << "[sa] radar unavailable: " << error << "\n";
    }

    if (!options.uwb_port.empty())
    {
        UwbConfig uwb_config;
        uwb_config.port = options.uwb_port;
        runtime.uwb = std::make_unique<UwbDriver>(uwb_config);
        runtime.uwb_ok = runtime.uwb->open(error);
        if (!runtime.uwb_ok) std::cerr << "[sa] UWB unavailable: " << error << "\n";
    }

    if (!options.novelda_port.empty())
    {
        NoveldaConfig novelda_config;
        novelda_config.port = options.novelda_port;
        runtime.novelda = std::make_unique<NoveldaDriver>(novelda_config);
        runtime.novelda_ok = runtime.novelda->open(error);
        if (!runtime.novelda_ok) std::cerr << "[sa] Novelda unavailable: " << error << "\n";
    }

    if (!options.ultrasonic_port.empty())
    {
        UltrasonicConfig ultrasonic_config;
        ultrasonic_config.port = options.ultrasonic_port;
        runtime.ultrasonic = std::make_unique<UltrasonicDriver>(ultrasonic_config);
        runtime.ultrasonic_ok = runtime.ultrasonic->open(error);
        if (!runtime.ultrasonic_ok) std::cerr << "[sa] ultrasonic unavailable: " << error << "\n";
    }

    if (!options.marker_port.empty())
    {
        MarkerConfig marker_config;
        marker_config.port = options.marker_port;
        runtime.marker = std::make_unique<MarkerDriver>(marker_config);
        runtime.marker_ok = runtime.marker->open(error);
        if (!runtime.marker_ok) std::cerr << "[sa] marker bridge unavailable: " << error << "\n";
    }

    return runtime.imu_ok || runtime.radar_ok || runtime.uwb_ok || runtime.novelda_ok || runtime.ultrasonic_ok ||
           runtime.marker_ok;
}

int runHardware(const Options& options, Telemetry& telemetry, Haptics& haptics, bool telemetry_ok,
                RuntimeContext& runtime)
{
    HardwareRuntime hardware;
    if (!openHardware(options, hardware))
    {
        std::cerr << "[sa] no hardware sensor could be opened; use --simulate or --replay\n";
        return 1;
    }

    FusionEngine fusion;
    if (!loadInitialSchematic(options, fusion, runtime)) return 1;
    Recorder recorder;
    bool recording = false;
    if (options.record) recording = openRecorder(recorder, options.record_path);
    recordActiveSchematic(recorder, recording, runtime);

    const auto period = std::chrono::microseconds(static_cast<int64_t>(1.0e6 / options.loop_rate_hz));
    auto next_tick = std::chrono::steady_clock::now();
    const uint64_t start_ms = monoNowMs();
    uint64_t frames = 0;
    uint32_t frame_id = 0;
    WorldModel last_world{};
    std::string error;

    std::cout << "[sa] hardware mode @" << options.loop_rate_hz << " Hz"
              << " sensors=" << sensorSelectionName(options.sensors)
              << " (IMU=" << (hardware.imu_ok ? "yes" : "no")
              << " radar=" << (hardware.radar_ok ? "yes" : "no")
              << " UWB=" << (hardware.uwb_ok ? "yes" : "no")
              << " novelda=" << (hardware.novelda_ok ? "yes" : "no")
              << " ultrasonic=" << (hardware.ultrasonic_ok ? "yes" : "no")
              << " marker=" << (hardware.marker_ok ? "yes" : "no") << ")\n";

    while (g_running.load())
    {
        next_tick += period;
        const uint64_t now_ms = monoNowMs();

        SensorFrame frame;
        frame.frame_id = ++frame_id;
        frame.timestamp_ms = now_ms;

        if (hardware.imu_ok)
        {
            ImuSample sample;
            if (hardware.imu->read(sample, error))
            {
                sample.timestamp_ms = now_ms;
                frame.imu = sample;
            }
        }

        if (hardware.radar_ok)
        {
            RadarFrame candidate;
            RadarFrame newest;
            bool has_radar = false;
            while (hardware.radar->poll(candidate))
            {
                newest = std::move(candidate);
                has_radar = true;
            }
            if (has_radar)
            {
                newest.timestamp_ms = now_ms;
                frame.radar = std::move(newest);
            }
        }

        if (hardware.uwb_ok) hardware.uwb->poll(frame.uwb);
        if (hardware.novelda_ok) hardware.novelda->poll(frame.novelda);
        if (hardware.ultrasonic_ok)
        {
            std::vector<UltrasonicMeasurement> samples;
            if (hardware.ultrasonic->poll(samples) && !samples.empty())
            {
                samples.back().timestamp_ms = now_ms;
                frame.ultrasonic = samples.back();
            }
        }
        if (hardware.marker_ok)
        {
            MarkerObservation observation;
            if (hardware.marker->poll(observation))
            {
                observation.timestamp_ms = now_ms;
                frame.marker = observation;
            }
        }

        if (applyMapUpload(telemetry, telemetry_ok, fusion, runtime))
            recordActiveSchematic(recorder, recording, runtime);
        const SensorFrame filtered = filterSensorFrame(frame, options.sensors);
        WorldModel world = fusion.process(filtered);
        last_world = world;

        if (recording && !recorder.recordFrame(frame, &world))
        {
            std::cerr << "[sa] recording stopped: " << recorder.error() << "\n";
            recording = false;
        }
        publishAll(world, filtered, telemetry, haptics, telemetry_ok, runtime);
        ++frames;
        logWorld(world, options.verbose && (frames % static_cast<uint64_t>(options.loop_rate_hz) == 0));

        if (options.duration_s > 0.0 && now_ms >= start_ms + static_cast<uint64_t>(options.duration_s * 1000.0))
            break;

        std::this_thread::sleep_until(next_tick);
        const auto actual = std::chrono::steady_clock::now();
        if (actual - next_tick > period * 4) next_tick = actual;
    }

    std::cout << "[sa] hardware stopped: frames=" << frames
              << " tracks=" << last_world.tracks.size()
              << " occupied=" << last_world.occupancy.occupiedCellCount() << "\n";
    if (recording)
    {
        std::cout << "[sa] recorded " << recorder.framesWritten() << " frames, " << recorder.bytesWritten()
                  << " bytes\n";
        recorder.close();
    }
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    std::string error;
    if (!parseOptions(argc, argv, options, error))
    {
        std::cerr << "[sa] " << error << "\n\n";
        printUsage(argv[0]);
        return 2;
    }
    if (options.show_help)
    {
        printUsage(argv[0]);
        return 0;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    Telemetry telemetry(telemetryConfig(options));
    const bool telemetry_ok = startTelemetry(options, telemetry);
    Haptics haptics(hapticConfig(options));
    startHaptics(haptics);

    const WristConfig wrist_config = wristConfig(options);
    RuntimeContext runtime(wrist_config, announceConfig(options));
    startWrist(runtime.vibrator, options);
    startAnnouncer(runtime.announcer, options);

    int result = 0;
    if (options.replay)
        result = runReplay(options, telemetry, haptics, telemetry_ok, runtime);
    else if (options.simulate)
        result = runSimulation(options, telemetry, haptics, telemetry_ok, runtime);
    else
        result = runHardware(options, telemetry, haptics, telemetry_ok, runtime);

    runtime.vibrator.stop();
    runtime.announcer.stop();
    haptics.stop();
    telemetry.stop();
    return result;
}
