#pragma once

#include "types.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace sa {

struct TelemetryConfig
{
    std::string bind_address = "127.0.0.1";
    uint16_t port = 8765;
    std::string web_root = "web";
    bool enabled = true;
    std::string sensor_mode = "mixed";
};

// Flattened wrist-guidance state for the JSON feed. Keeping this separate from
// WristGuidance avoids leaking actuator details into the telemetry layer.
struct WristTelemetry
{
    bool enabled = false;
    bool aimed = false;
    bool vibrating = false;
    float bearing_deg = 0.0f;
    float confidence = 0.0f;
    float engage_deg = 0.0f;
    float release_deg = 0.0f;
};

class Telemetry
{
public:
    Telemetry();
    explicit Telemetry(const TelemetryConfig& config);
    ~Telemetry();

    bool start(std::string& error);
    void stop();
    bool running() const { return running_.load(); }
    uint16_t port() const { return port_; }

    void publish(const WorldModel& world,
                 const std::optional<MarkerObservation>& marker = std::nullopt,
                 const WristTelemetry& wrist = WristTelemetry{},
                 const std::string& announcement = {},
                 bool scan_available = false,
                 bool scan_active = false,
                 const std::string& scan_mode = "on_demand");

    // Takes one pending POST /api/map body. Returns false when none is queued.
    // The caller parses and validates it before replacing the active schematic.
    bool takeMapUpload(std::string& text);
    bool takeScanBurstCommand();
    bool takeLidarModeCommand(std::string& mode);

private:
    void serverLoop();
    void handleClient(intptr_t client);
    void buildStateJson(const WorldModel& world,
                        const std::optional<MarkerObservation>& marker,
                        const WristTelemetry& wrist,
                        const std::string& announcement,
                        bool scan_available,
                        bool scan_active,
                        const std::string& scan_mode);
    void buildGridJson(const WorldModel& world);
    std::string response(const std::string& status,
                         const std::string& content_type,
                         const std::string& body,
                         bool head_only) const;
    bool loadIndex(std::string& error);
    void sendAll(intptr_t socket, const std::string& data) const;

    TelemetryConfig config_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    intptr_t listen_socket_ = -1;
    uint16_t port_ = 0;

    mutable std::mutex data_mutex_;
    std::string state_json_ = "{}";
    std::string grid_json_;
    std::string index_html_;
    bool has_grid_ = false;
    std::optional<uint64_t> last_grid_timestamp_ms_;
    std::string pending_map_text_;
    bool has_map_upload_ = false;
    bool scan_burst_command_pending_ = false;
    std::optional<std::string> lidar_mode_command_pending_;
};

} // namespace sa
