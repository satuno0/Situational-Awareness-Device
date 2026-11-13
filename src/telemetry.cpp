#include "telemetry.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sa {
namespace {

#if defined(_WIN32)
using Socket = SOCKET;
constexpr Socket kInvalidSocket = INVALID_SOCKET;
#else
using Socket = int;
constexpr Socket kInvalidSocket = -1;
#endif

constexpr const char* kFallbackHtml =
    "<!doctype html><html><body style='background:#101418;color:#eee;font-family:sans-serif'>"
    "<h1>SA device telemetry</h1><p>The web/index.html visualization was not found. "
    "JSON is available at <code>/api/state</code> and <code>/api/grid</code>.</p></body></html>";

std::string number(float value)
{
    if (!std::isfinite(value)) return "0";
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << value;
    std::string out = stream.str();
    while (out.size() > 1 && out.back() == '0') out.pop_back();
    if (!out.empty() && out.back() == '.') out.pop_back();
    return out;
}

std::string lowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::string trimCopy(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::string jsonString(const std::string& text)
{
    std::string out;
    out.reserve(text.size() + 2);
    out.push_back('"');
    for (unsigned char c : text)
    {
        switch (c)
        {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20)
            {
                char buffer[8];
                std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
                out += buffer;
            }
            else
            {
                out.push_back(static_cast<char>(c));
            }
        }
    }
    out.push_back('"');
    return out;
}

constexpr std::size_t kMaxMapUploadBytes = 512u * 1024u;

std::string base64(const std::vector<uint8_t>& data)
{
    static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((data.size() + 2) / 3 * 4);
    std::size_t i = 0;
    while (i + 2 < data.size())
    {
        const uint32_t v = (static_cast<uint32_t>(data[i]) << 16) |
                           (static_cast<uint32_t>(data[i + 1]) << 8) |
                           static_cast<uint32_t>(data[i + 2]);
        out.push_back(alphabet[(v >> 18) & 63]);
        out.push_back(alphabet[(v >> 12) & 63]);
        out.push_back(alphabet[(v >> 6) & 63]);
        out.push_back(alphabet[v & 63]);
        i += 3;
    }
    if (i < data.size())
    {
        uint32_t v = static_cast<uint32_t>(data[i]) << 16;
        if (i + 1 < data.size()) v |= static_cast<uint32_t>(data[i + 1]) << 8;
        out.push_back(alphabet[(v >> 18) & 63]);
        out.push_back(alphabet[(v >> 12) & 63]);
        out.push_back(i + 1 < data.size() ? alphabet[(v >> 6) & 63] : '=');
        out.push_back('=');
    }
    return out;
}

void closeSocket(Socket socket)
{
    if (socket == kInvalidSocket) return;
#if defined(_WIN32)
    closesocket(socket);
#else
    ::close(socket);
#endif
}

} // namespace

Telemetry::Telemetry() : Telemetry(TelemetryConfig{}) {}
Telemetry::Telemetry(const TelemetryConfig& config) : config_(config) {}
Telemetry::~Telemetry() { stop(); }

bool Telemetry::loadIndex(std::string& error)
{
    const std::string path = config_.web_root + "/index.html";
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        index_html_ = kFallbackHtml;
        error = "web UI not found at " + path + "; serving fallback";
        return true;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    index_html_ = buffer.str();
    return true;
}

bool Telemetry::start(std::string& error)
{
    stop();
    if (!config_.enabled)
    {
        error = "telemetry disabled";
        return false;
    }
    loadIndex(error);

#if defined(_WIN32)
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        error = "WSAStartup failed";
        return false;
    }
#endif

    Socket socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket == kInvalidSocket)
    {
        error = "failed to create telemetry socket";
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }

    int yes = 1;
    setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config_.port);
    if (inet_pton(AF_INET, config_.bind_address.c_str(), &address.sin_addr) != 1)
    {
        error = "invalid telemetry bind address: " + config_.bind_address;
        closeSocket(socket);
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }

    if (bind(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    {
        error = "failed to bind telemetry to " + config_.bind_address + ":" + std::to_string(config_.port);
        closeSocket(socket);
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }
    if (listen(socket, 8) != 0)
    {
        error = "failed to listen on telemetry socket";
        closeSocket(socket);
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }

    sockaddr_in actual{};
#if defined(_WIN32)
    int length = sizeof(actual);
#else
    socklen_t length = sizeof(actual);
#endif
    if (getsockname(socket, reinterpret_cast<sockaddr*>(&actual), &length) == 0)
        port_ = ntohs(actual.sin_port);

    listen_socket_ = static_cast<intptr_t>(socket);
    running_ = true;
    thread_ = std::thread(&Telemetry::serverLoop, this);
    return true;
}

void Telemetry::stop()
{
    running_ = false;
    if (listen_socket_ != -1)
    {
        closeSocket(static_cast<Socket>(listen_socket_));
        listen_socket_ = -1;
    }
    if (thread_.joinable()) thread_.join();
#if defined(_WIN32)
    WSACleanup();
#endif
}

void Telemetry::serverLoop()
{
    while (running_.load())
    {
        const Socket listener = static_cast<Socket>(listen_socket_);
        if (listener == kInvalidSocket) break;

        fd_set set;
        FD_ZERO(&set);
        FD_SET(listener, &set);
        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 250000;
#if defined(_WIN32)
        const int ready = select(0, &set, nullptr, nullptr, &timeout);
#else
        const int ready = select(listener + 1, &set, nullptr, nullptr, &timeout);
#endif
        if (!running_.load()) break;
        if (ready <= 0) continue;

        sockaddr_in client_address{};
#if defined(_WIN32)
        int client_length = sizeof(client_address);
#else
        socklen_t client_length = sizeof(client_address);
#endif
        const Socket client = accept(listener, reinterpret_cast<sockaddr*>(&client_address), &client_length);
        if (client == kInvalidSocket) continue;
        handleClient(static_cast<intptr_t>(client));
    }
}

void Telemetry::sendAll(intptr_t raw_socket, const std::string& data) const
{
    const Socket socket = static_cast<Socket>(raw_socket);
    std::size_t offset = 0;
    while (offset < data.size())
    {
#if defined(_WIN32)
        const int sent = send(socket, data.data() + offset, static_cast<int>(data.size() - offset), 0);
#else
        const ssize_t sent = send(socket, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
#endif
        if (sent <= 0) break;
        offset += static_cast<std::size_t>(sent);
    }
}

std::string Telemetry::response(const std::string& status,
                                const std::string& content_type,
                                const std::string& body,
                                bool head_only) const
{
    std::ostringstream out;
    out << "HTTP/1.1 " << status << "\r\n";
    out << "Content-Type: " << content_type << "\r\n";
    out << "Content-Length: " << body.size() << "\r\n";
    out << "Cache-Control: no-store\r\n";
    out << "X-Content-Type-Options: nosniff\r\n";
    out << "Connection: close\r\n\r\n";
    if (!head_only) out << body;
    return out.str();
}

void Telemetry::handleClient(intptr_t raw_socket)
{
    const Socket socket = static_cast<Socket>(raw_socket);
#if defined(_WIN32)
    DWORD timeout = 1500;
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 500000;
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif

    std::string request;
    char buffer[2048];
    while (request.find("\r\n\r\n") == std::string::npos && request.size() < 16384)
    {
        const int got = recv(socket, buffer, sizeof(buffer), 0);
        if (got <= 0) break;
        request.append(buffer, static_cast<std::size_t>(got));
    }

    const std::size_t line_end = request.find("\r\n");
    std::string line = line_end == std::string::npos ? request : request.substr(0, line_end);
    std::istringstream request_line(line);
    std::string method;
    std::string target;
    std::string version;
    request_line >> method >> target >> version;

    std::string path = target;
    const std::size_t query = path.find('?');
    if (query != std::string::npos) path.resize(query);
    const bool head_only = method == "HEAD";

    const std::size_t header_end = request.find("\r\n\r\n");
    if (method == "POST")
    {
        if (path != "/api/map")
        {
            const std::string reply =
                response("404 Not Found", "text/plain", "only POST /api/map is supported\n", false);
            sendAll(raw_socket, reply);
            closeSocket(socket);
            return;
        }
        if (header_end == std::string::npos)
        {
            const std::string reply = response("400 Bad Request", "text/plain", "missing headers\n", false);
            sendAll(raw_socket, reply);
            closeSocket(socket);
            return;
        }

        bool have_length = false;
        std::size_t content_length = 0;
        std::size_t position = 0;
        while (position < header_end)
        {
            const std::size_t end = request.find("\r\n", position);
            if (end == std::string::npos || end > header_end) break;
            const std::string header = request.substr(position, end - position);
            const std::size_t colon = header.find(':');
            if (colon != std::string::npos && lowerCopy(trimCopy(header.substr(0, colon))) == "content-length")
            {
                const std::string value = trimCopy(header.substr(colon + 1));
                char* parse_end = nullptr;
                const unsigned long parsed = std::strtoul(value.c_str(), &parse_end, 10);
                if (parse_end != value.c_str() && *parse_end == '\0')
                {
                    content_length = static_cast<std::size_t>(parsed);
                    have_length = true;
                }
            }
            position = end + 2;
        }

        if (!have_length || content_length == 0 || content_length > kMaxMapUploadBytes)
        {
            const std::string reply =
                response("400 Bad Request", "text/plain", "invalid or oversized map body\n", false);
            sendAll(raw_socket, reply);
            closeSocket(socket);
            return;
        }

        std::string body = request.substr(header_end + 4);
        while (body.size() < content_length)
        {
            const int got = recv(socket, buffer, sizeof(buffer), 0);
            if (got <= 0) break;
            body.append(buffer, static_cast<std::size_t>(got));
        }
        if (body.size() < content_length)
        {
            const std::string reply = response("400 Bad Request", "text/plain", "truncated map body\n", false);
            sendAll(raw_socket, reply);
            closeSocket(socket);
            return;
        }
        body.resize(content_length);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            pending_map_text_ = std::move(body);
            has_map_upload_ = true;
        }
        const std::string reply =
            response("202 Accepted", "text/plain", "schematic queued for validation\n", false);
        sendAll(raw_socket, reply);
        closeSocket(socket);
        return;
    }

    if (method != "GET" && method != "HEAD")
    {
        const std::string reply = response("405 Method Not Allowed", "text/plain", "read-only endpoint\n", head_only);
        sendAll(raw_socket, reply);
        closeSocket(socket);
        return;
    }

    std::string reply;
    if (path == "/api/state")
    {
        std::string body;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            body = state_json_;
        }
        reply = response("200 OK", "application/json; charset=utf-8", body, head_only);
    }
    else if (path == "/api/grid")
    {
        std::string body;
        bool available = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            body = grid_json_;
            available = has_grid_;
        }
        reply = available ? response("200 OK", "application/json; charset=utf-8", body, head_only)
                          : response("503 Service Unavailable", "application/json", "{\"error\":\"grid not ready\"}", head_only);
    }
    else if (path == "/api/health")
    {
        reply = response("200 OK", "application/json", "{\"ok\":true}", head_only);
    }
    else if (path == "/" || path == "/index.html")
    {
        reply = response("200 OK", "text/html; charset=utf-8", index_html_, head_only);
    }
    else if (path == "/favicon.ico")
    {
        reply = response("204 No Content", "text/plain", "", head_only);
    }
    else
    {
        reply = response("404 Not Found", "text/plain", "not found\n", head_only);
    }

    sendAll(raw_socket, reply);
    closeSocket(socket);
}

void Telemetry::buildStateJson(const WorldModel& world,
                               const std::optional<MarkerObservation>& marker,
                               const WristTelemetry& wrist,
                               const std::string& announcement)
{
    std::ostringstream out;
    out << "{";
    out << "\"timestamp\":" << world.timestamp_ms << ",";
    out << "\"sensor_mode\":" << jsonString(config_.sensor_mode) << ",";
    out << "\"pose\":{\"x\":" << number(world.pose.x) << ",\"y\":" << number(world.pose.y)
        << ",\"yaw_deg\":" << number(world.pose.yaw_deg) << "},";
    out << "\"environment_confidence\":" << number(world.environment_confidence) << ",";
    out << "\"sensor_health\":{"
        << "\"imu\":" << (world.health.imu ? "true" : "false")
        << ",\"radar\":" << (world.health.radar ? "true" : "false")
        << ",\"uwb\":" << (world.health.uwb ? "true" : "false")
        << ",\"novelda\":" << (world.health.novelda ? "true" : "false")
        << ",\"ultrasonic\":" << (world.health.ultrasonic ? "true" : "false")
        << ",\"score\":" << number(world.health.health_score) << "},";
    out << "\"occupancy\":{\"occupied\":" << world.occupancy.occupiedCellCount()
        << ",\"known\":" << world.occupancy.knownCellCount()
        << ",\"total\":" << world.occupancy.totalCellCount() << "},";

    out << "\"tracks\":[";
    for (std::size_t i = 0; i < world.tracks.size(); ++i)
    {
        const auto& track = world.tracks[i];
        if (i) out << ",";
        const Vec3 pos = track.filter.position();
        const Vec3 vel = track.filter.velocity();
        out << "{\"id\":" << track.id
            << ",\"state\":\"" << trackStateName(track.state) << "\""
            << ",\"classification\":\"" << personClassName(classifyTrack(track)) << "\""
            << ",\"position\":{\"x\":" << number(pos.x) << ",\"y\":" << number(pos.y)
            << ",\"z\":" << number(pos.z) << "}"
            << ",\"velocity\":{\"x\":" << number(vel.x) << ",\"y\":" << number(vel.y)
            << ",\"z\":" << number(vel.z) << "}"
            << ",\"position_sigma_m\":" << number(track.position_sigma_m)
            << ",\"existence_confidence\":" << number(track.existence_confidence)
            << ",\"position_confidence\":" << number(track.position_confidence)
            << ",\"visible\":" << (track.visible ? "true" : "false")
            << ",\"identity\":{\"tagged_safe\":" << (track.identity.tagged_safe ? "true" : "false")
            << ",\"tag_id\":" << track.identity.tag_id
            << ",\"confidence\":" << number(track.identity.confidence) << "}}";
    }
    out << "],";

    out << "\"presence_zones\":[";
    for (std::size_t i = 0; i < world.zones.size(); ++i)
    {
        const auto& zone = world.zones[i];
        if (i) out << ",";
        out << "{\"range_min\":" << number(zone.range_min)
            << ",\"range_max\":" << number(zone.range_max)
            << ",\"bearing_center_rad\":" << number(zone.bearing_center_rad)
            << ",\"bearing_width_rad\":" << number(zone.bearing_width_rad)
            << ",\"confidence\":" << number(zone.confidence) << "}";
    }
    out << "],";

    out << "\"map\":[";
    for (std::size_t i = 0; i < world.static_objects.size(); ++i)
    {
        const auto& object = world.static_objects[i];
        if (i) out << ",";
        out << "{\"position\":{\"x\":" << number(object.position.x) << ",\"y\":" << number(object.position.y)
            << ",\"z\":" << number(object.position.z) << "},\"confidence\":" << number(object.confidence) << "}";
    }
    out << "],";

    out << "\"walls\":[";
    for (std::size_t i = 0; i < world.walls.size(); ++i)
    {
        const auto& wall = world.walls[i];
        if (i) out << ",";
        out << "[" << number(wall.start.x) << "," << number(wall.start.y) << ","
            << number(wall.end.x) << "," << number(wall.end.y) << "]";
    }
    out << "],";

    out << "\"rooms\":[";
    for (std::size_t i = 0; i < world.rooms.size(); ++i)
    {
        const auto& room = world.rooms[i];
        if (i) out << ",";
        out << "{\"name\":" << jsonString(room.name)
            << ",\"min_x\":" << number(room.min_x)
            << ",\"min_y\":" << number(room.min_y)
            << ",\"max_x\":" << number(room.max_x)
            << ",\"max_y\":" << number(room.max_y) << "}";
    }
    out << "],";

    const bool marker_detected = marker && marker->valid;
    const uint64_t marker_age_ms =
        marker_detected && world.timestamp_ms >= marker->timestamp_ms
            ? world.timestamp_ms - marker->timestamp_ms
            : 0;
    out << "\"marker\":{\"detected\":" << (marker_detected ? "true" : "false")
        << ",\"bearing_deg\":" << number(marker_detected ? radToDeg(wrapRad(marker->bearing_rad)) : 0.0f)
        << ",\"confidence\":" << number(marker_detected ? marker->confidence : 0.0f)
        << ",\"area_ratio\":" << number(marker_detected ? marker->area_ratio : 0.0f)
        << ",\"age_ms\":" << marker_age_ms << "},";
    out << "\"wrist\":{\"enabled\":" << (wrist.enabled ? "true" : "false")
        << ",\"aimed\":" << (wrist.aimed ? "true" : "false")
        << ",\"vibrating\":" << (wrist.vibrating ? "true" : "false")
        << ",\"bearing_deg\":" << number(wrist.bearing_deg)
        << ",\"confidence\":" << number(wrist.confidence)
        << ",\"engage_deg\":" << number(wrist.engage_deg)
        << ",\"release_deg\":" << number(wrist.release_deg) << "},";
    out << "\"announcement\":" << jsonString(announcement);

    out << "}";

    std::lock_guard<std::mutex> lock(data_mutex_);
    state_json_ = out.str();
}

void Telemetry::buildGridJson(const WorldModel& world)
{
    const uint64_t now = world.timestamp_ms;
    if (last_grid_timestamp_ms_ && now >= *last_grid_timestamp_ms_ && now - *last_grid_timestamp_ms_ < 500)
        return;
    last_grid_timestamp_ms_ = now;

    const auto& config = world.occupancy.config();
    const auto& cells = world.occupancy.cells();
    std::vector<uint8_t> bytes;
    bytes.reserve(cells.size());
    for (const auto& cell : cells)
    {
        const float p = clampf(cell.occupancy_prob, 0.0f, 1.0f);
        bytes.push_back(static_cast<uint8_t>(std::lround(p * 255.0f)));
    }

    std::ostringstream out;
    out << "{\"width\":" << config.width << ",\"height\":" << config.height;
    out << ",\"cell_size\":" << number(config.cell_size);
    out << ",\"origin_x\":" << number(-0.5f * config.width * config.cell_size);
    out << ",\"origin_y\":" << number(-0.5f * config.height * config.cell_size);
    out << ",\"data\":\"" << base64(bytes) << "\"}";

    std::lock_guard<std::mutex> lock(data_mutex_);
    grid_json_ = out.str();
    has_grid_ = true;
}

void Telemetry::publish(const WorldModel& world,
                        const std::optional<MarkerObservation>& marker,
                        const WristTelemetry& wrist,
                        const std::string& announcement)
{
    buildStateJson(world, marker, wrist, announcement);
    buildGridJson(world);
}

bool Telemetry::takeMapUpload(std::string& text)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_map_upload_) return false;
    text.swap(pending_map_text_);
    pending_map_text_.clear();
    has_map_upload_ = false;
    return true;
}

} // namespace sa
