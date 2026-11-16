#include "hub_server.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string_view>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
using HubSocket = SOCKET;
constexpr HubSocket kInvalidHubSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using HubSocket = int;
constexpr HubSocket kInvalidHubSocket = -1;
#endif

namespace sa {
namespace {

constexpr uint16_t kHubPort = 8765;
constexpr uint64_t kFrameIntervalMs = 50;
constexpr std::size_t kMaxCommandBytes = 128;
constexpr std::size_t kFrameBytes = 32768;

void closeHubSocket(HubSocket socket)
{
#if defined(_WIN32)
    closesocket(socket);
#else
    close(socket);
#endif
}

bool isWouldBlock()
{
#if defined(_WIN32)
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
}

bool setNonBlocking(HubSocket socket)
{
#if defined(_WIN32)
    u_long enabled = 1;
    return ioctlsocket(socket, FIONBIO, &enabled) == 0;
#else
    const int flags = fcntl(socket, F_GETFL, 0);
    return flags >= 0 && fcntl(socket, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

bool appendFormat(char* output, std::size_t capacity, std::size_t& used,
                  const char* format, ...)
{
    if (used >= capacity) return false;
    va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(output + used, capacity - used, format, args);
    va_end(args);
    if (written < 0 || static_cast<std::size_t>(written) >= capacity - used) return false;
    used += static_cast<std::size_t>(written);
    return true;
}

bool finiteFrame(const HudRenderFrame& frame)
{
    if (frame.wall_count > frame.walls.size() || frame.target_count > frame.targets.size() ||
        frame.presence_zone_count > frame.presence_zones.size() ||
        frame.breadcrumb_count > frame.breadcrumb_path.size() ||
        !std::isfinite(frame.pose.x) || !std::isfinite(frame.pose.y) ||
        !std::isfinite(frame.pose.yaw_deg) || !std::isfinite(frame.return_heading_deg))
        return false;

    for (std::size_t i = 0; i < frame.wall_count; ++i)
    {
        const auto& wall = frame.walls[i];
        if (!std::isfinite(wall.x1) || !std::isfinite(wall.y1) ||
            !std::isfinite(wall.x2) || !std::isfinite(wall.y2))
            return false;
    }
    for (std::size_t i = 0; i < frame.target_count; ++i)
    {
        const auto& target = frame.targets[i];
        const float predicted_x = target.x + target.vx;
        const float predicted_y = target.y + target.vy;
        if (!std::isfinite(target.x) || !std::isfinite(target.y) ||
            !std::isfinite(target.vx) || !std::isfinite(target.vy) ||
            !std::isfinite(predicted_x) || !std::isfinite(predicted_y) ||
            !std::isfinite(target.confidence) || target.confidence < 0.0f ||
            target.confidence > 1.0f ||
            target.source > HudTargetSource::Lidar)
            return false;
    }
    for (std::size_t i = 0; i < frame.presence_zone_count; ++i)
    {
        const auto& zone = frame.presence_zones[i];
        if (!std::isfinite(zone.range_min) || !std::isfinite(zone.range_max) ||
            !std::isfinite(zone.bearing_center_rad) || !std::isfinite(zone.bearing_width_rad) ||
            !std::isfinite(zone.confidence) || zone.range_min <= 0.05f ||
            zone.range_max <= zone.range_min || zone.confidence < 0.0f ||
            zone.confidence > 1.0f)
            return false;
    }
    for (std::size_t i = 0; i < frame.breadcrumb_count; ++i)
    {
        const auto& point = frame.breadcrumb_path[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    }
    return true;
}

const char* sourceName(HudTargetSource source)
{
    switch (source)
    {
    case HudTargetSource::Mmwave: return "mmwave";
    case HudTargetSource::Uwb: return "uwb";
    case HudTargetSource::Tof: return "tof";
    case HudTargetSource::Lidar: return "lidar";
    }
    return "mmwave";
}

bool appendJsonString(char* output, std::size_t capacity, std::size_t& used,
                      const char* text)
{
    if (used >= capacity) return false;
    output[used++] = '"';
    for (std::size_t i = 0; i < kHudAnnouncementBytes && text[i] != '\0'; ++i)
    {
        const unsigned char value = static_cast<unsigned char>(text[i]);
        if (value == '"' || value == '\\')
        {
            if (used + 2 >= capacity) return false;
            output[used++] = '\\';
            output[used++] = static_cast<char>(value);
        }
        else if (value < 0x20)
        {
            if (used + 6 >= capacity) return false;
            const int written = std::snprintf(output + used, capacity - used, "\\u%04x", value);
            if (written != 6) return false;
            used += 6;
        }
        else
        {
            if (used + 1 >= capacity) return false;
            output[used++] = static_cast<char>(value);
        }
    }
    if (used + 1 >= capacity) return false;
    output[used++] = '"';
    output[used] = '\0';
    return true;
}

} // namespace

HubServer::~HubServer()
{
    stop();
}

bool HubServer::start(std::string& error)
{
    error.clear();
    if (socket_ >= 0) return true;
#if defined(_WIN32)
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
    {
        error = "failed to initialize Winsock";
        return false;
    }
#endif
    const HubSocket socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket == kInvalidHubSocket)
    {
        error = "failed to create hub UDP socket";
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }
    if (!setNonBlocking(socket))
    {
        error = "failed to make hub UDP socket non-blocking";
        closeHubSocket(socket);
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(kHubPort);
    if (inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1 ||
        ::bind(socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0)
    {
        error = "failed to bind hub UDP socket to 127.0.0.1:8765";
        closeHubSocket(socket);
#if defined(_WIN32)
        WSACleanup();
#endif
        return false;
    }

    socket_ = static_cast<std::intptr_t>(socket);
    command_head_ = 0;
    command_count_ = 0;
    rejected_command_count_ = 0;
    peer_address_size_ = 0;
    has_published_ = false;
    return true;
}

void HubServer::stop()
{
    if (socket_ < 0) return;
    closeHubSocket(static_cast<HubSocket>(socket_));
    socket_ = -1;
    peer_address_size_ = 0;
#if defined(_WIN32)
    WSACleanup();
#endif
}

bool HubServer::parseCommand(const char* data, std::size_t size, HubCommand& command)
{
    if (data == nullptr || size == 0 || size > kMaxCommandBytes) return false;
    std::array<char, kMaxCommandBytes + 1> compact{};
    std::size_t used = 0;
    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = 0; i < size; ++i)
    {
        const unsigned char value = static_cast<unsigned char>(data[i]);
        if (value < 0x20 && std::isspace(value) == 0) return false;
        if (std::isspace(value) != 0 && !in_string) continue;
        if (used >= compact.size() - 1) return false;
        compact[used++] = static_cast<char>(value);
        if (in_string)
        {
            if (escaped) escaped = false;
            else if (value == '\\') escaped = true;
            else if (value == '"') in_string = false;
        }
        else if (value == '"')
        {
            in_string = true;
        }
    }
    if (in_string || escaped) return false;
    compact[used] = '\0';
    const std::string_view text(compact.data(), used);
    if (text == "{\"cmd\":\"trail_purge\"}")
    {
        command = {HubCommandType::TrailPurge, false};
        return true;
    }
    if (text == "{\"cmd\":\"trail_toggle\",\"enabled\":true}")
    {
        command = {HubCommandType::TrailToggle, true};
        return true;
    }
    if (text == "{\"cmd\":\"trail_toggle\",\"enabled\":false}")
    {
        command = {HubCommandType::TrailToggle, false};
        return true;
    }
    return false;
}

bool HubServer::service(uint64_t timestamp_ms, const HudRenderFrame& frame, std::string& error)
{
    error.clear();
    if (socket_ < 0)
    {
        error = "hub server is not started";
        return false;
    }

    const HubSocket socket = static_cast<HubSocket>(socket_);
    for (std::size_t received_count = 0; received_count < 8; ++received_count)
    {
        std::array<char, kMaxCommandBytes + 1> data{};
        sockaddr_in peer{};
#if defined(_WIN32)
        int peer_size = sizeof(peer);
#else
        socklen_t peer_size = sizeof(peer);
#endif
        const int received = static_cast<int>(recvfrom(
            socket, data.data(), static_cast<int>(data.size()), 0,
            reinterpret_cast<sockaddr*>(&peer), &peer_size));
        if (received < 0)
        {
            if (isWouldBlock()) break;
            error = "failed to receive hub UDP command";
            return false;
        }

        HubCommand command{};
        if (!parseCommand(data.data(), static_cast<std::size_t>(received), command))
        {
            ++rejected_command_count_;
            continue;
        }
        if (command_count_ == commands_.size())
        {
            error = "hub command queue is full";
            return false;
        }
        const std::size_t tail = (command_head_ + command_count_) % commands_.size();
        commands_[tail] = command;
        ++command_count_;
        std::memcpy(peer_address_.data(), &peer, sizeof(peer));
        peer_address_size_ = sizeof(peer);
    }

    if (peer_address_size_ != 0 &&
        (!has_published_ || timestamp_ms < last_publish_ms_ ||
         timestamp_ms - last_publish_ms_ >= kFrameIntervalMs))
    {
        if (!publishFrame(timestamp_ms, frame, error)) return false;
        last_publish_ms_ = timestamp_ms;
        has_published_ = true;
    }
    return true;
}

bool HubServer::takeCommand(HubCommand& command)
{
    if (command_count_ == 0) return false;
    command = commands_[command_head_];
    command_head_ = (command_head_ + 1) % commands_.size();
    --command_count_;
    return true;
}

bool HubServer::serializeFrame(const HudRenderFrame& frame,
                               char* output,
                               std::size_t capacity,
                               std::size_t& written,
                               std::string& error)
{
    error.clear();
    written = 0;
    if (output == nullptr || capacity == 0)
    {
        error = "HUD output buffer is invalid";
        return false;
    }
    if (!finiteFrame(frame))
    {
        error = "HUD frame contains invalid fields";
        return false;
    }

    std::size_t used = 0;
    const std::size_t breadcrumb_count = frame.trail_enabled ? frame.breadcrumb_count : 0;
    if (!appendFormat(output, capacity, used,
                      "{\"background\":\"#000000\",\"foreground\":\"#FF0000\","
                      "\"declutter\":%s,"
                      "\"pose\":{\"x\":%.3f,\"y\":%.3f,\"heading_deg\":%.3f,\"floor\":%.0f},"
                      "\"walls\":[",
                      frame.declutter ? "true" : "false",
                      frame.pose.x, frame.pose.y, frame.pose.yaw_deg,
                      static_cast<double>(frame.floor)))
        goto overflow;
    for (std::size_t i = 0; i < frame.wall_count; ++i)
    {
        const auto& wall = frame.walls[i];
        if (!appendFormat(output, capacity, used, "%s[%.3f,%.3f,%.3f,%.3f]",
                          i == 0 ? "" : ",", wall.x1, wall.y1, wall.x2, wall.y2))
            goto overflow;
    }
    if (!appendFormat(output, capacity, used, "],\"targets\":["))
        goto overflow;
    for (std::size_t i = 0; i < frame.target_count; ++i)
    {
        const auto& target = frame.targets[i];
        const float predicted_x = target.x + target.vx;
        const float predicted_y = target.y + target.vy;
        if (used >= capacity) goto overflow;
        const int json_bytes = std::snprintf(
            output + used, capacity - used,
            "%s{\"x\":%.3f,\"y\":%.3f,\"vx\":%.3f,\"vy\":%.3f,"
            "\"prediction_1s\":[%.3f,%.3f],\"conf\":%.3f,\"source\":\"%s\"}",
            i == 0 ? "" : ",", target.x, target.y, target.vx, target.vy,
            predicted_x, predicted_y,
            target.confidence, sourceName(target.source));
        if (json_bytes < 0 || static_cast<std::size_t>(json_bytes) >= capacity - used)
            goto overflow;
        used += static_cast<std::size_t>(json_bytes);
    }
    if (!appendFormat(output, capacity, used, "],\"presence_zones\":["))
        goto overflow;
    for (std::size_t i = 0; i < frame.presence_zone_count; ++i)
    {
        const auto& zone = frame.presence_zones[i];
        if (!appendFormat(output, capacity, used,
                          "%s{\"range_min_m\":%.3f,\"range_max_m\":%.3f,"
                          "\"bearing_center_rad\":%.4f,\"bearing_width_rad\":%.4f,\"conf\":%.3f}",
                          i == 0 ? "" : ",", zone.range_min, zone.range_max,
                          zone.bearing_center_rad, zone.bearing_width_rad, zone.confidence))
            goto overflow;
    }
    if (!appendFormat(output, capacity, used,
                      "],\"breadcrumb_path\":["))
        goto overflow;
    for (std::size_t i = 0; i < breadcrumb_count; ++i)
    {
        const auto& point = frame.breadcrumb_path[i];
        if (!appendFormat(output, capacity, used, "%s[%.3f,%.3f]",
                          i == 0 ? "" : ",", point.x, point.y))
            goto overflow;
    }
    if (!appendFormat(output, capacity, used,
                      "],\"return_heading_deg\":%.3f,\"trail_enabled\":%s,"
                      "\"status\":{\"emcon_mode\":\"%s\",\"tts_alert\":",
                      frame.return_heading_deg, frame.trail_enabled ? "true" : "false",
                      frame.auto_mode ? "AUTO" : "MANUAL"))
        goto overflow;
    if (!appendJsonString(output, capacity, used, frame.announcement.data()))
        goto overflow;
    if (!appendFormat(output, capacity, used, "}}"))
        goto overflow;
    written = used;
    return true;

overflow:
    error = "HUD frame exceeds the fixed output buffer";
    return false;
}

bool HubServer::publishFrame(uint64_t timestamp_ms,
                             const HudRenderFrame& frame,
                             std::string& error)
{
    (void)timestamp_ms;
    std::array<char, kFrameBytes> output{};
    std::size_t written = 0;
    if (!serializeFrame(frame, output.data(), output.size(), written, error)) return false;

    {
        const HubSocket socket = static_cast<HubSocket>(socket_);
        const sockaddr* peer = reinterpret_cast<const sockaddr*>(peer_address_.data());
#if defined(_WIN32)
        const int sent = sendto(socket, output.data(), static_cast<int>(written), 0,
                                peer, static_cast<int>(peer_address_size_));
#else
        const ssize_t sent = sendto(socket, output.data(), written, 0,
                                    peer, static_cast<socklen_t>(peer_address_size_));
#endif
        if (sent < 0 || static_cast<std::size_t>(sent) != written)
        {
            error = "failed to send HUD frame";
            return false;
        }
    }
    return true;
}

} // namespace sa
