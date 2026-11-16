#pragma once

#include "protocol_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace sa {

enum class HubCommandType : uint8_t
{
    TrailToggle,
    TrailPurge
};

struct HubCommand
{
    HubCommandType type = HubCommandType::TrailPurge;
    bool enabled = false;
};

class HubServer
{
public:
    HubServer() = default;
    ~HubServer();

    HubServer(const HubServer&) = delete;
    HubServer& operator=(const HubServer&) = delete;

    bool start(std::string& error);
    void stop();
    bool service(uint64_t timestamp_ms, const HudRenderFrame& frame, std::string& error);
    bool takeCommand(HubCommand& command);
    uint64_t rejectedCommandCount() const { return rejected_command_count_; }

    static bool parseCommand(const char* data, std::size_t size, HubCommand& command);
    static bool serializeFrame(const HudRenderFrame& frame,
                               char* output,
                               std::size_t capacity,
                               std::size_t& written,
                               std::string& error);

private:
    bool publishFrame(uint64_t timestamp_ms, const HudRenderFrame& frame, std::string& error);

    std::intptr_t socket_ = -1;
    std::array<HubCommand, 8> commands_{};
    std::size_t command_head_ = 0;
    std::size_t command_count_ = 0;
    uint64_t rejected_command_count_ = 0;
    uint64_t last_publish_ms_ = 0;
    bool has_published_ = false;
    std::array<uint64_t, 16> peer_address_{};
    std::size_t peer_address_size_ = 0;
};

} // namespace sa
