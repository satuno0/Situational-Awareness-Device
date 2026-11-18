#pragma once

#include <stdint.h>

struct __attribute__((packed)) GlovePosePacket
{
    uint8_t magic;
    uint8_t seq;
    int16_t yaw_cdeg;
    int16_t pitch_cdeg;
    uint8_t btn_mask;
    uint8_t flags;
    uint8_t reserved;
    uint8_t checksum;
};

static_assert(sizeof(GlovePosePacket) == 10, "GlovePosePacket must be 10 bytes");

constexpr uint8_t GLOVE_PACKET_MAGIC = 0x5B;
constexpr uint8_t GLOVE_FLAG_ZERO_HEADING = 1u << 0;
constexpr uint8_t GLOVE_EVENT_RADAR_PING = 1u << 0;
constexpr uint8_t GLOVE_EVENT_QUICK_GLANCE = 1u << 1;
constexpr uint8_t GLOVE_EVENT_TRAIL_TOGGLE = 1u << 2;
constexpr uint8_t GLOVE_EVENT_PANIC_WIPE = 1u << 3;
