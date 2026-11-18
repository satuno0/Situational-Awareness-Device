#include "breadcrumb_navigator.hpp"
#include "emcon_controller.hpp"
#include "glove_raycaster.hpp"
#include "hub_server.hpp"

#include <cmath>
#include <array>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

namespace {

int failures = 0;

void check(bool condition, const char* message)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

void testEmconController()
{
    sa::EmconController controller;
    check(controller.mode() == sa::EmconMode::Manual && !controller.radarEnabled(),
          "EMCON starts in manual mode with radar gated off");
    check(controller.setMode(sa::EmconMode::Auto) && controller.radarEnabled(),
          "auto mode enables radar processing");
    check(!controller.setMode(sa::EmconMode::Auto),
          "setting the current mode does not report a transition");
    controller.snoozeAlerts(1000);
    check(controller.alertsSnoozed(1000 + 299999),
          "alert snooze remains active for five minutes");
    check(!controller.alertsSnoozed(1000 + 300000),
          "alert snooze expires at its deterministic deadline");
}

void testBreadcrumbNavigator()
{
    sa::BreadcrumbNavigator navigator;
    check(navigator.enabled() && navigator.breadcrumbs().empty(),
          "breadcrumb recording defaults on and waits for its first valid pose");
    check(navigator.update({0.0f, 0.0f, 0.0f}, 10),
          "default-on trail records its initial pose");
    check(navigator.update({0.0f, 1.1f, 0.0f}, 20),
          "valid pose update records waypoints");
    check(navigator.breadcrumbs().size() == 3,
          "trail records the origin and interpolated half-meter waypoints");
    check(std::fabs(navigator.breadcrumbs()[1].y - 0.5f) < 1.0e-5f &&
              std::fabs(navigator.breadcrumbs()[2].y - 1.0f) < 1.0e-5f,
          "waypoints follow the configured spacing");
    check(navigator.returnWaypoint(0) != nullptr &&
              std::fabs(navigator.returnWaypoint(0)->y - 1.0f) < 1.0e-5f,
          "return waypoints are traversed in reverse order");
    check(std::fabs(navigator.returnHeadingDeg({0.0f, 1.1f, 0.0f}) - 180.0f) < 1.0e-3f,
          "return heading points toward the breadcrumb origin");
    check(!navigator.update({0.0f, 1.2f, 0.0f}, 19),
          "stale timestamps are rejected");
    check(navigator.setEnabled(false, {0.0f, 1.1f, 0.0f}, 30) &&
              !navigator.enabled() && navigator.breadcrumbs().size() == 3,
          "turning recording off pauses writes while preserving the existing path");
    check(navigator.update({3.0f, 3.0f, 0.0f}, 35) &&
              navigator.breadcrumbs().size() == 3,
          "paused trail ignores pose updates without deleting stored points");
    check(navigator.setEnabled(true, {1.0f, 2.0f, 0.0f}, 40) &&
              navigator.breadcrumbs().size() == 3,
          "trail can resume without clearing its previous points");
    check(navigator.update({1.0f, 2.6f, 0.0f}, 50) &&
              navigator.breadcrumbs().size() == 4,
          "resumed trail records from the re-enable pose, not the paused interval");
    navigator.purge();
    check(!navigator.enabled() && navigator.returnWaypointCount() == 0,
          "purge disables recording and securely removes all waypoints");
}

void testHubCommandParser()
{
    sa::HubCommand command{};
    const std::string enable = " { \"cmd\" : \"trail_toggle\", \"enabled\" : true } ";
    check(sa::HubServer::parseCommand(enable.data(), enable.size(), command) &&
              command.type == sa::HubCommandType::TrailToggle && command.enabled,
          "hub accepts a valid trail-enable command");
    const std::string purge = "{\"cmd\":\"trail_purge\"}";
    check(sa::HubServer::parseCommand(purge.data(), purge.size(), command) &&
              command.type == sa::HubCommandType::TrailPurge,
          "hub accepts a trail-purge command");
    const std::string panic = "{\"cmd\":\"panic_wipe\"}";
    check(sa::HubServer::parseCommand(panic.data(), panic.size(), command) &&
              command.type == sa::HubCommandType::PanicWipe,
          "hub accepts a panic-wipe command");
    const std::string csi = "{\"cmd\":\"csi_update\",\"zone_id\":2}";
    check(!sa::HubServer::parseCommand(csi.data(), csi.size(), command),
          "hub rejects unsupported CSI zone commands");
    const std::string alert = "{\"cmd\":\"external_alert\",\"severity\":3}";
    check(!sa::HubServer::parseCommand(alert.data(), alert.size(), command),
          "hub rejects unsupported external alert commands");
}

void testHubBreadcrumbCommandIntegration()
{
    sa::BreadcrumbNavigator navigator;
    check(navigator.update({0.0f, 0.0f, 0.0f}, 10) &&
              navigator.update({0.0f, 1.0f, 0.0f}, 20),
          "default-on navigator records points before commands arrive");
    const std::size_t stored_points = navigator.breadcrumbs().size();
    bool blackout = false;
    std::string error;
    sa::HubCommand command{sa::HubCommandType::TrailToggle, false};
    check(sa::applyHubCommand(command, navigator, {0.0f, 1.0f, 0.0f}, 30, blackout, error) &&
              !navigator.enabled() && navigator.breadcrumbs().size() == stored_points,
          "trail-toggle OFF command pauses recording without erasing points");
    check(navigator.update({0.0f, 3.0f, 0.0f}, 40) &&
              navigator.breadcrumbs().size() == stored_points,
          "command-paused navigator ignores new poses");

    command = {sa::HubCommandType::TrailToggle, true};
    check(sa::applyHubCommand(command, navigator, {0.0f, 3.0f, 0.0f}, 50, blackout, error) &&
              navigator.enabled() && navigator.breadcrumbs().size() == stored_points,
          "trail-toggle ON command resumes without wiping history");

    command = {sa::HubCommandType::TrailPurge, false};
    const auto purge_points = navigator.breadcrumbs();
    check(sa::applyHubCommand(command, navigator, {0.0f, 3.0f, 0.0f}, 60, blackout, error) &&
              !navigator.enabled() && navigator.breadcrumbs().empty() && !blackout,
          "trail-purge invokes the zeroizing navigator purge without blackout");
    bool purge_zeroized = true;
    for (const sa::HudPoint& point : purge_points)
        purge_zeroized = purge_zeroized && point.x == 0.0f && point.y == 0.0f;
    check(purge_zeroized, "trail-purge zeroes the retained waypoint storage");

    check(navigator.setEnabled(true, {1.0f, 1.0f, 0.0f}, 70) &&
              navigator.update({1.0f, 1.6f, 0.0f}, 75),
          "navigator records after an explicit re-enable");
    const auto panic_points = navigator.breadcrumbs();
    command = {sa::HubCommandType::PanicWipe, false};
    check(sa::applyHubCommand(command, navigator, {1.0f, 1.0f, 0.0f}, 80, blackout, error) &&
              !navigator.enabled() && navigator.breadcrumbs().empty() && blackout,
          "panic-wipe clears the breadcrumb buffer and latches HUD blackout");
    bool panic_zeroized = true;
    for (const sa::HudPoint& point : panic_points)
        panic_zeroized = panic_zeroized && point.x == 0.0f && point.y == 0.0f;
    check(panic_zeroized, "panic-wipe zeroes the retained waypoint storage");
}

void testHudFrameSerialization()
{
    sa::HudRenderFrame frame{};
    frame.pose = {1.0f, 2.0f, 30.0f};
    frame.declutter = true;
    frame.wall_count = 1;
    frame.walls[0] = {0.0f, 1.0f, 2.0f, 3.0f};
    frame.target_count = 1;
    frame.targets[0] = {0.5f, 1.5f, 0.1f, 0.2f, 0.9f, sa::HudTargetSource::Tof};
    frame.presence_zone_count = 1;
    frame.presence_zones[0] = {1.0f, 3.0f, 0.25f, 0.5f, 0.8f};
    frame.breadcrumb_count = 1;
    frame.breadcrumb_path[0] = {9.0f, 9.0f};
    frame.announcement[0] = 'x';

    std::array<char, 4096> output{};
    std::size_t written = 0;
    std::string error;
    check(sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error),
          "HUD serializer accepts a bounded finite frame");
    const std::string_view json(output.data(), written);
    check(json.find("\"background\":\"#000000\"") != std::string_view::npos &&
              json.find("\"source\":\"tof\"") != std::string_view::npos &&
              json.find("\"prediction_1s\":[0.600,1.700]") != std::string_view::npos &&
              json.find("\"declutter\":true") != std::string_view::npos &&
              json.find("\"presence_zones\":[") != std::string_view::npos,
          "HUD JSON includes night mode, one-second prediction, and declutter state");
    check(json.find("\"breadcrumb_path\":[]") != std::string_view::npos &&
              json.find("\"emcon_mode\":\"MANUAL\"") != std::string_view::npos,
          "HUD JSON hides path points when trail recording is off");

    frame.has_glove_ray = true;
    frame.glove_ray = {true, 35.0f, 42, 3.25f};
    check(sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error) &&
              std::string_view(output.data(), written).find(
                  "\"glove_ray\":{\"active\":true,\"yaw_deg\":35.000,"
                  "\"locked_target_id\":42,\"locked_dist_m\":3.250}") != std::string_view::npos,
          "HUD JSON adds glove ray telemetry when supplied");

    frame.trail_enabled = true;
    check(sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error) &&
              std::string_view(output.data(), written).find("\"breadcrumb_path\":[[9.000,9.000]]") !=
                  std::string_view::npos,
          "HUD JSON includes breadcrumbs when recording is enabled");

    frame.pose.x = std::numeric_limits<float>::infinity();
    check(!sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error),
          "HUD serializer rejects non-finite render values");

    frame = sa::HudRenderFrame{};
    frame.blackout = true;
    frame.pose.x = std::numeric_limits<float>::infinity();
    check(sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error) &&
              std::string_view(output.data(), written).find("\"blackout\":true") !=
                  std::string_view::npos &&
              std::string_view(output.data(), written).find("\"targets\":[]") !=
                  std::string_view::npos,
          "blackout HUD frame hides target data while preserving the JSON frame shape");
}

sa::Track makeTrack(uint32_t id, const sa::Vec3& position, bool visible,
                   sa::TrackState state)
{
    sa::Track track{};
    track.id = id;
    track.filter.reset(position, {}, 0);
    track.visible = visible;
    track.state = state;
    return track;
}

void testGloveRaycaster()
{
    sa::WorldModel world{};
    world.pose = {0.0f, 0.0f, 0.0f};
    world.tracks.push_back(makeTrack(7, {0.2f, 3.0f, 0.0f}, true, sa::TrackState::VISIBLE));
    world.tracks.push_back(makeTrack(8, {0.0f, 2.0f, 0.0f}, false, sa::TrackState::PRESENCE_SUPPORTED));

    sa::GloveRaycaster raycaster;
    sa::GlovePosePacket packet{};
    packet.trigger_held = true;
    auto state = raycaster.update(world, packet, 100);
    check(state.active && state.locked_target_id == 7 &&
              std::fabs(state.locked_dist_m - std::sqrt(9.04f)) < 1.0e-4f,
          "glove ray locks the nearest visible target inside the cone");
    check(state.locked_target_id != 8,
          "glove ray does not turn a presence-supported occluded track into a target lock");

    sa::WorldModel outside_cone = world;
    outside_cone.tracks.clear();
    outside_cone.tracks.push_back(makeTrack(9, {1.0f, 3.0f, 0.0f}, true, sa::TrackState::VISIBLE));
    state = raycaster.update(outside_cone, packet, 110);
    check(state.locked_target_id == 0,
          "glove ray rejects visible targets outside the narrow pointing cone");

    sa::WorldModel within_range = world;
    within_range.tracks.clear();
    within_range.tracks.push_back(makeTrack(10, {0.0f, 8.1f, 0.0f}, true, sa::TrackState::VISIBLE));
    state = raycaster.update(within_range, packet, 120);
    check(state.locked_target_id == 0,
          "glove ray rejects targets beyond its maximum range");

    sa::WorldModel blocked = world;
    blocked.walls.push_back({{-1.0f, 2.0f, 0.0f}, {1.0f, 2.0f, 0.0f}, 2.5f});
    state = raycaster.update(blocked, packet, 130);
    check(state.locked_target_id == 0,
          "glove ray does not provide Geiger haptics for a target behind a mapped wall");

    sa::WorldModel near_wall = world;
    near_wall.walls.push_back({{-1.0f, 2.9f, 0.0f}, {1.0f, 2.9f, 0.0f}, 2.5f});
    state = raycaster.update(near_wall, packet, 140);
    check(state.locked_target_id == 7,
          "line-of-sight margin allows a target within 0.15 m of a mapped surface");

    sa::GlovePosePacket button_release{};
    button_release.yaw_deg = 0.0f;
    state = raycaster.update(world, button_release, 150);
    check(state.active, "releasing the wrist toggle button leaves hand-scan armed");
    packet.trigger_held = true;
    state = raycaster.update(world, packet, 160);
    check(!state.active, "a second wrist-button press toggles hand-scan off");

    sa::GlovePosePacket calibration{};
    calibration.yaw_deg = 25.0f;
    calibration.trigger_held = true;
    calibration.zero_heading_requested = true;
    raycaster.reset();
    state = raycaster.update(world, calibration, 200);
    check(state.active && std::fabs(state.yaw_deg) < 1.0e-4f &&
              state.locked_target_id == 7,
          "zero-heading calibration aligns the glove ray with body forward");
    state = raycaster.update(world, std::nullopt, 451);
    check(!state.active && state.locked_target_id == 0,
          "glove ray expires after more than 250 ms without a packet");
}

void testGlovePosePacket()
{
    const std::array<uint8_t, 10> bytes{{0x5B, 3, 0xD2, 0x04, 0x2E, 0xFB,
                                         0x01, 0x01, 0x0C, 0x00}};
    std::array<uint8_t, 10> packet_bytes = bytes;
    for (std::size_t i = 0; i < 9; ++i) packet_bytes[9] ^= packet_bytes[i];

    sa::GlovePosePacket packet{};
    std::string error;
    check(sa::parseGlovePosePacket(packet_bytes.data(), packet_bytes.size(), packet, error) &&
              packet.sequence == 3 && packet.trigger_held && packet.zero_heading_requested &&
              packet.events == 0x0C &&
              std::fabs(packet.yaw_deg - 12.34f) < 1.0e-4f &&
              std::fabs(packet.pitch_deg + 12.34f) < 1.0e-4f,
          "glove packet parser validates checksum and decodes signed centidegrees");
    packet_bytes[3] ^= 0x01;
    check(!sa::parseGlovePosePacket(packet_bytes.data(), packet_bytes.size(), packet, error),
          "glove packet parser rejects a checksum mismatch");
}

} // namespace

int main()
{
    testEmconController();
    testBreadcrumbNavigator();
    testHubCommandParser();
    testHubBreadcrumbCommandIntegration();
    testHudFrameSerialization();
    testGloveRaycaster();
    testGlovePosePacket();
    if (failures != 0) return 1;
    std::cout << "Core runtime module tests passed\n";
    return 0;
}
