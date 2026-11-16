#include "breadcrumb_navigator.hpp"
#include "emcon_controller.hpp"
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
    check(!navigator.enabled() && navigator.breadcrumbs().empty(),
          "breadcrumb recording defaults off with no points");
    check(navigator.setEnabled(true, {0.0f, 0.0f, 0.0f}, 10),
          "trail can be enabled at a valid pose");
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
    check(navigator.setEnabled(false, {0.0f, 1.1f, 0.0f}, 20) &&
              !navigator.enabled() && navigator.breadcrumbs().empty(),
          "turning recording off wipes the in-memory path");
    check(navigator.setEnabled(true, {1.0f, 2.0f, 0.0f}, 30),
          "trail can be re-enabled after clearing");
    navigator.purge();
    check(!navigator.enabled() && navigator.returnWaypointCount() == 0,
          "purge disables recording and removes all waypoints");
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
    const std::string csi = "{\"cmd\":\"csi_update\",\"zone_id\":2}";
    check(!sa::HubServer::parseCommand(csi.data(), csi.size(), command),
          "hub rejects unsupported CSI zone commands");
    const std::string alert = "{\"cmd\":\"external_alert\",\"severity\":3}";
    check(!sa::HubServer::parseCommand(alert.data(), alert.size(), command),
          "hub rejects unsupported external alert commands");
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

    frame.trail_enabled = true;
    check(sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error) &&
              std::string_view(output.data(), written).find("\"breadcrumb_path\":[[9.000,9.000]]") !=
                  std::string_view::npos,
          "HUD JSON includes breadcrumbs when recording is enabled");

    frame.pose.x = std::numeric_limits<float>::infinity();
    check(!sa::HubServer::serializeFrame(frame, output.data(), output.size(), written, error),
          "HUD serializer rejects non-finite render values");
}

} // namespace

int main()
{
    testEmconController();
    testBreadcrumbNavigator();
    testHubCommandParser();
    testHudFrameSerialization();
    if (failures != 0) return 1;
    std::cout << "Core runtime module tests passed\n";
    return 0;
}
