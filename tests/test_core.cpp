#include "announcer.hpp"
#include "fusion_engine.hpp"
#include "lidar_driver.hpp"
#include "marker_detector.hpp"
#include "occupancy_grid.hpp"
#include "playback.hpp"
#include "radar_driver.hpp"
#include "recorder.hpp"
#include "schematic.hpp"
#include "sensor_mode.hpp"
#include "serialization.hpp"
#include "simulation.hpp"
#include "track_manager.hpp"
#include "tof_driver.hpp"
#include "types.hpp"
#include "ultrasonic_driver.hpp"
#include "wrist_haptics.hpp"
#include "wallpad_driver.hpp"

#include <cmath>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace sa;

int g_failures = 0;

void check(bool condition, const std::string& what)
{
    if (!condition)
    {
        ++g_failures;
        std::cerr << "FAIL: " << what << "\n";
    }
}

bool near(float a, float b, float epsilon)
{
    return std::fabs(a - b) <= epsilon;
}

Detection makeDetection(float x, float y, float z = 0.9f)
{
    Detection detection{};
    detection.position = {x, y, z};
    detection.velocity = {0.0f, 0.0f, 0.0f};
    detection.range = std::sqrt(x * x + y * y + z * z);
    detection.snr = 15.0f;
    detection.from_radar = true;
    return detection;
}

// FusionEngine only promotes compact torso-height clusters to person
// candidates. Tests that exercise fusion therefore need person-like returns,
// not a single point.
std::vector<Detection> makePersonCluster(float x, float y)
{
    Detection low = makeDetection(x - 0.02f, y - 0.02f, 0.82f);
    Detection mid = makeDetection(x, y, 0.95f);
    Detection high = makeDetection(x + 0.02f, y + 0.02f, 1.08f);
    return {low, mid, high};
}

void testOccupancyGrid()
{
    OccupancyGrid grid;
    check(grid.config().width == 280 && grid.config().height == 280 &&
              grid.totalCellCount() == 280 * 280,
          "default occupancy grid is pre-sized to 280 by 280 cells");
    int far_cx = 0;
    int far_cy = 0;
    check(grid.worldToCell(0.0f, 30.0f, far_cx, far_cy) &&
              grid.worldToCell(30.0f, 0.0f, far_cx, far_cy),
          "30 m all-terrain scan endpoints fit within the default map");
    check(!grid.worldToCell(35.0f, 0.0f, far_cx, far_cy),
          "default map edge is bounded at 35 m");
    int cx = 0;
    int cy = 0;
    check(grid.worldToCell(1.0f, 1.0f, cx, cy), "worldToCell in range");
    int right_cx = 0;
    int right_cy = 0;
    int forward_cx = 0;
    int forward_cy = 0;
    check(grid.worldToCell(1.0f, 0.0f, right_cx, right_cy) &&
              grid.worldToCell(0.0f, 1.0f, forward_cx, forward_cy),
          "right and forward axes map into the grid");
    check(right_cx > grid.config().width / 2 && right_cy == grid.config().height / 2 &&
              forward_cx == grid.config().width / 2 && forward_cy > grid.config().height / 2,
          "grid +X is right and +Y is forward");
    grid.setProbability(cx, cy, 0.9f);
    grid.decay(0.5f);
    check(near(grid.probability(cx, cy), 0.7f, 1.0e-5f), "mandatory decay pulls probability toward 0.5");

    OccupancyGrid ray_grid;
    check(ray_grid.integrateRay({0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.85f, -0.35f, 20.0f),
          "integrateRay accepts a valid ray");
    int ex = 0;
    int ey = 0;
    ray_grid.worldToCell(0.0f, 1.0f, ex, ey);

    OccupancyGrid stamp_grid;
    stamp_grid.markOccupiedWorld({0.0f, 1.0f, 0.0f}, 0.85f);

    check(near(ray_grid.probability(ex, ey), stamp_grid.probability(ex, ey), 1.0e-6f),
          "ray endpoint receives occupied evidence exactly once");

    int free_cx = 0;
    int free_cy = 0;
    check(ray_grid.worldToCell(0.0f, 0.25f, free_cx, free_cy), "free-space sample cell is in range");
    check(ray_grid.probability(free_cx, free_cy) < 0.5f, "free-space traversal lowers occupancy");
}

void testRearRadarRotation()
{
    RadarConfig rear_config;
    rear_config.yaw_offset_deg = 180.0f;
    RadarFrame frame;
    Detection detection = makeDetection(1.0f, 2.0f, 0.5f);
    detection.velocity = {0.25f, -0.5f, 0.1f};
    frame.detections.push_back(detection);

    RadarDriver::transformToBody(frame, rear_config);
    check(frame.detections.size() == 1, "rear radar transform preserves detection count");
    if (!frame.detections.empty())
    {
        const Detection& rotated = frame.detections.front();
        check(near(rotated.position.x, -1.0f, 1.0e-5f) &&
                  near(rotated.position.y, -2.0f, 1.0e-5f) &&
                  near(rotated.position.z, 0.5f, 1.0e-5f),
              "rear radar position rotates 180 degrees into the canonical frame");
        check(near(rotated.velocity.x, -0.25f, 1.0e-5f) &&
                  near(rotated.velocity.y, 0.5f, 1.0e-5f),
              "rear radar velocity rotates with its position");
    }
}

void testKalmanTrack()
{
    Detection detection = makeDetection(2.0f, 0.0f);
    KalmanTrack track(detection, 1000);
    const float initial = track.positionSigma();
    check(initial > 0.0f, "Kalman position sigma is derived from covariance");

    for (int i = 0; i < 10; ++i) track.predict(0.05f);
    const float predicted = track.positionSigma();
    check(predicted > initial, "prediction grows position uncertainty");

    Detection update = detection;
    update.position.x += 0.02f;
    track.update(update);
    check(track.positionSigma() < predicted, "measurement update shrinks position uncertainty");
    check(near(track.position().x, 2.0f, 0.25f), "Kalman estimate stays near the measurement");
}

void testTrackManagerIdentityPersistence()
{
    TrackManager manager;
    std::vector<Detection> detections{makeDetection(0.0f, 2.0f)};
    manager.process(0.05f, detections, 1000);
    check(manager.tracks().size() == 1, "first detection spawns one track");
    const uint32_t id = manager.tracks().front().id;

    check(manager.applyIdentity(id, 0x2A01, 0.9f, 1000), "UWB identity applies to an existing track");
    check(manager.findTrack(id) != nullptr && manager.findTrack(id)->identity.tagged_safe,
          "track is marked safe-tagged");

    for (int i = 1; i <= 400; ++i)
        manager.process(0.05f, {}, 1000 + static_cast<uint64_t>(i) * 50);
    const Track* tagged = manager.findTrack(id);
    check(tagged != nullptr, "tagged identity persists beyond visibility");
    if (tagged)
    {
        check(!tagged->visible, "track is no longer visible while occluded");
        check(tagged->identity.tagged_safe && tagged->identity.tag_id == 0x2A01,
              "tag identity is not aggressively decayed");
    }

    TrackManager untagged_manager;
    untagged_manager.process(0.05f, detections, 1000);
    const uint32_t untagged_id = untagged_manager.tracks().front().id;
    for (int i = 1; i <= 400; ++i)
        untagged_manager.process(0.05f, {}, 1000 + static_cast<uint64_t>(i) * 50);
    check(untagged_manager.findTrack(untagged_id) == nullptr,
          "untagged track expires faster than a tagged one");
}

void testPresenceSupportRules()
{
    TrackManager manager;
    std::vector<Detection> detections{makeDetection(0.0f, 2.0f)};
    manager.process(0.05f, detections, 1000);
    const uint32_t id = manager.tracks().front().id;
    check(!manager.applyPresenceSupport(id, 0.9f, 1000), "Novelda cannot reinforce a visible track");

    manager.process(0.05f, {}, 1100);
    check(manager.findTrack(id) != nullptr && !manager.findTrack(id)->visible,
          "track becomes occluded without radar");
    check(manager.applyPresenceSupport(id, 0.9f, 1100), "Novelda can reinforce an occluded track");
    check(manager.findTrack(id) != nullptr && manager.findTrack(id)->state == TrackState::PRESENCE_SUPPORTED,
          "presence support sets PRESENCE_SUPPORTED");
}

void testWallpadPresenceRules()
{
    FusionEngine fusion;
    SensorFrame radar_frame;
    radar_frame.frame_id = 1;
    radar_frame.timestamp_ms = 1000;
    RadarFrame radar;
    radar.timestamp_ms = 1000;
    radar.detections = makePersonCluster(0.0f, 2.0f);
    radar_frame.radar = radar;
    check(fusion.process(radar_frame).tracks.size() == 1,
          "radar creates a track before wall-pad presence support");

    SensorFrame coast_frame;
    coast_frame.frame_id = 2;
    coast_frame.timestamp_ms = 1050;
    const WorldModel coasting = fusion.process(coast_frame);
    check(coasting.tracks.size() == 1 && coasting.tracks.front().state == TrackState::OCCLUDED,
          "track becomes occluded before wall-pad support");

    SensorFrame presence_frame;
    presence_frame.frame_id = 3;
    presence_frame.timestamp_ms = 1100;
    presence_frame.wallpad_zones.push_back({1.2f, 2.8f, 0.0f, degToRad(60.0f), 0.9f});
    const WorldModel supported = fusion.process(presence_frame);
    check(supported.tracks.size() == 1 &&
              supported.tracks.front().state == TrackState::PRESENCE_SUPPORTED,
          "wall-pad sector reinforces an existing occluded radar track");
    check(supported.zones.size() == 1,
          "wall-pad sector remains available in the world model");

    FusionEngine presence_only;
    SensorFrame untracked_presence;
    untracked_presence.frame_id = 1;
    untracked_presence.timestamp_ms = 2000;
    untracked_presence.wallpad_zones.push_back({1.0f, 3.0f, 0.0f, degToRad(45.0f), 0.8f});
    check(presence_only.process(untracked_presence).tracks.empty(),
          "wall-pad presence never creates a standalone track");
}

uint32_t gridHash(const WorldModel& world)
{
    const auto encoded = world.occupancy.encodedProbabilities();
    return crc32(reinterpret_cast<const uint8_t*>(encoded.data()), encoded.size() * sizeof(uint16_t));
}

void testUwbOnlyCreatesNoTracks()
{
    FusionEngine fusion;
    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1000;
    UwbMeasurement uwb{};
    uwb.timestamp_ms = 1000;
    uwb.tag_id = 42;
    uwb.range_m = 2.0f;
    uwb.has_aoa = true;
    uwb.aoa_rad = 0.0f;
    uwb.confidence = 0.95f;
    frame.uwb.push_back(uwb);

    const WorldModel world = fusion.process(frame);
    check(world.tracks.empty(), "UWB measurement alone never creates a track");
}

void testNoveldaOnlyCreatesNoTracks()
{
    FusionEngine fusion;
    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1000;
    NoveldaMeasurement presence{};
    presence.timestamp_ms = 1000;
    presence.range_m = 2.0f;
    presence.bearing_center_rad = 0.0f;
    presence.bearing_width_rad = degToRad(30.0f);
    presence.confidence = 0.9f;
    frame.novelda.push_back(presence);

    const WorldModel world = fusion.process(frame);
    check(world.tracks.empty(), "Novelda presence alone never creates a track");
}

void testImuStationaryZupt()
{
    SlamManager slam;
    ImuSample sample;
    sample.accel_mps2 = {2.0f, 0.0f, kGravity};
    for (int i = 0; i < 10; ++i)
    {
        sample.timestamp_ms += 20;
        slam.update(sample, 0.02f);
    }
    check(norm(slam.egoVelocity()) > 0.05f, "IMU acceleration produces an ego-motion estimate");

    sample.accel_mps2 = {0.0f, 0.0f, kGravity};
    sample.gyro_radps = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 25; ++i)
    {
        sample.timestamp_ms += 20;
        slam.update(sample, 0.02f);
    }
    check(slam.ahrs().stationary(), "stationary IMU samples engage the ZUPT lock");
    check(norm(slam.egoVelocity()) < 1.0e-5f, "ZUPT resets accumulated translational velocity");
}

void testCorrelativeTofScanMatching()
{
    OccupancyGrid grid;
    const std::vector<Vec3> landmarks{
        {-0.875f, 1.125f, 0.0f}, {-0.375f, 1.875f, 0.0f}, {0.125f, 2.625f, 0.0f},
        {0.625f, 3.375f, 0.0f},  {1.125f, 4.125f, 0.0f},  {-1.375f, 2.375f, 0.0f},
        {0.875f, 1.375f, 0.0f},  {-0.125f, 3.875f, 0.0f}, {1.625f, 2.125f, 0.0f},
        {-1.875f, 4.375f, 0.0f}};
    for (const Vec3& point : landmarks)
    {
        int cx = 0;
        int cy = 0;
        check(grid.worldToCell(point.x, point.y, cx, cy), "scan-matching landmark is inside the map");
        grid.setProbability(cx, cy, 0.95f);
    }

    TofScan scan;
    for (const Vec3& point : landmarks)
        scan.rays.push_back({{0.0f, 0.0f, 0.0f}, point});

    SlamManager slam;
    slam.setPose({0.18f, 0.0f, 0.0f});
    slam.update(scan, grid, 0.50f);
    check(near(slam.pose().x, 0.18f, 1.0e-5f),
          "ToF scan matching remains disabled at the environment-confidence threshold");
    slam.update(scan, grid, 0.80f);
    check(slam.pose().x < 0.15f && slam.pose().x > 0.0f,
          "correlative ToF matching reduces local x drift by at least 3 cm");
}

void testLongRangeScanMatching()
{
    OccupancyGrid grid;
    const std::vector<Vec3> landmarks{
        {0.20f, 10.20f, 0.0f}, {0.45f, 12.20f, 0.0f}, {0.70f, 14.20f, 0.0f},
        {0.95f, 16.20f, 0.0f}, {1.20f, 18.20f, 0.0f}, {-0.05f, 20.20f, 0.0f},
        {-0.30f, 22.20f, 0.0f}, {-0.55f, 24.20f, 0.0f}, {-0.80f, 26.20f, 0.0f},
        {-1.05f, 28.20f, 0.0f}};
    for (const Vec3& point : landmarks)
    {
        int cx = 0;
        int cy = 0;
        check(grid.worldToCell(point.x, point.y, cx, cy), "30 m scan landmark is within map bounds");
        grid.setProbability(cx, cy, 0.95f);
    }

    TofScan scan;
    for (const Vec3& point : landmarks)
        scan.rays.push_back({{0.0f, 0.0f, 0.0f}, point});

    SlamManager slam;
    slam.setPose({0.12f, 0.0f, 0.0f});
    slam.update(scan, grid, 0.80f);
    check(slam.pose().x < 0.07f && slam.pose().x > -0.04f,
          "30 m scan returns a drifted pose toward mapped landmarks");
}

void testBurstLoopClosure()
{
    OccupancyGrid grid;
    const std::vector<Vec3> scan_points{
        {-2.625f, -0.125f, 0.0f}, {-2.125f, 1.375f, 0.0f}, {-1.125f, 2.625f, 0.0f},
        {0.125f, 3.625f, 0.0f},   {1.625f, 2.875f, 0.0f},  {2.625f, 1.375f, 0.0f},
        {3.125f, -0.125f, 0.0f},  {2.375f, -1.625f, 0.0f}, {1.125f, -2.625f, 0.0f},
        {-0.375f, -3.125f, 0.0f}, {-1.875f, -2.375f, 0.0f}, {-3.125f, -1.375f, 0.0f},
        {-1.625f, 0.375f, 0.0f},  {0.625f, 1.125f, 0.0f},  {1.375f, -0.375f, 0.0f},
        {-0.625f, -1.125f, 0.0f}};
    for (const Vec3& point : scan_points)
    {
        int cx = 0;
        int cy = 0;
        check(grid.worldToCell(point.x, point.y, cx, cy), "loop-closure landmark is inside the map");
        grid.setProbability(cx, cy, 0.95f);
    }

    SlamManager slam;
    slam.setPose({1.0f, -0.5f, 12.0f});
    LidarScan active_burst;
    active_burst.points = scan_points;
    active_burst.burst_active = true;
    slam.update(active_burst, grid, 0.49f);
    check(near(slam.pose().x, 1.0f, 1.0e-5f),
          "loop closure is deferred until a burst-completion event");
    const float confidence_before_closure = slam.confidence();

    LidarScan completed_burst;
    completed_burst.burst_complete = true;
    slam.update(completed_burst, grid, 0.80f);
    check(std::abs(slam.pose().x) < 0.20f && std::abs(slam.pose().y) < 0.20f &&
              std::abs(slam.pose().yaw_deg) < 3.0f,
          "completed 360-degree burst snaps to a high-confidence mapped pose");
    check(slam.confidence() > confidence_before_closure,
          "successful loop closure increases localization confidence");

    OccupancyGrid unmapped_grid;
    SlamManager rejected;
    rejected.setPose({1.0f, -0.5f, 12.0f});
    check(!rejected.detectLoopClosure(scan_points, unmapped_grid, 0.80f) &&
              near(rejected.pose().x, 1.0f, 1.0e-5f),
          "loop closure rejects scans without enough high-confidence map matches");
}

void testUwbAnchorCorrection()
{
    UwbMeasurement anchor;
    anchor.tag_id = 17;
    anchor.range_m = 1.0f;
    anchor.confidence = 0.8f;
    anchor.is_fixed_anchor = true;
    anchor.anchor_x_m = 2.0f;
    anchor.anchor_y_m = 0.0f;

    SlamManager slam;
    slam.correctPose(anchor);
    check(slam.pose().x > 0.0f && slam.pose().x < 1.0f,
          "fixed UWB anchor softly pulls pose toward its measured range circle");

    FusionEngine fusion;
    SensorFrame frame;
    frame.timestamp_ms = 1000;
    anchor.timestamp_ms = 1000;
    frame.uwb.push_back(anchor);
    const WorldModel world = fusion.process(frame);
    check(world.pose.x > 0.0f && world.tracks.empty(),
          "fixed UWB anchor constrains device pose without creating a person track");
}

void testUwbAssociation()
{
    FusionEngine fusion;

    SensorFrame radar_frame;
    radar_frame.frame_id = 1;
    radar_frame.timestamp_ms = 1000;
    RadarFrame radar;
    radar.timestamp_ms = 1000;
    const auto person = makePersonCluster(0.0f, 2.0f);
    radar.detections.insert(radar.detections.end(), person.begin(), person.end());
    radar_frame.radar = radar;
    const WorldModel first = fusion.process(radar_frame);
    check(first.tracks.size() == 1, "radar creates the primary track");

    SensorFrame uwb_frame;
    uwb_frame.frame_id = 2;
    uwb_frame.timestamp_ms = 1050;
    UwbMeasurement uwb{};
    uwb.timestamp_ms = 1050;
    uwb.tag_id = 0x1234;
    uwb.range_m = 2.0f;
    uwb.has_aoa = true;
    uwb.aoa_rad = 0.0f;
    uwb.has_range_rate = true;
    uwb.range_rate_mps = 0.0f;
    uwb.confidence = 0.9f;
    uwb_frame.uwb.push_back(uwb);

    const WorldModel second = fusion.process(uwb_frame);
    check(second.tracks.size() == 1, "UWB does not spawn an additional track");
    if (!second.tracks.empty())
    {
        check(second.tracks.front().identity.tagged_safe, "UWB associates identity to existing track");
        check(second.tracks.front().identity.tag_id == 0x1234, "UWB association preserves tag id");
    }
}

void testPresenceFusionNeedsOverlap()
{
    FusionEngine fusion;
    SensorFrame radar_frame;
    radar_frame.frame_id = 1;
    radar_frame.timestamp_ms = 1000;
    RadarFrame radar;
    radar.timestamp_ms = 1000;
    const auto person = makePersonCluster(0.0f, 2.0f);
    radar.detections.insert(radar.detections.end(), person.begin(), person.end());
    radar_frame.radar = radar;
    check(fusion.process(radar_frame).tracks.size() == 1, "presence fixture creates a track");

    SensorFrame matching;
    matching.frame_id = 2;
    matching.timestamp_ms = 1100;
    NoveldaMeasurement presence{};
    presence.timestamp_ms = 1100;
    presence.range_m = 2.0f;
    presence.bearing_center_rad = 0.0f;
    presence.bearing_width_rad = degToRad(25.0f);
    presence.confidence = 0.85f;
    matching.novelda.push_back(presence);
    const WorldModel supported = fusion.process(matching);
    check(!supported.zones.empty(), "Novelda produces a presence zone");
    check(supported.tracks.size() == 1 &&
              supported.tracks.front().state == TrackState::PRESENCE_SUPPORTED,
          "overlapping presence zone reinforces the occluded track");

    FusionEngine other;
    SensorFrame radar_frame2 = radar_frame;
    check(other.process(radar_frame2).tracks.size() == 1, "negative presence fixture creates a track");
    SensorFrame non_matching;
    non_matching.frame_id = 2;
    non_matching.timestamp_ms = 1100;
    NoveldaMeasurement far{};
    far.timestamp_ms = 1100;
    far.range_m = 5.0f;
    far.bearing_center_rad = degToRad(80.0f);
    far.bearing_width_rad = degToRad(10.0f);
    far.confidence = 0.85f;
    non_matching.novelda.push_back(far);
    const WorldModel unsupported = other.process(non_matching);
    check(unsupported.tracks.size() == 1 &&
              unsupported.tracks.front().state != TrackState::PRESENCE_SUPPORTED,
          "non-overlapping presence zone does not reinforce a track");
}

void testRedMarkerDetector()
{
    const int width = 64;
    const int height = 48;
    const int stride = width * 3;
    std::vector<uint8_t> image(static_cast<std::size_t>(width * height * 3), 24);
    auto paintRed = [&](int x0, int y0, int x1, int y1) {
        for (int y = y0; y < y1; ++y)
        {
            for (int x = x0; x < x1; ++x)
            {
                uint8_t* pixel = image.data() + static_cast<std::size_t>(y * stride + x * 3);
                pixel[0] = 220;
                pixel[1] = 25;
                pixel[2] = 28;
            }
        }
    };

    paintRed(6, 14, 20, 34); // left of the optical axis
    RgbFrameView view{image.data(), width, height, stride};
    RedMarkerDetector detector;
    const MarkerDetection left = detector.detect(view);
    check(left.valid, "red marker detector finds a red cluster");
    check(left.bearing_rad < -0.03f, "left-of-axis marker has negative bearing");
    check(left.confidence >= 0.35f, "red marker confidence is usable");
    check(left.pixel_count >= 24, "red marker cluster is large enough");

    std::vector<uint8_t> right_image(static_cast<std::size_t>(width * height * 3), 24);
    for (int y = 14; y < 34; ++y)
    {
        for (int x = 44; x < 58; ++x)
        {
            uint8_t* pixel = right_image.data() + static_cast<std::size_t>(y * stride + x * 3);
            pixel[0] = 220;
            pixel[1] = 25;
            pixel[2] = 28;
        }
    }
    const MarkerDetection right = detector.detect({right_image.data(), width, height, stride});
    check(right.valid && right.bearing_rad > 0.03f, "right-of-axis marker has positive bearing");

    std::vector<uint8_t> blank(static_cast<std::size_t>(width * height * 3), 24);
    RgbFrameView blank_view{blank.data(), width, height, stride};
    check(!detector.detect(blank_view).valid, "gray image produces no red marker");

    std::vector<uint8_t> tiny(static_cast<std::size_t>(width * height * 3), 24);
    for (int y = 20; y < 22; ++y)
    {
        for (int x = 30; x < 32; ++x)
        {
            uint8_t* pixel = tiny.data() + static_cast<std::size_t>(y * stride + x * 3);
            pixel[0] = 230;
            pixel[1] = 20;
            pixel[2] = 20;
        }
    }
    RgbFrameView tiny_view{tiny.data(), width, height, stride};
    check(!detector.detect(tiny_view).valid, "tiny red speck is rejected");
}

void testWristGuidanceHysteresis()
{
    WristGuidance guidance;
    MarkerObservation marker{};
    marker.valid = true;
    marker.confidence = 0.9f;
    marker.timestamp_ms = 1000;

    marker.bearing_rad = degToRad(2.0f);
    check(guidance.update(marker, 1000).aimed, "wrist engages inside the aim cone");

    marker.bearing_rad = degToRad(11.0f);
    check(guidance.update(marker, 1000).aimed, "wrist holds vibration until the release angle");

    marker.bearing_rad = degToRad(15.0f);
    check(!guidance.update(marker, 1000).aimed, "wrist releases outside the release angle");

    marker.bearing_rad = degToRad(9.0f);
    check(!guidance.update(marker, 1000).aimed, "wrist must re-enter the engage angle");

    marker.bearing_rad = 0.0f;
    marker.timestamp_ms = 1000;
    check(!guidance.update(marker, 2000).aimed, "stale marker does not vibrate");

    marker.timestamp_ms = 2000;
    marker.confidence = 0.1f;
    check(!guidance.update(marker, 2000).aimed, "low-confidence marker does not vibrate");

    marker.confidence = 0.9f;
    check(guidance.update(marker, 2000).aimed, "fresh confident marker re-engages after reset");
}

void testWristVibratorOutput()
{
    const std::string path = "sa_test_wrist.txt";
    std::remove(path.c_str());

    auto readFile = [&]() {
        std::ifstream file(path, std::ios::binary);
        std::ostringstream content;
        content << file.rdbuf();
        return content.str();
    };
    auto waitForFile = [&](const std::string& expected) {
        for (int attempt = 0; attempt < 200; ++attempt)
        {
            if (readFile() == expected) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    };

    WristConfig config;
    config.device_path = path;
    WristVibrator vibrator(config);
    std::string error;
    check(vibrator.start(error), "wrist actuator opens its output: " + error);

    vibrator.update(1000, true);
    check(vibrator.outputOn(), "wrist actuator reports active output");
    check(waitForFile("1"), "wrist asynchronously writes active value while aimed");

    vibrator.update(1050, false);
    check(!vibrator.outputOn(), "wrist actuator reports inactive output");
    check(waitForFile("0"), "wrist asynchronously writes inactive value when aim is lost");

    vibrator.stop();
    std::remove(path.c_str());
}

void testWristOrientationParserAndPointing()
{
    float yaw = 0.0f;
    float pitch = 0.0f;
    std::string error;
    check(WristHaptics::parseOrientationLine("WRIST,45,-8", yaw, pitch, error),
          "wrist bridge accepts yaw/pitch packets");
    check(near(yaw, 45.0f, 1.0e-5f) && near(pitch, -8.0f, 1.0e-5f),
          "wrist bridge preserves orientation values");
    check(!WristHaptics::parseOrientationLine("WRIST,nan,0", yaw, pitch, error),
          "wrist bridge rejects non-finite orientation");
    check(!WristHaptics::parseOrientationLine("RADAR,0,0", yaw, pitch, error),
          "wrist bridge rejects unrelated packet types");

    WristHaptics guidance;
    WorldModel world{};
    world.timestamp_ms = 1000;
    Track unknown{};
    unknown.id = 1;
    unknown.filter.reset({0.0f, 2.0f, 1.0f}, {}, 1000);
    unknown.identity = {false, 0, 0.0f};
    unknown.state = TrackState::VISIBLE;
    unknown.visible = true;
    world.tracks.push_back(unknown);

    WristOrientation orientation{1000, 45.0f, 0.0f};
    const WristHapticState dead_on = guidance.update(world, orientation, 1000);
    check(dead_on.orientation_valid && dead_on.pointing &&
              dead_on.pattern == WristHapticPattern::UNKNOWN_PERSON,
          "startup calibration aligns the forward wrist direction with the target");
    check(near(dead_on.wrist_yaw_deg, 0.0f, 1.0e-5f) &&
              near(dead_on.frequency_hz, 20.0f, 1.0e-5f),
          "dead-on unknown-person pointing produces the maximum pulse rate");

    orientation.timestamp_ms = 1010;
    orientation.wrist_yaw_deg = 60.0f;
    const WristHapticState cone_edge = guidance.update(world, orientation, 1010);
    check(cone_edge.pointing && near(cone_edge.frequency_hz, 4.0f, 1.0e-5f),
          "pointing-cone edge produces the minimum 4 Hz pulse rate");

    orientation.timestamp_ms = 1020;
    orientation.wrist_yaw_deg = 70.0f;
    const WristHapticState pointing_away = guidance.update(world, orientation, 1020);
    check(!pointing_away.pointing && pointing_away.pattern == WristHapticPattern::SILENT &&
              near(pointing_away.duty_pct, 0.0f, 1.0e-5f),
          "pointing outside the cone keeps the wrist actuator silent");

    world.tracks.front().identity.tagged_safe = true;
    orientation.timestamp_ms = 1030;
    orientation.wrist_yaw_deg = 45.0f;
    const WristHapticState safe_target = guidance.update(world, orientation, 1030);
    check(safe_target.pattern == WristHapticPattern::SAFE_TAGGED &&
              safe_target.duty_pct < dead_on.duty_pct,
          "safe-tagged pointing uses a distinct low-intensity cue");

    orientation.timestamp_ms = 1040;
    const WristHapticState collision = guidance.update(world, orientation, 1040, true);
    check(collision.pattern == WristHapticPattern::COLLISION && collision.duty_pct == 100.0f,
          "close-range collision warning overrides the pointing cone");
}

void testSensorSelection()
{
    SensorSelection selection;
    std::string error;

    check(parseSensorSelection("mixed", selection, error), "mixed preset parses");
    check(selection.mask == kAllSensorMask, "mixed enables every sensor");
    check(sensorSelectionName(selection) == "mixed", "mixed keeps its preset name");

    check(parseSensorSelection("radar", selection, error), "radar preset parses");
    check(selection.has(SensorKind::Radar), "radar preset enables radar");
    check(selection.has(SensorKind::Imu), "radar preset retains IMU ego-motion");
    check(!selection.has(SensorKind::Uwb), "radar preset disables UWB");
    check(sensorSelectionName(selection) == "radar", "radar keeps its preset name");

    check(parseSensorSelection("lidar", selection, error), "lidar preset parses");
    check(selection.has(SensorKind::Lidar) && selection.has(SensorKind::Imu),
          "lidar preset enables LiDAR and IMU");
    check(!selection.has(SensorKind::Radar), "lidar preset disables radar");
    check(sensorSelectionName(selection) == "lidar", "lidar keeps its preset name");

    check(parseSensorSelection("tof", selection, error), "tof preset parses");
    check(selection.has(SensorKind::Tof) && selection.has(SensorKind::Imu),
          "tof preset enables ToF and IMU");
    check(!selection.has(SensorKind::Lidar), "tof preset disables LiDAR");
    check(sensorSelectionName(selection) == "tof", "tof keeps its preset name");

    check(parseSensorSelection("wallpad", selection, error), "wall-pad preset parses");
    check(selection.has(SensorKind::Wallpad) && selection.has(SensorKind::Imu),
          "wall-pad preset enables wall-pad presence and IMU");
    check(sensorSelectionName(selection) == "wallpad", "wall-pad keeps its preset name");

    check(parseSensorSelection("imu,radar, UWB ,marker", selection, error), "custom sensor list parses");
    check(selection.has(SensorKind::Imu) && selection.has(SensorKind::Radar) &&
              selection.has(SensorKind::Uwb) && selection.has(SensorKind::Marker),
          "custom list enables each named sensor");
    check(sensorSelectionName(selection) == "imu+radar+uwb+marker", "custom list has a stable name");

    check(!parseSensorSelection("unknown", selection, error), "unknown sensor is rejected");
    check(!parseSensorSelection("", selection, error), "empty sensor selection is rejected");

    SensorFrame frame;
    frame.frame_id = 7;
    frame.timestamp_ms = 4200;
    frame.imu = ImuSample{};
    RadarFrame radar;
    radar.detections.push_back(makeDetection(2.0f, 0.0f));
    frame.radar = radar;
    LidarScan lidar;
    lidar.points.push_back({1.0f, 2.0f, 0.5f});
    frame.lidar = lidar;
    TofScan tof;
    tof.rays.push_back({{0.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}});
    frame.tof = tof;
    frame.uwb.push_back(UwbMeasurement{});
    frame.wallpad_zones.push_back({1.0f, 3.0f, 0.0f, degToRad(50.0f), 0.7f});
    frame.ultrasonic = UltrasonicMeasurement{};
    frame.marker = MarkerObservation{};

    SensorSelection radar_only;
    check(parseSensorSelection("radar", radar_only, error), "radar selection parses for filtering");
    const SensorFrame filtered = filterSensorFrame(frame, radar_only);
    check(filtered.imu.has_value(), "filter keeps selected IMU");
    check(filtered.radar.has_value() && filtered.radar->detections.size() == 1, "filter keeps selected radar");
    check(filtered.uwb.empty(), "filter drops unselected UWB");
    check(!filtered.ultrasonic.has_value(), "filter drops unselected ultrasonic");
    check(!filtered.marker.has_value(), "filter drops unselected marker");
    check(!filtered.lidar.has_value(), "filter drops unselected LiDAR");
    check(!filtered.tof.has_value(), "filter drops unselected ToF");
    check(filtered.wallpad_zones.empty(), "filter drops unselected wall-pad sectors");
    check(filtered.frame_id == frame.frame_id && filtered.timestamp_ms == frame.timestamp_ms,
          "filter preserves frame id and timestamp");
    check(frame.uwb.size() == 1 && frame.marker.has_value(), "filter leaves the raw frame untouched");

    SensorSelection wallpad_only;
    check(parseSensorSelection("wallpad", wallpad_only, error), "wall-pad selection parses for filtering");
    const SensorFrame wallpad_filtered = filterSensorFrame(frame, wallpad_only);
    check(wallpad_filtered.wallpad_zones.size() == 1 && !wallpad_filtered.radar.has_value(),
          "filter keeps wall-pad sectors and drops unselected radar");
}

void testLidarPointParser()
{
    Vec3 point{};
    std::string error;
    check(LidarDriver::parsePointLine("LIDAR,1.25,2.5", point, error),
          "LiDAR parser accepts 2D bridge points");
    check(near(point.x, 1.25f, 1.0e-5f) && near(point.y, 2.5f, 1.0e-5f) && near(point.z, 0.0f, 1.0e-5f),
          "2D LiDAR records default height to zero");
    check(LidarDriver::parsePointLine(" LIDAR, -1, 0.75, 1.5 ", point, error),
          "LiDAR parser accepts 3D bridge points");
    check(near(point.x, -1.0f, 1.0e-5f) && near(point.y, 0.75f, 1.0e-5f) &&
              near(point.z, 1.5f, 1.0e-5f),
          "3D LiDAR coordinates are preserved");
    check(!LidarDriver::parsePointLine("LIDAR,nan,1", point, error),
          "LiDAR parser rejects non-finite coordinates");
    check(!LidarDriver::parsePointLine("RADAR,1,2", point, error),
          "LiDAR parser rejects another sensor's record prefix");
}

void testWallpadZoneParser()
{
    PresenceZone zone{};
    std::string error;
    check(WallpadDriver::parseZoneLine("WALLPAD,1.2,3.8,25,40,0.8", 30.0f, zone, error),
          "wall-pad parser accepts a valid sector bridge record");
    check(near(zone.range_min, 1.2f, 1.0e-5f) &&
              near(zone.range_max, 3.8f, 1.0e-5f) &&
              near(zone.bearing_center_rad, degToRad(25.0f), 1.0e-5f) &&
              near(zone.bearing_width_rad, degToRad(40.0f), 1.0e-5f) &&
              near(zone.confidence, 0.8f, 1.0e-5f),
          "wall-pad parser converts bridge fields into a PresenceZone");
    check(!WallpadDriver::parseZoneLine("WALLPAD,1.2,31,0,45,0.8", 30.0f, zone, error),
          "wall-pad parser rejects sectors beyond the configured range");
    check(!WallpadDriver::parseZoneLine("WALLPAD,1.2,3.8,0,0,0.8", 30.0f, zone, error),
          "wall-pad parser rejects invalid bearing widths");
}

void testUltrasonicCarriageReturnBridge()
{
    UltrasonicDriver driver;
    std::vector<UltrasonicMeasurement> measurements;
    const std::string first = "R1234\r";
    check(driver.ingestBytes(reinterpret_cast<const uint8_t*>(first.data()), first.size(), 420,
                             measurements),
          "ultrasonic parser accepts the locked carriage-return terminator");
    check(measurements.size() == 1 && measurements[0].valid &&
              near(measurements[0].range_m, 1.234f, 1.0e-5f) &&
              measurements[0].timestamp_ms == 420,
          "ultrasonic CR record preserves range and injected timestamp");

    const std::string partial = "R56";
    check(!driver.ingestBytes(reinterpret_cast<const uint8_t*>(partial.data()), partial.size(), 430,
                              measurements),
          "ultrasonic parser waits for a complete record across reads");
    const std::string remainder = "78\r\nR0000\r";
    check(driver.ingestBytes(reinterpret_cast<const uint8_t*>(remainder.data()), remainder.size(), 440,
                             measurements),
          "ultrasonic parser accepts split CR and CRLF-terminated records");
    check(measurements.size() == 3 && near(measurements[1].range_m, 5.678f, 1.0e-5f) &&
              !measurements[2].valid,
          "ultrasonic parser emits both complete records and ignores the CRLF empty line");
}

void testLd19PacketParser()
{
    std::array<uint8_t, 47> packet{};
    packet[0] = 0x54;
    packet[1] = 0x2C;
    for (std::size_t i = 0; i < 12; ++i)
    {
        packet[6 + i * 3] = 0xE8;
        packet[7 + i * 3] = 0x03;
    }
    packet[46] = LidarDriver::crc8(packet.data(), packet.size() - 1);
    std::vector<Vec3> points;
    std::string error;
    check(LidarDriver::parseLd19Packet(packet, points, error), "LD19 parser accepts a valid 47-byte packet");
    check(points.size() == 12, "LD19 parser extracts all twelve distance samples");
    if (!points.empty())
        check(near(points.front().x, 0.0f, 1.0e-5f) && near(points.front().y, 1.0f, 1.0e-5f),
              "LD19 zero-degree sample projects along +Y forward");
    packet[4] = 0x28;
    packet[5] = 0x23;
    packet[42] = 0x28;
    packet[43] = 0x23;
    packet[46] = LidarDriver::crc8(packet.data(), packet.size() - 1);
    check(LidarDriver::parseLd19Packet(packet, points, error), "LD19 parser accepts a valid leftward bearing");
    if (!points.empty())
        check(near(points.front().x, -1.0f, 1.0e-5f) && near(points.front().y, 0.0f, 1.0e-5f),
              "LD19 counter-clockwise bearing projects toward -X left");
    packet[46] ^= 0x01;
    check(!LidarDriver::parseLd19Packet(packet, points, error), "LD19 parser rejects a CRC mismatch");
}

void testLidarBurstState()
{
    LidarConfig config;
    config.power_gate_enabled = false;
    check(config.port == "/dev/ttyS2", "LiDAR defaults to the locked UART2 device path");
    check(config.motor_enable.chip == "/dev/gpiochip3" &&
              config.motor_enable.offset == 12 && config.motor_enable.active_high,
          "LiDAR defaults to the locked active-high motor-gate GPIO");
    LidarDriver lidar(config);
    std::string error;
    check(lidar.triggerScanBurst(1000), "LiDAR burst starts from an injected timestamp");
    check(lidar.motorPowered() && lidar.isBurstActive(1000), "burst enables power and enters active state");
    check(!lidar.triggerScanBurst(1100), "repeated scan trigger is ignored while active");

    LidarScan scan;
    const std::string point = "LIDAR,0,1\n";
    check(!lidar.ingestBytes(reinterpret_cast<const uint8_t*>(point.data()), point.size(), 1299, scan, error),
          "LiDAR points are discarded during the first 300 ms");
    check(lidar.ingestBytes(reinterpret_cast<const uint8_t*>(point.data()), point.size(), 1300, scan, error),
          "LiDAR points are accepted after the 300 ms spin-up window");
    check(scan.points.size() == 1, "accepted burst data reaches the scan frame");
    check(scan.burst_active, "LiDAR data frames identify the active burst");
    check(!lidar.ingestBytes(nullptr, 0, 2500, scan, error), "burst ends at its deterministic deadline");
    check(!lidar.motorPowered() && !lidar.isBurstActive(2500), "deadline automatically cuts scan power");
}

void testLidarOwnsMapping()
{
    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1000;
    RadarFrame radar;
    radar.timestamp_ms = 1000;
    Detection radar_return = makeDetection(3.0f, 3.0f, 0.2f);
    radar.detections.push_back(radar_return);
    frame.radar = radar;
    LidarScan scan;
    scan.timestamp_ms = 1000;
    scan.points.push_back({0.0f, 2.0f, 0.0f});
    frame.lidar = scan;

    FusionEngine with_lidar;
    const WorldModel lidar_world = with_lidar.process(frame);
    int lidar_x = 0;
    int lidar_y = 0;
    int radar_x = 0;
    int radar_y = 0;
    check(lidar_world.health.lidar, "fresh LiDAR scan reports healthy");
    check(lidar_world.occupancy.worldToCell(0.0f, 2.0f, lidar_x, lidar_y) &&
              lidar_world.occupancy.probability(lidar_x, lidar_y) > 0.55f,
          "LiDAR endpoint updates the primary map");
    check(lidar_world.occupancy.worldToCell(3.0f, 3.0f, radar_x, radar_y) &&
              near(lidar_world.occupancy.probability(radar_x, radar_y), 0.5f, 1.0e-4f),
          "healthy LiDAR prevents radar returns from entering the map");

    FusionEngine radar_fallback;
    frame.lidar.reset();
    const WorldModel fallback_world = radar_fallback.process(frame);
    check(!fallback_world.health.lidar, "missing LiDAR reports unhealthy");
    check(fallback_world.occupancy.worldToCell(3.0f, 3.0f, radar_x, radar_y) &&
              fallback_world.occupancy.probability(radar_x, radar_y) > 0.55f,
          "radar rays map only when LiDAR health is false");
}

void testFinalTofHardwareDefaults()
{
    const TofConfig config;
    check(config.i2c_device == "/dev/i2c-1", "ToF defaults to the locked I2C bus");
    check(config.left.xshut.chip == "/dev/gpiochip3" &&
              config.left.xshut.offset == 10 &&
              config.left.mount_yaw_deg == 30.0f,
          "left ToF defaults to its locked XSHUT and +30-degree mounting");
    check(config.right.xshut.chip == "/dev/gpiochip3" &&
              config.right.xshut.offset == 11 &&
              config.right.mount_yaw_deg == -30.0f,
          "right ToF defaults to its locked XSHUT and -30-degree mounting");
    check(config.left_i2c_address == 0x60 && config.right_i2c_address == 0x52,
          "ToF defaults to 7-bit addresses 0x30 and 0x29 in ST-encoded form");
}

void testTofOwnsMapping()
{
    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1000;
    RadarFrame radar;
    radar.timestamp_ms = 1000;
    radar.detections.push_back(makeDetection(3.0f, 3.0f, 0.2f));
    frame.radar = radar;
    TofScan tof;
    tof.timestamp_ms = 1000;
    tof.rays.push_back({{-0.05f, 0.0f, 0.0f}, {-0.05f, 2.0f, 0.0f}});
    frame.tof = tof;

    FusionEngine fusion;
    const WorldModel world = fusion.process(frame);
    int tof_x = 0;
    int tof_y = 0;
    int radar_x = 0;
    int radar_y = 0;
    check(world.health.tof, "fresh ToF data reports healthy");
    check(world.occupancy.worldToCell(-0.05f, 2.0f, tof_x, tof_y) &&
              world.occupancy.probability(tof_x, tof_y) > 0.55f,
          "ToF rays update the continuous surface map");
    check(world.occupancy.worldToCell(3.0f, 3.0f, radar_x, radar_y) &&
              near(world.occupancy.probability(radar_x, radar_y), 0.5f, 1.0e-4f),
          "healthy ToF mapping suppresses noisy radar fallback while LiDAR is off");
}

Track makeTestTrack(uint32_t id, float x, float y, PersonClass cls, bool visible = true)
{
    Track track{};
    track.id = id;
    const Detection detection = makeDetection(x, y);
    track.filter = KalmanTrack(detection, 1000);
    track.identity.tagged_safe = cls == PersonClass::SAFE_TAGGED;
    track.identity.tag_id = cls == PersonClass::SAFE_TAGGED ? 7 : 0;
    track.identity.confidence = cls == PersonClass::SAFE_TAGGED ? 0.95f : 0.0f;
    track.state = visible ? TrackState::VISIBLE : TrackState::PRESENCE_SUPPORTED;
    track.existence_confidence = 0.9f;
    track.position_confidence = 0.8f;
    track.position_sigma_m = 0.2f;
    track.visible = visible;
    return track;
}

void testAnnouncements()
{
    AnnounceConfig config;
    WorldModel world{};
    world.timestamp_ms = 1000;

    check(buildAnnouncement(world, config) == "Area clear.", "empty world announces clear");

    world.zones.push_back(PresenceZone{1.0f, 2.0f, 0.0f, 0.3f, 0.8f});
    check(buildAnnouncement(world, config) == "PRESENCE_ONLY detected; no person track.",
          "presence without a track is announced honestly");
    world.zones.clear();

    world.tracks.push_back(makeTestTrack(1, 0.0f, 5.0f, PersonClass::UNKNOWN_PERSON));
    check(buildAnnouncement(world, config) == "UNKNOWN_PERSON 5 meters ahead.",
          "single unknown announces type, distance and bearing");

    world.tracks.clear();
    world.tracks.push_back(makeTestTrack(1, 1.2f, 1.0f, PersonClass::SAFE_TAGGED));
    check(buildAnnouncement(world, config) == "SAFE_TAGGED 2 meters to the right.",
          "tagged friendly is announced with side");

    world.tracks.clear();
    world.tracks.push_back(makeTestTrack(1, 0.0f, 5.0f, PersonClass::UNKNOWN_PERSON));
    world.tracks.push_back(makeTestTrack(2, 0.0f, -1.0f, PersonClass::SAFE_TAGGED));
    const std::string summary = buildAnnouncement(world, config);
    check(summary.find("2 people:") != std::string::npos, "multiple people announce a count");
    check(summary.find("1 UNKNOWN_PERSON") != std::string::npos, "multiple people count unknown tracks");
    check(summary.find("1 SAFE_TAGGED") != std::string::npos, "multiple people count safe-tagged tracks");
    check(summary.find("SAFE_TAGGED 1 meter behind.") != std::string::npos,
          "nearest person is announced first with side");

    // A marker or static map object must never be spoken as a person.
    WorldModel marker_world{};
    marker_world.static_objects.push_back(MapObject{{0.0f, 2.0f, 0.0f}, 0.9f});
    check(buildAnnouncement(marker_world, config) == "Area clear.",
          "map objects are never announced as people");

    const std::string path = "sa_test_say.txt";
    std::remove(path.c_str());
    AnnounceConfig say;
    say.enabled = true;
    say.device_path = path;
    say.min_interval_s = 1.0;
    say.repeat_interval_s = 3.0;

    Announcer announcer(say);
    std::string error;
    check(announcer.start(error), "announcer opens its output: " + error);

    WorldModel scene{};
    scene.tracks.push_back(makeTestTrack(1, 0.0f, 2.0f, PersonClass::UNKNOWN_PERSON));
    scene.timestamp_ms = 1000;
    announcer.update(scene, 1000);
    check(announcer.lastText() == "UNKNOWN_PERSON 2 meters ahead.", "announcer composes the expected line");

    const auto sayCount = [&]() {
        std::ifstream file(path, std::ios::binary);
        std::ostringstream content;
        content << file.rdbuf();
        const std::string text = content.str();
        std::size_t count = 0;
        std::size_t position = 0;
        while ((position = text.find("SAY ", position)) != std::string::npos)
        {
            ++count;
            position += 4;
        }
        return count;
    };
    const auto waitForSayCount = [&](std::size_t expected) {
        for (int attempt = 0; attempt < 200; ++attempt)
        {
            if (sayCount() == expected) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    };

    check(waitForSayCount(1), "announcer asynchronously writes one SAY line");
    announcer.update(scene, 1500);
    check(sayCount() == 1, "announcer debounces an unchanged scene");
    announcer.update(scene, 4000);
    check(waitForSayCount(2), "announcer repeats an unchanged scene after the repeat interval");

    scene.tracks.clear();
    scene.tracks.push_back(makeTestTrack(2, 0.0f, 3.0f, PersonClass::UNKNOWN_PERSON));
    announcer.update(scene, 4300);
    check(sayCount() == 2, "announcer debounces a changed scene");
    announcer.update(scene, 5100);
    check(waitForSayCount(3), "announcer speaks a changed scene after the debounce");

    announcer.announceText("Red marker detected 10 degrees to the right.", 5200);
    check(sayCount() == 3, "custom alerts use the announcer change debounce");
    announcer.announceText("Red marker detected 10 degrees to the right.", 6100);
    check(waitForSayCount(4) && announcer.lastText() == "Red marker detected 10 degrees to the right.",
          "announcer emits a debounced custom alert");

    announcer.stop();
    std::remove(path.c_str());
}

void testSchematic()
{
    const std::string text =
        "# unit test plan\n"
        "origin 1 2\n"
        "yaw 90\n"
        "scale 2\n"
        "wall 0 0 1 0\n"
        "door 0 0 0 1\n"
        "room Kitchen 0 0 1 1\n"
        "label Ladder 0.5 0.5\n";

    Schematic schematic;
    std::string error;
    check(schematic.loadText(text, error), "schematic parses: " + error);
    check(schematic.walls().size() == 1, "schematic separates walls");
    check(schematic.doors().size() == 1, "schematic separates doors");
    check(schematic.rooms().size() == 1, "schematic parses rooms");
    check(schematic.marks().size() == 1, "schematic parses labels");

    const MapWall& wall = schematic.walls().front();
    check(near(wall.start.x, 1.0f, 1.0e-4f) && near(wall.start.y, 2.0f, 1.0e-4f),
          "schematic applies origin to wall start");
    check(near(wall.end.x, 1.0f, 1.0e-4f) && near(wall.end.y, 4.0f, 1.0e-4f),
          "schematic applies yaw and scale to wall end");

    const MapRoom& room = schematic.rooms().front();
    check(room.name == "Kitchen", "schematic keeps the room name");
    check(near(room.min_x, -1.0f, 1.0e-4f) && near(room.max_x, 1.0f, 1.0e-4f) &&
              near(room.min_y, 2.0f, 1.0e-4f) && near(room.max_y, 4.0f, 1.0e-4f),
          "schematic rotates room corners into an axis-aligned box");

    const SchematicMark& mark = schematic.marks().front();
    check(mark.name == "Ladder" && near(mark.position.x, 0.0f, 1.0e-4f) &&
              near(mark.position.y, 3.0f, 1.0e-4f),
          "schematic transforms labels into world coordinates");

    OccupancyGrid grid;
    int wall_cx = 0;
    int wall_cy = 0;
    int door_cx = 0;
    int door_cy = 0;
    check(grid.worldToCell(1.0f, 3.0f, wall_cx, wall_cy), "schematic wall is inside the grid");
    check(grid.worldToCell(0.0f, 2.0f, door_cx, door_cy), "schematic door is inside the grid");
    for (int i = 0; i < 5; ++i)
    {
        grid.decay(0.98f);
        schematic.applyToGrid(grid);
    }
    check(grid.probability(wall_cx, wall_cy) > 0.5f, "schematic wall prior survives repeated decay");
    check(grid.probability(door_cx, door_cy) < 0.5f, "schematic door prior carves free space");

    Schematic invalid;
    check(!invalid.loadText("bogus 1 2\n", error), "unknown schematic command is rejected");
    check(!invalid.loadText("wall 0 0 1\n", error), "incomplete wall is rejected");
    check(!invalid.loadText("# only comments\n", error), "empty schematic is rejected");
    check(!invalid.loadText("scale 0\nwall 0 0 1 1\n", error), "non-positive scale is rejected");
}

void testSchematicFusion()
{
    Schematic schematic;
    std::string error;
    check(schematic.loadText("wall -2 0 2 0\nroom Hall -2 -2 2 2\nlabel Door 1.5 0\n", error),
          "fusion schematic parses: " + error);

    FusionEngine fusion;
    fusion.setSchematic(schematic);
    SensorFrame frame;
    frame.timestamp_ms = 1000;
    const WorldModel world = fusion.process(frame);

    check(world.walls.size() == 1, "schematic walls reach the world model");
    check(world.rooms.size() == 1, "schematic rooms reach the world model");
    bool label_found = false;
    for (const auto& object : world.static_objects)
    {
        if (near(object.position.x, 1.5f, 1.0e-3f) && near(object.position.y, 0.0f, 1.0e-3f) &&
            near(object.confidence, 0.90f, 1.0e-3f))
            label_found = true;
    }
    check(label_found, "schematic labels become map objects");
    check(world.tracks.empty(), "schematic never creates person tracks");

    int cx = 0;
    int cy = 0;
    check(world.occupancy.worldToCell(0.0f, 0.0f, cx, cy), "schematic wall cell is in range");
    check(world.occupancy.probability(cx, cy) > 0.55f, "schematic wall is stamped into occupancy");
}

void testSchematicRecordingReplay()
{
    const std::string path = "sa_test_schematic_session.bin";
    std::remove(path.c_str());
    const std::string first = "wall -2 0 2 0\nroom Hall -2 -2 2 2\n";
    const std::string second = "wall -1 0 1 0\n";

    Recorder recorder;
    std::string error;
    check(recorder.open(path, error), "schematic session records: " + error);
    check(recorder.writeSchematicRecord(first), "initial schematic record writes");

    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1000;
    check(recorder.recordFrame(frame), "frame before schematic change records");
    check(recorder.writeSchematicRecord(second), "mid-session schematic record writes");
    frame.frame_id = 2;
    frame.timestamp_ms = 1200;
    check(recorder.recordFrame(frame), "frame after schematic change records");
    recorder.close();

    Playback playback;
    check(playback.open(path, error), "schematic session replays: " + error);
    check(playback.schematicText() == first, "playback exposes the preface schematic");

    PlaybackFrame playback_frame;
    check(playback.next(playback_frame), "first schematic playback frame arrives");
    check(playback_frame.schematic && *playback_frame.schematic == first,
          "preface schematic is delivered with the first frame");
    check(playback.next(playback_frame), "second schematic playback frame arrives");
    check(playback_frame.schematic && *playback_frame.schematic == second,
          "mid-session schematic is delivered in order");

    playback.close();
    std::remove(path.c_str());
}

void testLidarRecordingReplay()
{
    const std::string path = "sa_test_lidar_session.bin";
    std::remove(path.c_str());

    Recorder recorder;
    std::string error;
    check(recorder.open(path, error), "LiDAR session opens for recording: " + error);
    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1400;
    LidarScan scan;
    scan.timestamp_ms = 1395;
    scan.origin = {0.1f, 0.2f, 0.3f};
    scan.points = {{1.0f, 2.0f, 0.0f}, {-0.5f, 3.0f, 1.2f}};
    scan.burst_active = true;
    frame.lidar = scan;
    check(recorder.recordFrame(frame), "LiDAR scan records");
    SensorFrame completion_frame;
    completion_frame.frame_id = 2;
    completion_frame.timestamp_ms = 1500;
    LidarScan completion;
    completion.timestamp_ms = 1500;
    completion.burst_complete = true;
    completion_frame.lidar = completion;
    check(recorder.recordFrame(completion_frame), "LiDAR burst-completion event records");
    recorder.close();

    Playback playback;
    check(playback.open(path, error), "LiDAR session opens for replay: " + error);
    PlaybackFrame replay;
    check(playback.next(replay), "LiDAR replay frame arrives");
    check(replay.sensors.lidar.has_value(), "recorded LiDAR scan is restored");
    if (replay.sensors.lidar)
    {
        check(replay.sensors.lidar->timestamp_ms == scan.timestamp_ms &&
                  replay.sensors.lidar->points.size() == scan.points.size(),
              "LiDAR replay preserves timestamp and point count");
        check(near(replay.sensors.lidar->origin.x, scan.origin.x, 1.0e-5f) &&
                  near(replay.sensors.lidar->points[1].z, scan.points[1].z, 1.0e-5f),
              "LiDAR replay preserves origin and point coordinates");
        check(replay.sensors.lidar->burst_active, "LiDAR replay preserves active-burst state");
    }
    check(playback.next(replay) && replay.sensors.lidar && replay.sensors.lidar->burst_complete,
          "LiDAR replay preserves the burst-completion event");

    playback.close();
    std::remove(path.c_str());
}

void testTofRecordingReplay()
{
    const std::string path = "sa_test_tof_session.bin";
    std::remove(path.c_str());

    SensorFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ms = 1500;
    TofScan scan;
    scan.timestamp_ms = 1495;
    scan.rays.push_back({{0.1f, 0.0f, 0.2f}, {0.4f, 1.8f, 0.7f}});
    frame.tof = scan;
    UwbMeasurement anchor;
    anchor.timestamp_ms = 1490;
    anchor.tag_id = 19;
    anchor.range_m = 2.2f;
    anchor.confidence = 0.85f;
    anchor.is_fixed_anchor = true;
    anchor.anchor_x_m = -2.0f;
    anchor.anchor_y_m = 3.0f;
    frame.uwb.push_back(anchor);

    FusionEngine direct_fusion;
    const WorldModel direct_world = direct_fusion.process(frame);
    Recorder recorder;
    std::string error;
    check(recorder.open(path, error), "ToF session opens for recording: " + error);
    check(recorder.recordFrame(frame, &direct_world), "ToF scan and world snapshot record");
    recorder.close();

    Playback playback;
    check(playback.open(path, error), "ToF session opens for replay: " + error);
    PlaybackFrame replay;
    check(playback.next(replay), "ToF replay frame arrives");
    check(replay.sensors.tof.has_value(), "recorded ToF scan is restored");
    if (replay.sensors.tof)
    {
        check(replay.sensors.tof->timestamp_ms == scan.timestamp_ms &&
                  replay.sensors.tof->rays.size() == scan.rays.size(),
              "ToF replay preserves timestamp and ray count");
        check(near(replay.sensors.tof->rays.front().endpoint.z, scan.rays.front().endpoint.z, 1.0e-5f),
              "ToF replay preserves 3D endpoint coordinates");
    }
    check(replay.sensors.uwb.size() == 1 && replay.sensors.uwb.front().is_fixed_anchor &&
              near(replay.sensors.uwb.front().anchor_x_m, anchor.anchor_x_m, 1.0e-5f) &&
              near(replay.sensors.uwb.front().anchor_y_m, anchor.anchor_y_m, 1.0e-5f),
          "replay preserves the fixed-anchor flag and known map coordinates");
    FusionEngine replay_fusion;
    const WorldModel replay_world = replay_fusion.process(replay.sensors);
    check(gridHash(direct_world) == gridHash(replay_world), "ToF replay regenerates the same occupancy map");
    playback.close();
    std::remove(path.c_str());
}

void testWallpadRecordingReplay()
{
    const std::string path = "sa_test_wallpad_session.bin";
    std::remove(path.c_str());

    SensorFrame frame;
    frame.frame_id = 4;
    frame.timestamp_ms = 2700;
    frame.wallpad_zones.push_back({1.1f, 4.2f, degToRad(-30.0f), degToRad(55.0f), 0.73f});
    FusionEngine fusion;
    const WorldModel direct_world = fusion.process(frame);

    RecorderConfig recorder_config;
    recorder_config.write_world_snapshots = false;
    recorder_config.write_track_snapshots = false;
    Recorder recorder(recorder_config);
    std::string error;
    check(recorder.open(path, error), "wall-pad session opens for recording: " + error);
    check(recorder.recordFrame(frame, &direct_world), "wall-pad sector is recorded as a replay input");
    recorder.close();

    Playback playback;
    check(playback.open(path, error), "wall-pad session opens for replay: " + error);
    PlaybackFrame replay;
    check(playback.next(replay), "wall-pad replay frame arrives");
    check(replay.sensors.timestamp_ms == frame.timestamp_ms &&
              replay.sensors.wallpad_zones.size() == 1,
          "wall-pad replay preserves frame timestamp and sector count");
    if (!replay.sensors.wallpad_zones.empty())
    {
        const PresenceZone& zone = replay.sensors.wallpad_zones.front();
        check(near(zone.range_min, 1.1f, 1.0e-5f) &&
                  near(zone.range_max, 4.2f, 1.0e-5f) &&
                  near(zone.bearing_center_rad, degToRad(-30.0f), 1.0e-5f) &&
                  near(zone.bearing_width_rad, degToRad(55.0f), 1.0e-5f) &&
                  near(zone.confidence, 0.73f, 1.0e-5f),
              "wall-pad replay preserves sector geometry and confidence");
    }
    FusionEngine replay_fusion;
    const WorldModel replay_world = replay_fusion.process(replay.sensors);
    check(replay_world.zones.size() == direct_world.zones.size(),
          "replay regenerates the wall-pad presence zone");
    playback.close();
    std::remove(path.c_str());
}

void testWristOrientationRecordingReplay()
{
    const std::string path = "sa_test_wrist_orientation_session.bin";
    std::remove(path.c_str());

    SensorFrame frame;
    frame.frame_id = 5;
    frame.timestamp_ms = 2800;
    frame.wrist_orientation = WristOrientation{2795, 37.5f, -12.0f};
    frame.wrist_orientation->glove_packet = true;
    frame.wrist_orientation->pointing_active = true;
    frame.wrist_orientation->zero_heading_requested = true;
    frame.wrist_orientation->glove_events = 0x0C;
    RecorderConfig recorder_config;
    recorder_config.write_world_snapshots = false;
    recorder_config.write_track_snapshots = false;
    Recorder recorder(recorder_config);
    std::string error;
    check(recorder.open(path, error), "wrist session opens for recording: " + error);
    check(recorder.recordFrame(frame), "wrist orientation is recorded as a replay input");
    recorder.close();

    Playback playback;
    check(playback.open(path, error), "wrist session opens for replay: " + error);
    PlaybackFrame replay;
    check(playback.next(replay) && replay.sensors.wrist_orientation.has_value(),
          "wrist orientation replay record arrives");
    if (replay.sensors.wrist_orientation)
    {
        const WristOrientation& orientation = *replay.sensors.wrist_orientation;
        check(orientation.timestamp_ms == 2795 &&
                  near(orientation.wrist_yaw_deg, 37.5f, 1.0e-5f) &&
                  near(orientation.wrist_pitch_deg, -12.0f, 1.0e-5f),
              "wrist orientation replay preserves timestamp, yaw, and pitch");
        check(orientation.glove_packet && orientation.pointing_active &&
                  orientation.zero_heading_requested && orientation.glove_events == 0x0C,
              "glove trigger and zero-heading flags are preserved through replay");
    }
    playback.close();
    std::remove(path.c_str());
}

void testLegacySessionRejected()
{
    const std::string path = "sa_test_v1_session.bin";
    std::remove(path.c_str());

    Recorder recorder;
    std::string error;
    check(recorder.open(path, error), "legacy-version fixture is created: " + error);
    recorder.close();

    std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
    file.seekp(8, std::ios::beg);
    const char version_v1[4] = {1, 0, 0, 0};
    file.write(version_v1, sizeof(version_v1));
    file.close();

    Playback playback;
    check(!playback.open(path, error), "v1 sessions are rejected");
    check(error.find("v1") != std::string::npos && error.find("legacy coordinate frame") != std::string::npos,
          "v1 rejection explains the coordinate-frame incompatibility");
    playback.close();

    std::fstream version2_file(path, std::ios::binary | std::ios::in | std::ios::out);
    version2_file.seekp(8, std::ios::beg);
    const char version_v2[4] = {2, 0, 0, 0};
    version2_file.write(version_v2, sizeof(version_v2));
    version2_file.close();
    check(!playback.open(path, error), "v2 sessions are rejected after LiDAR format changes");
    check(error.find("v2") != std::string::npos && error.find("LiDAR records") != std::string::npos,
          "v2 rejection explains the LiDAR format incompatibility");
    playback.close();

    std::fstream version3_file(path, std::ios::binary | std::ios::in | std::ios::out);
    version3_file.seekp(8, std::ios::beg);
    const char version_v3[4] = {3, 0, 0, 0};
    version3_file.write(version_v3, sizeof(version_v3));
    version3_file.close();
    check(!playback.open(path, error), "v3 sessions are rejected after ToF records are added");
    check(error.find("v3") != std::string::npos && error.find("ToF records") != std::string::npos,
          "v3 rejection explains the ToF format incompatibility");
    playback.close();

    std::fstream version4_file(path, std::ios::binary | std::ios::in | std::ios::out);
    version4_file.seekp(8, std::ios::beg);
    const char version_v4[4] = {4, 0, 0, 0};
    version4_file.write(version_v4, sizeof(version_v4));
    version4_file.close();
    check(!playback.open(path, error), "v4 sessions are rejected after burst and anchor fields are added");
    check(error.find("v4") != std::string::npos && error.find("fixed-anchor fields") != std::string::npos,
          "v4 rejection explains the localization record incompatibility");
    playback.close();

    std::fstream version5_file(path, std::ios::binary | std::ios::in | std::ios::out);
    version5_file.seekp(8, std::ios::beg);
    const char version_v5[4] = {5, 0, 0, 0};
    version5_file.write(version_v5, sizeof(version_v5));
    version5_file.close();
    check(!playback.open(path, error), "v5 sessions are rejected after the occupancy-grid expansion");
    check(error.find("v5") != std::string::npos && error.find("occupancy-grid") != std::string::npos,
          "v5 rejection explains the map-profile incompatibility");
    playback.close();

    std::fstream version6_file(path, std::ios::binary | std::ios::in | std::ios::out);
    version6_file.seekp(8, std::ios::beg);
    const char version_v6[4] = {6, 0, 0, 0};
    version6_file.write(version_v6, sizeof(version_v6));
    version6_file.close();
    check(!playback.open(path, error), "v6 sessions are rejected after wall-pad input is added");
    check(error.find("v6") != std::string::npos && error.find("wall-pad") != std::string::npos,
          "v6 rejection explains the new raw input record");
    playback.close();

    std::fstream version7_file(path, std::ios::binary | std::ios::in | std::ios::out);
    version7_file.seekp(8, std::ios::beg);
    const char version_v7[4] = {7, 0, 0, 0};
    version7_file.write(version_v7, sizeof(version_v7));
    version7_file.close();
    check(!playback.open(path, error), "v7 sessions are rejected after wrist orientation is recorded");
    check(error.find("v7") != std::string::npos && error.find("wrist orientation") != std::string::npos,
          "v7 rejection explains the new wrist input record");
    playback.close();
    std::remove(path.c_str());
}

bool sameWorld(const WorldModel& a, const WorldModel& b)
{
    if (a.timestamp_ms != b.timestamp_ms) return false;
    if (!near(a.pose.x, b.pose.x, 1.0e-4f) || !near(a.pose.y, b.pose.y, 1.0e-4f) ||
        !near(a.pose.yaw_deg, b.pose.yaw_deg, 1.0e-3f))
        return false;
    if (!near(a.environment_confidence, b.environment_confidence, 1.0e-4f)) return false;
    if (a.tracks.size() != b.tracks.size()) return false;
    for (std::size_t i = 0; i < a.tracks.size(); ++i)
    {
        const Track& ta = a.tracks[i];
        const Track& tb = b.tracks[i];
        if (ta.id != tb.id || ta.identity.tagged_safe != tb.identity.tagged_safe ||
            ta.identity.tag_id != tb.identity.tag_id || ta.state != tb.state || ta.visible != tb.visible)
            return false;
        const Vec3 pa = ta.filter.position();
        const Vec3 pb = tb.filter.position();
        const Vec3 va = ta.filter.velocity();
        const Vec3 vb = tb.filter.velocity();
        if (!near(pa.x, pb.x, 1.0e-4f) || !near(pa.y, pb.y, 1.0e-4f) || !near(pa.z, pb.z, 1.0e-4f))
            return false;
        if (!near(va.x, vb.x, 1.0e-4f) || !near(va.y, vb.y, 1.0e-4f) || !near(va.z, vb.z, 1.0e-4f))
            return false;
        if (!near(ta.existence_confidence, tb.existence_confidence, 1.0e-4f)) return false;
    }
    return gridHash(a) == gridHash(b);
}

void testSimulationDeterminismAndReplay()
{
    SimulationConfig config;
    config.seed = 0x5A17u;
    config.duration_s = 12.0;
    config.frame_period_s = 0.05;

    const std::string path = "sa_test_session.bin";
    std::remove(path.c_str());

    Simulation simulation(config);
    FusionEngine direct_engine;
    Recorder recorder;
    std::string error;
    check(recorder.open(path, error), "recorder opens test session: " + error);

    SensorFrame frame;
    WorldModel direct_last{};
    uint64_t direct_frames = 0;
    uint64_t direct_markers = 0;
    while (simulation.next(frame))
    {
        WorldModel world = direct_engine.process(frame);
        direct_last = world;
        ++direct_frames;
        if (frame.marker) ++direct_markers;
        check(recorder.recordFrame(frame, &world), "recordFrame succeeds");
    }
    recorder.close();
    check(recorder.framesWritten() == direct_frames, "recorded frame count matches simulation");
    check(direct_markers > 0, "simulation emits red marker observations");

    Playback playback;
    check(playback.open(path, error), "playback opens recorded session: " + error);
    FusionEngine replay_engine;
    PlaybackFrame playback_frame;
    WorldModel replay_last{};
    uint64_t compared = 0;
    uint64_t mismatches = 0;
    uint64_t playback_markers = 0;
    while (playback.next(playback_frame))
    {
        WorldModel world = replay_engine.process(playback_frame.sensors);
        replay_last = world;
        if (playback_frame.sensors.marker) ++playback_markers;
        if (playback_frame.reference_world)
        {
            ++compared;
            if (!sameWorld(*playback_frame.reference_world, world)) ++mismatches;
        }
    }
    check(compared > 0, "session contains reference world snapshots");
    check(mismatches == 0, "replay regenerates every reference world snapshot exactly");
    check(playback_markers == direct_markers, "marker observations survive recording and replay");
    check(sameWorld(direct_last, replay_last), "replay final world matches direct run");
    check(playback.stats().crc_errors == 0, "session has no CRC errors");
    check(!playback.stats().truncated, "session is not truncated");

    std::string rewind_error;
    check(playback.rewind(rewind_error), "playback can rewind");
    check(playback.next(playback_frame), "playback yields a frame after rewind");

    playback.close();
    std::remove(path.c_str());
}

} // namespace

int main()
{
    testOccupancyGrid();
    testRearRadarRotation();
    testKalmanTrack();
    testTrackManagerIdentityPersistence();
    testPresenceSupportRules();
    testWallpadPresenceRules();
    testUwbOnlyCreatesNoTracks();
    testNoveldaOnlyCreatesNoTracks();
    testImuStationaryZupt();
    testCorrelativeTofScanMatching();
    testLongRangeScanMatching();
    testBurstLoopClosure();
    testUwbAnchorCorrection();
    testUwbAssociation();
    testPresenceFusionNeedsOverlap();
    testRedMarkerDetector();
    testWristGuidanceHysteresis();
    testWristVibratorOutput();
    testWristOrientationParserAndPointing();
    testSensorSelection();
    testLidarPointParser();
    testWallpadZoneParser();
    testUltrasonicCarriageReturnBridge();
    testLd19PacketParser();
    testLidarBurstState();
    testFinalTofHardwareDefaults();
    testLidarOwnsMapping();
    testTofOwnsMapping();
    testAnnouncements();
    testSchematic();
    testSchematicFusion();
    testSchematicRecordingReplay();
    testLidarRecordingReplay();
    testTofRecordingReplay();
    testWallpadRecordingReplay();
    testWristOrientationRecordingReplay();
    testLegacySessionRejected();
    testSimulationDeterminismAndReplay();

    if (g_failures != 0)
    {
        std::cerr << g_failures << " test failure(s)\n";
        return 1;
    }
    std::cout << "all core tests passed\n";
    return 0;
}
