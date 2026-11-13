#include "announcer.hpp"
#include "fusion_engine.hpp"
#include "marker_detector.hpp"
#include "occupancy_grid.hpp"
#include "playback.hpp"
#include "recorder.hpp"
#include "schematic.hpp"
#include "sensor_mode.hpp"
#include "serialization.hpp"
#include "simulation.hpp"
#include "track_manager.hpp"
#include "types.hpp"
#include "wrist_haptics.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
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
    int cx = 0;
    int cy = 0;
    check(grid.worldToCell(1.0f, 1.0f, cx, cy), "worldToCell in range");
    grid.setProbability(cx, cy, 0.9f);
    grid.decay(0.5f);
    check(near(grid.probability(cx, cy), 0.7f, 1.0e-5f), "mandatory decay pulls probability toward 0.5");

    OccupancyGrid ray_grid;
    check(ray_grid.integrateRay({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, 0.85f, -0.35f, 20.0f),
          "integrateRay accepts a valid ray");
    int ex = 0;
    int ey = 0;
    ray_grid.worldToCell(1.0f, 0.0f, ex, ey);

    OccupancyGrid stamp_grid;
    stamp_grid.markOccupiedWorld({1.0f, 0.0f, 0.0f}, 0.85f);

    check(near(ray_grid.probability(ex, ey), stamp_grid.probability(ex, ey), 1.0e-6f),
          "ray endpoint receives occupied evidence exactly once");

    int free_cx = 0;
    int free_cy = 0;
    check(ray_grid.worldToCell(0.25f, 0.0f, free_cx, free_cy), "free-space sample cell is in range");
    check(ray_grid.probability(free_cx, free_cy) < 0.5f, "free-space traversal lowers occupancy");
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
    std::vector<Detection> detections{makeDetection(2.0f, 0.0f)};
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
    std::vector<Detection> detections{makeDetection(2.0f, 0.0f)};
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

void testUwbAssociation()
{
    FusionEngine fusion;

    SensorFrame radar_frame;
    radar_frame.frame_id = 1;
    radar_frame.timestamp_ms = 1000;
    RadarFrame radar;
    radar.timestamp_ms = 1000;
    const auto person = makePersonCluster(2.0f, 0.0f);
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
    const auto person = makePersonCluster(2.0f, 0.0f);
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
    check(left.bearing_rad > 0.03f, "left-of-axis marker has positive bearing");
    check(left.confidence >= 0.35f, "red marker confidence is usable");
    check(left.pixel_count >= 24, "red marker cluster is large enough");

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

    WristConfig config;
    config.device_path = path;
    WristVibrator vibrator(config);
    std::string error;
    check(vibrator.start(error), "wrist actuator opens its output: " + error);

    vibrator.update(1000, true);
    check(vibrator.outputOn(), "wrist actuator reports active output");
    check(readFile() == "1", "wrist writes active value while aimed");

    vibrator.update(1050, false);
    check(!vibrator.outputOn(), "wrist actuator reports inactive output");
    check(readFile() == "0", "wrist writes inactive value when aim is lost");

    vibrator.stop();
    std::remove(path.c_str());
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

    check(parseSensorSelection("imu,radar, UWB ,marker", selection, error), "custom sensor list parses");
    check(selection.has(SensorKind::Imu) && selection.has(SensorKind::Radar) &&
              selection.has(SensorKind::Uwb) && selection.has(SensorKind::Marker),
          "custom list enables each named sensor");
    check(sensorSelectionName(selection) == "imu+radar+uwb+marker", "custom list has a stable name");

    check(!parseSensorSelection("lidar", selection, error), "unknown sensor is rejected");
    check(!parseSensorSelection("", selection, error), "empty sensor selection is rejected");

    SensorFrame frame;
    frame.frame_id = 7;
    frame.timestamp_ms = 4200;
    frame.imu = ImuSample{};
    RadarFrame radar;
    radar.detections.push_back(makeDetection(2.0f, 0.0f));
    frame.radar = radar;
    frame.uwb.push_back(UwbMeasurement{});
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
    check(filtered.frame_id == frame.frame_id && filtered.timestamp_ms == frame.timestamp_ms,
          "filter preserves frame id and timestamp");
    check(frame.uwb.size() == 1 && frame.marker.has_value(), "filter leaves the raw frame untouched");
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
    check(buildAnnouncement(world, config) == "Presence detected, no tracked people.",
          "presence without a track is announced honestly");
    world.zones.clear();

    world.tracks.push_back(makeTestTrack(1, 5.0f, 0.0f, PersonClass::UNKNOWN_PERSON));
    check(buildAnnouncement(world, config) == "Unit 5 meters ahead.",
          "single unknown announces type, distance and bearing");

    world.tracks.clear();
    world.tracks.push_back(makeTestTrack(1, 1.0f, -1.2f, PersonClass::SAFE_TAGGED));
    check(buildAnnouncement(world, config) == "Friendly 2 meters to the right.",
          "tagged friendly is announced with side");

    world.tracks.clear();
    world.tracks.push_back(makeTestTrack(1, 5.0f, 0.0f, PersonClass::UNKNOWN_PERSON));
    world.tracks.push_back(makeTestTrack(2, -1.0f, 0.0f, PersonClass::SAFE_TAGGED));
    const std::string summary = buildAnnouncement(world, config);
    check(summary.find("2 people:") != std::string::npos, "multiple people announce a count");
    check(summary.find("1 unit") != std::string::npos, "multiple people count units");
    check(summary.find("1 friendly") != std::string::npos, "multiple people count friendlies");
    check(summary.find("Friendly 1 meter behind.") != std::string::npos,
          "nearest person is announced first with side");

    // A marker or static map object must never be spoken as a person.
    WorldModel marker_world{};
    marker_world.static_objects.push_back(MapObject{{2.0f, 0.0f, 0.0f}, 0.9f});
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
    scene.tracks.push_back(makeTestTrack(1, 2.0f, 0.0f, PersonClass::UNKNOWN_PERSON));
    scene.timestamp_ms = 1000;
    announcer.update(scene, 1000);
    check(announcer.lastText() == "Unit 2 meters ahead.", "announcer composes the expected line");

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

    check(sayCount() == 1, "announcer writes one SAY line");
    announcer.update(scene, 1500);
    check(sayCount() == 1, "announcer debounces an unchanged scene");
    announcer.update(scene, 4000);
    check(sayCount() == 2, "announcer repeats an unchanged scene after the repeat interval");

    scene.tracks.clear();
    scene.tracks.push_back(makeTestTrack(2, 3.0f, 0.0f, PersonClass::UNKNOWN_PERSON));
    announcer.update(scene, 4300);
    check(sayCount() == 2, "announcer debounces a changed scene");
    announcer.update(scene, 5100);
    check(sayCount() == 3, "announcer speaks a changed scene after the debounce");

    announcer.announceText("Red marker detected 10 degrees to the left.", 5200);
    check(sayCount() == 3, "custom alerts use the announcer change debounce");
    announcer.announceText("Red marker detected 10 degrees to the left.", 6100);
    check(sayCount() == 4 && announcer.lastText() == "Red marker detected 10 degrees to the left.",
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
    testKalmanTrack();
    testTrackManagerIdentityPersistence();
    testPresenceSupportRules();
    testUwbOnlyCreatesNoTracks();
    testNoveldaOnlyCreatesNoTracks();
    testUwbAssociation();
    testPresenceFusionNeedsOverlap();
    testRedMarkerDetector();
    testWristGuidanceHysteresis();
    testWristVibratorOutput();
    testSensorSelection();
    testAnnouncements();
    testSchematic();
    testSchematicFusion();
    testSchematicRecordingReplay();
    testSimulationDeterminismAndReplay();

    if (g_failures != 0)
    {
        std::cerr << g_failures << " test failure(s)\n";
        return 1;
    }
    std::cout << "all core tests passed\n";
    return 0;
}
