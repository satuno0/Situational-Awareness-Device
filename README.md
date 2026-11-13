# SA Daemon

A dependency-free C++20 situational-awareness daemon for a handheld assistive
device. It fuses a mmWave radar, IMU, UWB ranging, Novelda/XeThru presence
sensing, and a MaxBotix ultrasonic ranger into a world model with:

- 6-state Kalman person tracks (position and velocity)
- obstacle and free-space occupancy mapping
- safe-tag identification via UWB ranging (confirmation only)
- occluded-track reinforcement via Novelda presence (confirmation only)
- haptic feedback for collisions, tagged/untagged people, lost tracks, and
  presence behind a barrier
- wrist vibration when the camera is aimed at a red marker
- optional spoken summaries of how many people are nearby, what they are, and
  which direction they are in
- switchable sensor/radar modes, including a mixed mode that combines every
  available input
- operator-supplied text schematics merged with sensor mapping
- JSON telemetry and a phone-friendly live map with schematic upload
- deterministic recording and replay of the raw sensor stream

The reference targets are Radxa Zero 3W and CM5 class Linux SBCs. Simulation,
replay, tests, and the telemetry server also build and run on Windows.

## Safety and privacy scope

This project is deliberately limited to **situational awareness**:

- It does **not** perform threat assessment.
- It does **not** infer emotion, intent, or identity.
- It does **not** recognize faces or track the identity of untagged people.
- The only classifications are `SAFE_TAGGED`, `UNKNOWN_PERSON`, and
  `PRESENCE_ONLY`.
- UWB is never the primary tracker. Novelda is never allowed to create a
  track. Both can only confirm a track that radar already established.
- Through-concrete tracking and through-wall imaging are **not claimed**.
  Novelda results behind a barrier are treated as lower-confidence presence
  evidence, not as an image.

This is an assistive device, not a certified safety instrument. Do not rely on
it as the sole means of avoiding injury.

## Requirements

- C++20 compiler (GCC 11+, Clang 13+, or MSVC 19.30+)
- CMake 3.20+ (optional; direct compiler commands are documented below)
- POSIX threads on Linux
- No third-party libraries

## Repository layout

```
include/                 Public interfaces and core data types
  math.hpp               Vec3, Quaternion, Pose2D, angles, filters
  types.hpp              Detection, Track, WorldModel, sensor frames
  kalman_track.hpp       6-state constant-velocity Kalman filter
  track_manager.hpp      Association, lifecycle, identity persistence
  occupancy_grid.hpp     Bayesian 160 x 160 grid and map objects
  slam_manager.hpp       AHRS pose integration and radar scan correction
  fusion_engine.hpp      Sensor fusion pipeline and confidence engine
  radar_driver.hpp       TI IWR6843AOP mmWave packet driver
  imu_driver.hpp         ICM-42688 I2C driver + attitude estimator
  uwb_driver.hpp         DWM3000 serial-bridge driver
  novelda_driver.hpp     XeThru/Novelda serial-bridge driver
  ultrasonic_driver.hpp  MaxBotix UART driver
  telemetry.hpp          Loopback HTTP/JSON server
  recorder.hpp           Versioned binary session writer
  playback.hpp           Session reader and replay source
  haptics.hpp            Haptic pattern scheduler
  marker_detector.hpp    Red-marker RGB detector and bearing estimator
  marker_driver.hpp      MARK/RED camera-bridge serial protocol
  wrist_haptics.hpp      Wrist aim hysteresis and actuator output
  announcer.hpp          Deterministic spoken track summaries
  sensor_mode.hpp        Sensor preset/list parsing and frame filtering
  schematic.hpp          Text/DXF floor-plan parser and occupancy prior
  simulation.hpp         Deterministic synthetic sensor source
  serial_port.hpp        Cross-platform serial port abstraction
  serialization.hpp      Binary encoding helpers and CRC32
src/                     Implementations of every header above, plus main.cpp
tests/test_core.cpp      Behavioral and replay-determinism tests
tests/test_dxf.cpp       DXF schematic parser test
web/index.html           Phone-friendly live visualization
maps/example_sa.txt      Example text schematic
CMakeLists.txt           Library, daemon, tests, and CTest definitions
```

`main.cpp` only wires modes and lifecycle; each subsystem has its own
translation unit.

## Build

### CMake

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

This produces `build/sa_daemon` (`build/sa_daemon.exe` on Windows) and
the test executables. CTest runs the core tests, DXF parser test, and a
simulate/record/replay verification pair.

### Direct compiler commands

Linux/macOS (bash):

```sh
g++ -std=c++20 -O2 -Wall -Wextra -Iinclude src/*.cpp -o sa_daemon -pthread

# Tests exclude main.cpp
g++ -std=c++20 -O2 -Wall -Wextra -Iinclude \
    $(find src -name '*.cpp' ! -name 'main.cpp') \
    tests/test_core.cpp -o sa_tests -pthread
```

Windows (PowerShell):

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The Windows build links Media Foundation and SAPI for the optional desktop
webcam/TTS test mode.

## Running

### Simulation

The simulation injects synthetic measurements only. It never writes tracks or
world state directly, so it exercises the same fusion path as hardware.

```sh
./sa_daemon --simulate --duration 10 --record=build/demo.bin \
    --no-telemetry --no-haptics
```

Defaults: 20 Hz simulated sensor cadence, seed `0x5A17`, duration 60 s. Use
`--seed` to choose another deterministic scenario. By default the simulation
runs as fast as the CPU; add `--realtime` to pace it to wall-clock time.

With live telemetry:

```sh
./sa_daemon --simulate --realtime --duration 60 --port 8080
# Open http://127.0.0.1:8080
```

### Replay

```sh
./sa_daemon --replay=build/demo.bin --verify --no-telemetry --no-haptics
```

Replay reads the recorded **raw sensor records** and reruns the entire fusion
pipeline: `TrackManager`, `SlamManager`, `FusionEngine`, `WorldModel`, and
telemetry. Recorded track/world snapshots are debug references only. They are
never used to synthesize outputs. An embedded schematic timeline is replayed in
order, including mid-session map uploads.

`--verify` compares regenerated world and track state against those reference
snapshots. A successful verification reports zero failures, zero CRC errors,
and no truncation. The comparison is meaningful only when replay uses the same
inputs as the recording: it is skipped (with a warning) if `--sensors` filters
the stream or if `--map` overrides the embedded schematic.

### Hardware

Hardware is the default mode. Each sensor is optional; the daemon runs with
whatever opens successfully.

```sh
sudo ./sa_daemon \
    --radar-data /dev/ttyUSB0 \
    --radar-cli /dev/ttyUSB1 \
    --radar-cfg /path/to/iwr6843_handheld.cfg \
    --imu /dev/i2c-1 \
    --uwb /dev/ttyACM0 \
    --novelda /dev/ttyACM1 \
    --ultrasonic /dev/ttyUSB2 \
    --record=session.bin \
    --bind 0.0.0.0 --port 8080
```

Exact device paths depend on the host and udev rules. On a Radxa Zero 3W the
IWR6843 usually exposes two USB CDC ports; use the higher-throughput one for
data and the other for the CLI.

### CLI reference

```
Modes:
  (default)                  Hardware sensors
  --simulate                 Deterministic synthetic sensors
  --replay <session.bin>     Re-run fusion from a recorded session

Sessions:
  --record[=path]            Record raw sensors (default session.bin)
  --duration <seconds>       Stop after N seconds
  --verify                   Compare replay output with reference snapshots
  --seed <uint32>            Simulation RNG seed
  --realtime                 Pace simulation to wall-clock time
  --rate <hz>                Hardware processing rate (default 20)

Telemetry:
  --port <port>              HTTP port; 0 selects an ephemeral port (default 8080)
  --bind <address>           Bind address (default 127.0.0.1; 0.0.0.0 for phone)
  --web-root <dir>           Directory containing index.html (default web)
  --no-telemetry             Disable the HTTP server

Haptics:
  --no-haptics               Disable the haptic worker
  --haptic-device <path>     Write motor state (1/0) to a device or file
  --verbose                  Log haptic events and progress

Red marker / wrist:
  --no-wrist                 Disable wrist guidance
  --wrist-device <path>      Write wrist motor state (1/0) to a device or file
  --marker <dev>             Camera bridge (MARK/RED lines) serial port
  --marker-offset <deg>      Camera yaw offset relative to device forward
  --marker-engage <deg>      Aim cone that starts vibration (default 8)
  --marker-release <deg>     Aim cone that stops vibration (default 14)
  --no-marker-sim            Disable the synthetic marker in simulation

Perception / audio / map:
  --sensors <mode>           mixed, radar, uwb, novelda, ultrasonic, vision,
                             imu, or a comma list such as radar,uwb
  --announce                 Speak track summaries
  --announce-device <path>   Speaker bridge (writes SAY <text> lines)
  --announce-interval <s>    Debounce for changed scenes (default 1.5)
  --announce-repeat <s>      Repeat unchanged summaries (default 8)
  --map <file>               Load a text or ASCII DXF floor plan as a prior

Hardware:
  --radar-data <dev>         IWR6843 data port
  --radar-cli <dev>          IWR6843 CLI port for .cfg upload
  --radar-cfg <file>         mmWave .cfg file
  --imu <dev>                ICM-42688 I2C device (default /dev/i2c-1)
  --uwb <dev>                DWM3000 bridge serial port
  --novelda <dev>            XeThru bridge serial port
  --ultrasonic <dev>         MaxBotix serial port
```

`--simulate` and `--replay` are mutually exclusive. Recording during replay is
rejected because replay is already a consumer of a session.

## Perception pipeline

For every sensor frame, `FusionEngine::process()` performs the following in a
fixed order:

1. **IMU and attitude.** The ICM-42688 sample feeds a complementary/Mahony
   attitude estimator. Without a magnetometer, yaw is gyro-integrated and is
   corrected by radar scan matching; it is not treated as absolute heading.
2. **Radar mapping and ego motion.** IWR6843 point clouds are transformed into
   the world frame. `SlamManager` estimates ego velocity from static radar
   clutter with deterministic RANSAC over bearing pairs, integrates
   translation once per radar frame, and applies scan correction only when the
   device is moving and the map is mature.
3. **Person-candidate selection.** The full cloud feeds the occupancy map, but
   only compact torso-height clusters may create or update person tracks. The
   default gates are: cluster radius 0.36 m, 2-6 points, centroid z between
   0.78 m and 1.70 m, plan-view extent at most 0.55 m, vertical span at least
   0.015 m, and at least 0.40 m clearance from static structure. Flat, wide,
   or wall-adjacent returns stay map-only.
4. **Tracking.** `TrackManager` predicts, applies deterministic gated
   association (1.40 m / 2.5 sigma), updates the 6-state Kalman filter, spawns
   new tracks, merges duplicates within 0.30 m, damps velocity while coasting,
   and expires stale tracks.
5. **UWB identity confirmation.** For each existing track, the predicted range,
   bearing, and range rate are compared to each measurement. The association
   score is the required sum of normalized errors:
   `range_error + bearing_error + velocity_error`, where each term is divided
   by its configured sigma. The lowest valid score wins. A tag can own only one
   track. UWB never creates a track.
6. **Novelda presence confirmation.** Each presence report becomes a timed
   zone. A zone can only reinforce an already-occluded track whose uncertainty
   overlaps the zone in range and bearing. Novelda never creates a track.
7. **Mapping.** The occupancy grid uses Bayesian log-odds ray integration.
   Every frame applies the mandatory probabilistic decay
   `p <- 0.5 + (p - 0.5) * factor`, default factor 0.999. Detections within
   0.70 m of a person track and map objects near tracks are excluded so people
   do not become furniture.
8. **World assembly and confidence.** The output `WorldModel` contains pose,
   health, grid, map objects, tracks, and presence zones. Environment
   confidence combines localization, map coverage/knowledge, and sensor
   health.

Identity is intentionally persistent: tagged tracks use a 90 s expiry ceiling
and a 25 s existence time constant, versus 10 s and 3.2 s for untagged tracks.
Identity is not aggressively decayed.

## Haptics

Patterns are produced by a worker thread at a 10 ms tick. Collision feedback
has priority over event patterns.

| Condition | Pattern |
| --- | --- |
| Obstacle 1.5 m away | Slow pulse, easing to faster/longer pulses |
| Obstacle at or inside 0.25 m | Continuous vibration |
| Safe tag becomes visible | Double pulse: 80 ms on, 80 ms off, 180 ms on |
| Unknown person becomes visible | Single 180 ms pulse |
| Track lost (occluded or expired) | Triple pulse: 60/70 ms three times |
| Presence behind a barrier | Slow 250 ms pulse every 1.25 s |

Collision range uses the ultrasonic ranger when it reports a valid sample;
otherwise the nearest track or map object within a forward +/-30 degree cone.
With `--haptic-device <path>`, the worker writes `1`/`0` so a GPIO or motor
controller can consume the state.

## Red-marker wrist guidance

A red marker is a navigation cue only. It is never person data and never becomes
a track or an identity.

`RedMarkerDetector` consumes an RGB888 view and computes red dominance per pixel
(`r - max(g, b)` with a brightness floor). It keeps the largest connected red
component, rejects specks that occupy too few pixels, and reports a bearing in
radians relative to the camera axis. Positive bearing means the marker is left
of the axis. `WristGuidance` then applies hysteresis: vibration engages inside
`--marker-engage` degrees (default 8), holds until `--marker-release` degrees
(default 14), and requires a fresh, confident observation (250 ms maximum age,
0.35 minimum confidence). `WristVibrator` writes `1`/`0` to
`--wrist-device <path>`, so a GPIO, haptic driver, or test file can consume it.

Camera transport is abstract. A camera bridge can emit either format:

```
MARK,<bearing_deg>,<confidence>[,<area_ratio>]
RED,<cx_px>,<cy_px>,<width_px>,<height_px>,<confidence>[,<hfov_deg>]
```

`RED` lines are converted to a bearing from the pixel offset and horizontal
field of view. A physical camera and wrist actuator are hardware deliverables;
the detector, hysteresis, recording, replay, and telemetry paths are testable
without them. The simulation emits a synthetic marker by default; use
`--no-marker-sim` to disable it. Marker observations are recorded and replayed.

On Windows, `--desktop-demo` opens the first available webcam through Media
Foundation and runs the red-marker detector on live RGB frames. It speaks
marker-found and marker-lost transitions through the Windows default audio
device using SAPI text-to-speech. Use `--duration <seconds>` to end the demo
automatically; otherwise press Ctrl+C.

```powershell
build\sa_daemon.exe --desktop-demo --duration 30
```

## Spoken announcements

`--announce` enables deterministic, line-oriented speech. With
`--announce-device <path>` each line is written as:

```
SAY Unit 5 meters ahead.
```

An external TTS process or earpiece firmware owns the actual audio. Without a
device path the logic still runs and the current sentence is exposed through
telemetry (useful for testing and for a browser-side speaker).
The Windows desktop demo enables built-in SAPI speech automatically.

`buildAnnouncement` sorts tracks by range and states the count, type, distance,
and world-relative bearing. Examples:

- `Friendly 1 meter to the right.`
- `Unit 5 meters ahead.`
- `3 people: 2 units, 1 friendly. Friendly 2 meters to the left. Unit 6 meters ahead.`
- `Presence detected, no tracked people.`
- `Area clear.`

Only `SAFE_TAGGED`, `UNKNOWN_PERSON`, and `PRESENCE_ONLY` tracks are described as
people. Red markers and static map objects are never announced as people.
Changed scenes are debounced by `--announce-interval`; an unchanged scene is
repeated at `--announce-repeat`, so the device does not chatter.

## Sensor modes

`--sensors` selects which inputs reach fusion. This is a perception mask, not a
hardware power switch: drivers still poll, and **raw frames are still recorded**,
so the same session can be replayed under a different selection.

| Mode | Inputs |
| --- | --- |
| `mixed` (default) | Every available sensor |
| `radar` | IMU + radar |
| `uwb` | IMU + UWB |
| `novelda` | IMU + Novelda |
| `ultrasonic` | IMU + ultrasonic |
| `vision` | IMU + red marker |
| `imu` | IMU only |

Custom lists are accepted, for example `--sensors radar,uwb,marker`. Presets keep
the IMU because it supplies ego-motion, not target data; a custom list can omit
it. Replay honors the selection:

```sh
./sa_daemon --replay=session.bin --sensors radar --no-telemetry
```

`--verify` compares against snapshots recorded from an unfiltered run. When a
sensor filter or a schematic is active, replay reports that reference comparison
was skipped rather than pretending the changed output is a failure.

## Schematics and map upload

`--map <file>` loads a text floor plan or an ASCII DXF as a structural prior.
DXF input supports `LINE` and straight-segment `LWPOLYLINE` entities, with
coordinates converted to metres using `$INSUNITS`. Curved polyline segments
are rejected rather than approximated. PDF and raster parsing are not
supported.

```
# maps/example_sa.txt
origin 0 0            # schematic origin in world metres
yaw 0                 # rotation in degrees
scale 1.0             # schematic units -> metres
wall -5 -4 5 -4       # wall segment x1 y1 x2 y2 [height_m]
door -1 -1 -1 1       # doorway: carves free space
room Office 1 -4 5 1  # axis-aligned room
label Ladder 3.7 -3.3 # named map object
```

The schematic is transformed into world coordinates and re-applied to the
occupancy grid after every probabilistic decay; otherwise the mandatory decay
would erase it. Sensor rays still refine the same cells, so a wrong schematic
shows up as a disagreement instead of being silently trusted. Walls and rooms
are exported in `WorldModel` and telemetry, and labels become map objects.
**A schematic never creates a person track.**

The phone UI can upload a schematic at runtime. `POST /api/map` accepts a raw
`text/plain` body up to 512 KiB. The HTTP thread queues it; parsing and validation
happen on the fusion thread, so processing stays deterministic and
single-threaded. Invalid plans are rejected with a log message and leave the
active schematic unchanged.

## Telemetry and phone visualization

The server binds to loopback by default. It serves:

| Endpoint | Contents |
| --- | --- |
| `/` | `web/index.html` live visualization |
| `/api/state` | JSON world model, sensor mode, announcement |
| `/api/grid` | Base64 occupancy grid, refreshed at most every 500 ms |
| `/api/map` | `POST` a raw text schematic (512 KiB cap) |
| `/api/health` | `{"ok":true}` liveness probe |
| `/favicon.ico` | 204 response |

`/api/state` always includes the required top-level keys `timestamp`, `pose`,
`environment_confidence`, `tracks`, `presence_zones`, and `map`, plus
occupancy counts and sensor health:

```json
{
  "timestamp": 12340,
  "sensor_mode": "mixed",
  "pose": {"x": 1.2, "y": -0.4, "yaw_deg": 8.5},
  "environment_confidence": 0.42,
  "sensor_health": {"imu": true, "radar": true, "uwb": true,
                    "novelda": true, "ultrasonic": true, "score": 0.9},
  "occupancy": {"occupied": 63, "known": 4211, "total": 25600},
  "tracks": [
    {
      "id": 1,
      "state": "VISIBLE",
      "classification": "SAFE_TAGGED",
      "position": {"x": 2.1, "y": 0.3, "z": 1.0},
      "velocity": {"x": 0.4, "y": 0.0, "z": 0.0},
      "position_sigma_m": 0.18,
      "existence_confidence": 0.92,
      "position_confidence": 0.81,
      "visible": true,
      "identity": {"tagged_safe": true, "tag_id": 1025, "confidence": 0.9}
    }
  ],
  "presence_zones": [],
  "map": [{"position": {"x": 4.0, "y": 1.2, "z": 0.8}, "confidence": 0.7}],
  "walls": [[-5, -4, 5, -4]],
  "rooms": [{"name": "Office", "min_x": 1, "min_y": -4, "max_x": 5, "max_y": 1}],
  "marker": {"detected": false},
  "wrist": {"enabled": true, "aimed": false, "vibrating": false},
  "announcement": "Friendly 2 meters to the left."
}
```

`/api/grid` returns `width`, `height`, `cell_size`, `origin_x`, `origin_y`, and
`data`, where `data` is a row-major base64 byte array of probabilities scaled
to 0-255.

### Using a phone

1. Start the daemon with a LAN-accessible bind:
   `./sa_daemon --simulate --realtime --bind 0.0.0.0 --port 8080`
2. Find the host IP (`ip addr` on Linux, `ipconfig` on Windows).
3. Open `http://<host-ip>:8080` on the phone, connected to the same network.
4. Keep the daemon behind a trusted network. There is no authentication; do
   not expose the port to the public internet.

The page renders a forward-up canvas with the occupancy image, track rings and
labels, presence wedges, map objects, and pose/health/confidence cards. It polls
state every 250 ms and the grid every 500 ms.

## Recording format

Sessions are little-endian and versioned. The current writer emits:

- **File header (64 bytes):** magic `SADASESS`, version `1`, header size `64`,
  endian marker `0x01020304`, creation time in Unix milliseconds, then reserved
  zero padding.
- **Record header (16 bytes):** `uint32 type`, `uint32 payload_size`,
  `uint32 crc32(payload)`, `uint32 reserved`.
- **Payload:** frame id and timestamp followed by the typed data. Each
  processing tick ends with a `FrameEnd` record, so truncated files are
  detectable.

Record types:

| Value | Type | Purpose |
| --- | --- | --- |
| 0 | Metadata | Creation time, application name, format version |
| 1 | Imu | Raw accelerometer/gyro/temperature sample |
| 2 | Radar | Point-cloud frame |
| 3 | Uwb | One tag ranging result |
| 4 | Novelda | One presence result |
| 5 | Ultrasonic | One range result |
| 6 | Tracks | Debug reference snapshot (default 2 Hz) |
| 7 | World | Debug reference snapshot (default 1 Hz) |
| 8 | FrameEnd | Tick boundary and snapshot flags |
| 9 | Marker | Red-marker bearing and confidence |
| 10 | Schematic | Active text schematic (at start and after runtime uploads) |

Only raw sensor records (including marker observations and schematic records) are
replay inputs. Track and world records exist for debugging and for `--verify`;
they never drive regenerated output.

Schematic records make a map session self-contained. Recording writes the active
schematic immediately after the session header, and writes another record at the
tick where a runtime `POST /api/map` upload becomes active. Replay applies the
embedded timeline in order, so `--replay session.bin` needs no external map file.
`--map` replaces the embedded *initial* prior for experiments; recorded
mid-session uploads still apply. An overridden session is no longer identical to
the recording, so `--verify` reports that comparison was skipped.

## Hardware interfaces

### TI IWR6843AOP

The radar driver reads TI mmWave demo packets from the data UART and parses
point-cloud TLV 1 with side-info TLV 7. When `--radar-cfg` is supplied, the
optional CLI UART receives the `.cfg` commands before the data stream opens.
The profile must be configured for point-cloud output; this project does not
implement the full range-Doppler or heatmap pipelines.

### ICM-42688

The IMU driver talks to the sensor over Linux I2C (`/dev/i2c-1`, address
`0x68` by default) and exposes raw acceleration, angular rate, and
temperature. Mounting orientation and gyro bias are configuration concerns;
the attitude estimator handles short-term integration.

### DWM3000 UWB

A small MCU or firmware bridge owns the DWM3000 SPI radio. The daemon consumes
one ranging result per line:

```
tag_id,range_m,aoa_deg,confidence[,range_rate_mps]
```

Empty or `nan` AoA fields mean the optional angle estimate is unavailable.
Because the bridge performs the radio-specific two-way ranging, this project
does not include a bare-metal DWM3000 SPI stack.

### Novelda / XeThru

The XeThru module also sits behind a firmware bridge and emits:

```
NOV,range_m,motion,breathing,confidence[,bearing_deg,bearing_width_deg[,through_barrier]]
```

The result is presence evidence only. The driver never produces a track, and
`through_barrier=1` lowers the zone confidence. This is not imaging.

### MaxBotix ultrasonic

The driver parses the asynchronous UART output (for example `R1234\r` in
millimetres) at 9600 baud and supplies the forward collision range used by
haptics.

## Tests and verification

`tests/test_core.cpp` covers:

- mandatory occupancy decay toward 0.5
- ray integration versus direct marking
- covariance-derived Kalman position sigma
- identity persistence beyond visibility
- Novelda reinforcement rules (visible tracks rejected, occluded tracks
  accepted)
- UWB-only and Novelda-only frames never creating tracks
- UWB association preserving tag identity without spawning a duplicate
- Novelda fusion requiring uncertainty overlap
- red-marker detection, rejection of tiny/gray images, and wrist hysteresis
- sensor preset/list parsing and raw-frame-preserving filtering
- announcement count, type, bearing, debounce, and repeat behavior
- schematic parsing, transform, occupancy priors, and no-track guarantee
- deterministic simulation, recording, replay, rewind, and exact world
  regeneration

The CTest pair records a 6 s deterministic session and replays it with
`--verify`.

Manual verification:

```sh
./sa_daemon --simulate --duration 10 --record=build/demo.bin \
    --no-telemetry --no-haptics
./sa_daemon --replay=build/demo.bin --verify --no-telemetry --no-haptics
```

The replay summary should show `failures=0`, `crc_errors=0`, and
`truncated=no`.

To exercise announcements, a schematic, and runtime map upload:

```sh
./sa_daemon --simulate --realtime --announce \
    --announce-device build/say.txt \
    --map maps/example_sa.txt --port 8080
curl --data-binary @maps/example_sa.txt http://127.0.0.1:8080/api/map
cat build/say.txt
```

A successful upload returns `202 Accepted`; the daemon log reports the parsed
wall, room, and label counts. Invalid plans are rejected without replacing the
active map.

## Determinism

- Processing is fixed-step and frame-ordered.
- Association, UWB candidate selection, clustering, and RANSAC use stable
  tie-breakers.
- The simulation has a private deterministic RNG seeded by `--seed`.
- Replay reads raw sensor values and recomputes every derived value.
- No wall-clock time enters the perception math; timestamps come from the
  frame.

This makes regressions reproducible: the same session must regenerate the same
tracks, grid, and confidence values.

## Limitations

- The hardware drivers target Linux and must be validated against the actual
  boards, firmware, and wiring. Simulation and replay are cross-platform.
- The IWR6843 profile, UWB bridge firmware, and Novelda bridge firmware are
  external deliverables.
- The red-marker camera, wrist actuator, and earpiece/speaker bridge are also
  external hardware. The daemon's side is implemented and tested, but no
  physical device is bundled or validated here.
- Red-marker detection is a simple red-dominance blob detector. Strong red
  lighting, reflective tape, or tiny markers can produce false results; it is a
  cue, not a certified targeting system.
- DXF schematic parsing supports ASCII `LINE` and straight-segment
  `LWPOLYLINE` entities only. PDF, raster images, and lidar scans are not
  parsed. There is no lidar driver, and a schematic alone cannot discover
  rooms that were never described.
- There is no global map-to-layout registration. Schematic coordinates must
  already align with the device's world frame (the frame in which the device
  starts near the plan origin); slowly drifting yaw will visibly offset the plan
  from sensor returns.
- Yaw is not absolute without a magnetometer and can drift slowly between
  radar corrections.
- Through-wall and through-concrete detection is not guaranteed. Presence
  evidence may be weak, delayed, or absent depending on the barrier.
- The occupancy grid is 2-D at torso height; it is not a full 3-D scene
  reconstruction.
- This is not a certified collision-avoidance or medical device.

## Troubleshooting

- **`telemetry warning: failed to bind`**: another process owns the port. Use
  `--port 0` for an ephemeral port or choose another port.
- **Web page says the visualization was not found**: run from the repository
  root or pass `--web-root <path-to-web>`.
- **No tracks in hardware mode**: verify the radar data port and that the
  `.cfg` enables point-cloud TLV output. Use `--verbose` to watch frame and
  track counts.
- **Haptics never fire**: ensure `--haptics` is enabled (default) and that
  `--haptic-device` points to a writable device. Without `--haptic-device`
  patterns are scheduled but not emitted to hardware.
- **No spoken announcements**: `--announce` must be set. Without
  `--announce-device` the sentence only appears in telemetry. Check that the
  bridge process can open the same device path.
- **Map upload returns 202 but nothing changes**: inspect the daemon log. The
  body is queued asynchronously and then validated; syntax errors are rejected
  there. See `maps/example_sa.txt` for the accepted format.
- **Replay verification fails**: the session may be truncated or was produced
  by an incompatible format version. Check `crc_errors` and `truncated` in the
  summary.
