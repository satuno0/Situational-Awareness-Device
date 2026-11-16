# SA Daemon

A C++20 situational-awareness daemon for a handheld assistive device. It fuses
dual multi-zone ToF, an on-demand scanning LiDAR, dual front/rear mmWave radar,
IMU, UWB ranging, Novelda/XeThru and optional LibreVNA sector-presence sensing,
and a MaxBotix ultrasonic ranger into a world model with:

- 6-state Kalman person tracks (position and velocity)
- obstacle and free-space occupancy mapping
- safe-tag identification via UWB ranging (confirmation only), with optional
  fixed-anchor range constraints on device pose
- occluded-track reinforcement via Novelda/LibreVNA presence (confirmation only)
- haptic feedback for collisions, tagged/untagged people, lost tracks, and
  presence behind a barrier
- wrist pointing-cone cues for `SAFE_TAGGED`, `UNKNOWN_PERSON`, and
  `PRESENCE_ONLY` tracks using the wearable IMU/LRA module
- optional red-marker navigation cues, kept separate from person tracks
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
- UWB is never the primary tracker. Novelda and LibreVNA presence are never
  allowed to create tracks. They can only reinforce a compatible existing
  occluded track.
- 3D through-concrete tracking and imaging are **not goals**. An external USB
  LibreVNA sweep adapter may provide 1D/sector breathing and motion
  `PresenceZone` evidence behind concrete; this is sector evidence, not an
  image or a standalone track source.

This is an assistive device, not a certified safety instrument. Do not rely on
it as the sole means of avoiding injury.

## Requirements

- C++20 compiler (GCC 11+, Clang 13+, or MSVC 19.30+)
- CMake 3.20+ (optional; direct compiler commands are documented below)
- POSIX threads on Linux
- No external runtime libraries; the ST VL53L8CX ULD and BSD-3-Clause license
  are included under `third_party/st_vl53l8cx`

## Repository layout

```
include/                 Public interfaces and core data types
  math.hpp               Vec3, Quaternion, Pose2D, angles, filters
  types.hpp              Detection, Track, WorldModel, sensor frames
  kalman_track.hpp       6-state constant-velocity Kalman filter
  track_manager.hpp      Association, lifecycle, identity persistence
  occupancy_grid.hpp     Bayesian 280 x 280 grid and map objects
  slam_manager.hpp       AHRS pose integration and LiDAR scan correction
  fusion_engine.hpp      Sensor fusion pipeline and confidence engine
  radar_driver.hpp       Dual-instance TI IWR6843AOPEVM packet driver
  lidar_driver.hpp       On-demand LD19 / ASCII LiDAR scan-burst driver
  tof_driver.hpp         Dual VL53L8CX ULD driver and 3D zone projection
  gpio_line.hpp          Linux GPIO character-device line helper
  imu_driver.hpp         ICM-42688 I2C driver + attitude estimator
  uwb_driver.hpp         DWM3000 serial-bridge driver
  novelda_driver.hpp     XeThru/Novelda serial-bridge driver
  wallpad_driver.hpp     LibreVNA sector-presence USB serial-bridge driver
  ultrasonic_driver.hpp  MaxBotix UART driver
  telemetry.hpp          Loopback HTTP/JSON server
  recorder.hpp           Versioned binary session writer
  playback.hpp           Session reader and replay source
  haptics.hpp            Haptic pattern scheduler
  marker_detector.hpp    Red-marker RGB detector and bearing estimator
  marker_driver.hpp      MARK/RED camera-bridge serial protocol
  wrist_haptics.hpp      Wrist orientation bridge and pointing-cone haptics
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
third_party/st_vl53l8cx/ Bundled ST VL53L8CX ULD and BSD-3-Clause license
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
g++ -std=c++20 -O2 -Wall -Wextra -Werror -Iinclude src/*.cpp -o sa_daemon -pthread

# Tests exclude main.cpp
g++ -std=c++20 -O2 -Wall -Wextra -Werror -Iinclude \
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
./sa_daemon --simulate --realtime --duration 60 --port 8765
# Open http://127.0.0.1:8765
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
The current session format is v8; v7 and earlier sessions are rejected because
wrist orientation is now recorded as a replay input.

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
    --radar-front-data /dev/ttyUSB0 \
    --radar-front-cfg /dev/ttyUSB1 \
    --radar-rear-data /dev/ttyUSB2 \
    --radar-rear-cfg /dev/ttyUSB3 \
    --radar-cfg /path/to/iwr6843_handheld.cfg \
    --lidar /dev/ttyS2 \
    --imu /dev/i2c-1 \
    --uwb /dev/ttyACM0 \
    --novelda /dev/ttyACM1 \
    --wallpad /dev/ttyACM2 \
    --wrist-module /dev/ttyACM3 \
    --ultrasonic /dev/ttyUSB5 \
    --record=session.bin \
    --port 8765
```

Exact device paths depend on the host and udev rules. Each IWR6843AOPEVM uses
one data UART and one configuration UART. The front radar is mounted at 0
degrees; the rear radar is rotated 180 degrees in the driver before the two
frames are merged for tracking.

#### Finalized hardware profile (locked v1.0)

- **Core:** Radxa Zero 3W, 4 GB RAM / 32 GB eMMC.
- **Power:** 1S2P 21700 pack (2 x 5000 mAh), 15 A 1S BMS, off-only USB-C
  charger, 8 A fuse, and latching switch. The pack feeds a 6 A synchronous
  5V_SYS boost; a separate 1.5 A 3V3_SENS buck powers sensors and is strictly
  isolated from Radxa 3.3 V pins 1 and 17. Use shunt-damped C-L-C pi-filters
  on 5V_RADAR_A and 5V_RADAR_B, with device-side USB VBUS injection.
- **Primary 360-degree tracking:** two TI IWR6843AOPEVM radars. Front data/config
  are `/dev/ttyUSB0` and `/dev/ttyUSB1` at 0 degrees; rear data/config are
  `/dev/ttyUSB2` and `/dev/ttyUSB3` at 180 degrees.
- **Continuous surface mapping:** two ST VL53L8CX 8x8 ToF arrays, yawed +30
  degrees (left) and -30 degrees (right), for close-range mapping.
- **Orientation and environment:** TDK ICM-42688-P IMU, TI TMP117 temperature
  sensor, and TI DRV2605L with an LRA for main-unit haptics.
- **On-demand scan:** LDROBOT LD19 on UART2 (`/dev/ttyS2`, 230400 baud) using
  its native packet format, or an RPLIDAR S2 / YDLIDAR TG30 through the
  documented ASCII bridge for 30 m 360-degree bursts. The motor is
  power-gated and off at startup.
- **Optional confirmation bridges:** two DWM3000 UWB modules, Novelda / XeThru
  X4M200, and a detachable LibreVNA SFCW concrete wall-pad (1.2-3.5 GHz).
  They confirm/reinforce existing tracks or report sector presence; none
  creates tracks. LibreVNA output is 1D/sector presence only, never 3D imaging.
- **Wearable:** separate battery-powered Seeed XIAO ESP32-C3 or nRF52840 Sense
  with an IMU and DRV2605L/LRA, connected through a BLE-UART bridge. It sends
  orientation at 50 Hz and receives wrist haptic commands at 115200 baud.
- **Audio:** paired Bluetooth earbuds; no wired I2S DAC/amplifier circuit.

#### Locked buses, addresses, and pinout

- Shared `I2C3_M0` is exposed as `/dev/i2c-1`: physical pin 3 SDA and pin 5
  SCL. The ICM-42688-P uses address `0x68`, TMP117 `0x48`, and main DRV2605L
  `0x5A` (the haptics worker performs its I2C output).
- Both ToF sensors use this bus. Left VL53L8CX is assigned 7-bit address
  `0x30` at boot (ST ULD encoded address `0x60`), with XSHUT/LPn on
  `/dev/gpiochip3` line 10, physical pin 18, yaw +30 degrees. Right remains at
  default 7-bit address `0x29` (ST encoded `0x52`), with XSHUT/LPn on line 11,
  physical pin 29, yaw -30 degrees.
- LD19 UART2 RX is `/dev/ttyS2`, physical pin 10; disable the Linux debug
  console on that UART. Its TPS22965 high-side load switch is controlled by
  `/dev/gpiochip3` line 12, physical pin 31, active high. The gate remains low
  at startup and powers the sensor only for a 1.5 s `CMD:SCAN_360` burst;
  discard the first 300 ms of scan frames.

The default occupancy grid is 280 x 280 cells at 0.25 m per cell, covering a
70 m x 70 m area centered on the device. 30 m scans are matched and mapped
before the current frame's surface rays are inserted.

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
  --port <port>              HTTP port; 0 selects an ephemeral port (default 8765)
  --bind <address>           Bind address (only 127.0.0.1 is allowed)
  --web-root <dir>           Directory containing index.html (default web)
  --no-telemetry             Disable the HTTP server

Haptics:
  --no-haptics               Disable the haptic worker
  --haptic-device <path>     Write motor state (1/0) to a device or file
  --verbose                  Log haptic events and progress

Red marker / wrist:
  --no-wrist                 Disable wrist guidance
  --wrist-device <path>      Write wrist motor state (1/0) to a device or file
  --wrist-module <dev>       Wrist IMU/LRA BLE-UART bridge (WRIST and HAPTIC lines)
  --wrist-baud <rate>        Wrist bridge baud rate (default 115200)
  --wrist-offset <deg>       Additional startup wrist yaw zero offset
  --wrist-sweep              Sweep wrist orientation in deterministic simulation
  --marker <dev>             Camera bridge (MARK/RED lines) serial port
  --marker-offset <deg>      Camera yaw offset relative to device forward
  --marker-engage <deg>      Aim cone that starts vibration (default 8)
  --marker-release <deg>     Aim cone that stops vibration (default 14)
  --no-marker-sim            Disable the synthetic marker in simulation

Perception / audio / map:
  --sensors <mode>           mixed, radar, lidar, tof, uwb, novelda, wallpad, ultrasonic,
                             vision, imu, or a comma list such as radar,uwb
  --announce                 Speak track summaries
  --announce-device <path>   Speaker bridge (writes SAY <text> lines)
  --announce-interval <s>    Debounce for changed scenes (default 1.5)
  --announce-repeat <s>      Repeat unchanged summaries (default 8)
  --map <file>               Load a text or ASCII DXF floor plan as a prior

Hardware:
  --radar-front-data <dev>   Front IWR6843 data port (default /dev/ttyUSB0)
  --radar-front-cfg <dev>    Front IWR6843 config port (default /dev/ttyUSB1)
  --radar-rear-data <dev>    Rear IWR6843 data port (default /dev/ttyUSB2)
  --radar-rear-cfg <dev>     Rear IWR6843 config port (default /dev/ttyUSB3)
  --radar-cfg <file>         mmWave .cfg file sent to each radar
  --radar-data <dev>         Alias for --radar-front-data
  --radar-cli <dev>          Alias for --radar-front-cfg
  --lidar <dev>              LD19 or ASCII LIDAR,x_m,y_m[,z_m] (default /dev/ttyS2)
  --lidar-baud <rate>        LiDAR UART baud rate (default 230400 for LD19)
  --lidar-gpio <spec>        LD19 motor power gate (default /dev/gpiochip3:12:high)
  --tof-i2c <dev>            Dual VL53L8CX I2C bus (default /dev/i2c-1)
  --tof-left-gpio <spec>     Left sensor XSHUT line (default /dev/gpiochip3:10:high)
  --tof-right-gpio <spec>    Right sensor XSHUT line (default /dev/gpiochip3:11:high)
  --tof-left-addr <hex>      Left ST-encoded address (default 0x60 / 7-bit 0x30)
  --tof-right-addr <hex>     Right ST-encoded address (default 0x52 / 7-bit 0x29)
  --tof-left-yaw <deg>       Left sensor yaw (default 30)
  --tof-right-yaw <deg>      Right sensor yaw (default -30)
  --no-tof                   Disable dual ToF
  --haptic-i2c <dev>         DRV2605L I2C device (worker-thread output)
  --haptic-addr <value>      DRV2605L 7-bit address (default 0x5A)
  --imu <dev>                ICM-42688 I2C device (default /dev/i2c-1)
  --uwb <dev>                DWM3000 bridge serial port
  --novelda <dev>            XeThru bridge serial port
  --wallpad <dev>            LibreVNA sweep-adapter USB serial port
  --ultrasonic <dev>         MaxBotix serial port
```

`--simulate` and `--replay` are mutually exclusive. Recording during replay is
rejected because replay is already a consumer of a session.

## Perception pipeline

For every sensor frame, `FusionEngine::process()` performs the following in a
fixed order:

1. **IMU and attitude.** The ICM-42688 sample feeds a complementary/Mahony
   attitude estimator. Without a magnetometer, yaw is gyro-integrated and is
   corrected by LiDAR scan matching; it is not treated as absolute heading.
2. **Surface sensing and localization.** Dual VL53L8CX sensors supply up to
   128 3D zone rays per frame during silent patrol. An optional 30 m RPLIDAR
   S2 / YDLIDAR TG30 burst, bridged to the supported ASCII point format,
   supplies 360-degree surface points for mapping and scan matching. Its motor
   gate is off at startup; `CMD:SCAN_360` powers it for 1.5 seconds, discarding
   the first 300 ms of samples. The UART accepts native 47-byte LD19 packets
   or ASCII `LIDAR,x_m,y_m[,z_m]` lines. Coordinates use +X right, +Y forward,
   +Z up. Once environment confidence exceeds 0.50, deterministic correlative
   matching searches ±0.15 m in 0.05 m steps and ±4 degrees in 1-degree steps
   against the existing map before current-frame surface rays are inserted.
   A completed 1.5-second burst also runs a coarse-to-fine ±1.5 m / ±25-degree
   loop closure against occupied cells above 0.75 probability. Neither path
   reads wall-clock time.
3. **Radar person-candidate selection.** Front/rear IWR6843 detections are
   rotated by 0/180 degrees into the canonical sensor frame, merged, and
   transformed into the world frame for person tracking. The
   default gates are: cluster radius 0.36 m, 2-6 points, centroid z between
   0.78 m and 1.70 m, plan-view extent at most 0.55 m, vertical span at least
   0.015 m, and at least 0.40 m clearance from static structure. Radar does
   not enter the primary map while LiDAR is healthy.
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
6. **Presence-zone confirmation.** Novelda and the optional LibreVNA bridge
   produce timed range/bearing sectors. A sector can only reinforce an
   already-occluded track whose uncertainty overlaps the zone. Neither sensor
   can create a track; the wall-pad emits no 3D image or standalone track.
7. **Mapping.** The occupancy grid uses ToF and LiDAR surface rays for
   Bayesian log-odds mapping. Radar rays are fallback-only while both surface
   sensors are unhealthy.
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
controller can consume the state. On Linux, `--haptic-i2c <dev>` selects the
DRV2605L LRA controller; I2C output and the PWM fallback are handled by the
haptics worker.

## Surface sensors

The dual VL53L8CX arrays use 8x8 multi-zone ranging at up to 4 m. Each device
needs its own active-high XSHUT line so the driver can assign distinct I2C
addresses at startup. The default bus, GPIO lines, sensor yaw, and pitch are
configurable; measure the actual mounting pose before using the map. I2C work
runs on the driver worker, and completed rays are timestamped before entering
the deterministic fusion/replay pipeline.

The LD19 is optional and normally unpowered. The default GPIO gate is active
high and must control the TPS22965 high-side load switch, not the motor
directly. The default UART is `/dev/ttyS2` at 230400 baud; disable its Linux
debug console. Physical pin 10 is UART2 RX and `/dev/gpiochip3` line 12
(physical pin 31) is the active-high gate.
An active localhost client can request a 1.5-second burst with the dashboard
button. The first 300 ms are discarded for spin-up; retriggers during a burst
are ignored. Verify the LD19 packet/CRC and GPIO polarity on the target Linux
hardware before relying on this path.

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

## Wrist pointing-cone guidance

The separate battery-powered Seeed XIAO ESP32-C3 or nRF52840 Sense module
streams orientation at 50 Hz as newline-delimited
`WRIST,yaw_deg,pitch_deg` records over a non-blocking BLE-UART bridge at
115200 baud. Its IMU and DRV2605L/LRA are local to the wearable.
Point the wrist straight forward (+Y in the scanner frame) during startup; the
first fresh yaw sample establishes zero, with `--wrist-offset` applied as an
additional correction. Orientation samples are timestamped by the processing
loop and recorded, so replay uses the same calibration and guidance inputs.

For active tracks, target bearing is `atan2(dx, dy)` in the +X-right/+Y-forward
frame relative to the device pose. Within the ±15° cone, `UNKNOWN_PERSON`
produces proportional 4-20 Hz pulses scaled slightly by range;
`SAFE_TAGGED` produces a distinct low-intensity double-tap; and
`PRESENCE_ONLY` uses a gentle slow pulse. Outside the cone the command is
silent unless the existing collision haptic is active. The module output is
queued to a worker thread as newline-delimited
`HAPTIC,duty_pct,freq_hz,pattern` commands, keeping serial writes off the
processing loop.

Use `--simulate --wrist-sweep` to exercise orientation and pointing logic with
deterministic synthetic yaw/pitch samples. This does not create or alter tracks.

```powershell
build\sa_daemon.exe --desktop-demo --duration 30
```

## Spoken announcements

`--announce` enables deterministic, line-oriented speech. With
`--announce-device <path>` each line is written as:

```
SAY UNKNOWN_PERSON 5 meters ahead.
```

An external TTS process or earpiece firmware owns the actual audio. Without a
device path the logic still runs and the current sentence is exposed through
telemetry (useful for testing and for a browser-side speaker).
The Windows desktop demo enables built-in SAPI speech automatically.

`buildAnnouncement` sorts tracks by range and states the count, type, distance,
and world-relative bearing. Examples:

- `SAFE_TAGGED 1 meter to the right.`
- `UNKNOWN_PERSON 5 meters ahead.`
- `3 people: 2 UNKNOWN_PERSON, 1 SAFE_TAGGED. SAFE_TAGGED 2 meters to the left. UNKNOWN_PERSON 6 meters ahead.`
- `PRESENCE_ONLY 2 meters ahead.`
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
| `lidar` | IMU + LiDAR |
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
                    "lidar": true, "novelda": true, "ultrasonic": true, "score": 0.9},
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
  "announcement": "SAFE_TAGGED 2 meters to the left."
}
```

`/api/grid` returns `width`, `height`, `cell_size`, `origin_x`, `origin_y`, and
`data`, where `data` is a row-major base64 byte array of probabilities scaled
to 0-255.

### Remote visualization

Telemetry is restricted to `127.0.0.1:8765`. Open
`http://127.0.0.1:8765` on the device, or use an SSH local port forward from a
trusted client to view it remotely; the daemon does not listen on a LAN address.

The page renders a forward-up canvas with the occupancy image, track rings and
labels, presence wedges, map objects, and pose/health/confidence cards. It polls
state every 250 ms and the grid every 500 ms.

## Recording format

Sessions are little-endian and versioned. The current writer emits version 8;
versions 1-7 are rejected with an explanation because their records predate
the current coordinate, sensor, map, wall-pad, or wrist-orientation profile.
The file structure is:

- **File header (64 bytes):** magic `SADASESS`, version `8`, header size `64`,
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
| 11 | LiDAR | Surface scan points and burst lifecycle |
| 12 | ToF | Surface-ray frame |
| 13 | Wallpad | 1D/sector presence zone |
| 14 | WristOrientation | Wearable yaw/pitch sample |

Only raw sensor records (including marker and wrist-orientation observations,
wall-pad sectors, and schematic records) are replay inputs. Track and world
records exist for debugging and for `--verify`; they never drive regenerated
output.

Schematic records make a map session self-contained. Recording writes the active
schematic immediately after the session header, and writes another record at the
tick where a runtime `POST /api/map` upload becomes active. Replay applies the
embedded timeline in order, so `--replay session.bin` needs no external map file.
`--map` replaces the embedded *initial* prior for experiments; recorded
mid-session uploads still apply. An overridden session is no longer identical to
the recording, so `--verify` reports that comparison was skipped.

## Hardware interfaces

### Dual TI IWR6843AOPEVM

The radar driver reads TI mmWave demo packets from the data UART and parses
point-cloud TLV 1 with side-info TLV 7. The default front/rear UART pairs are
`/dev/ttyUSB0` + `/dev/ttyUSB1` and `/dev/ttyUSB2` + `/dev/ttyUSB3`. The rear
frame receives a 180-degree yaw rotation before detections are merged. When
`--radar-cfg` is supplied, each config UART receives the `.cfg` commands before
its data stream opens. Both profiles must emit point-cloud output; this project
does not implement full range-Doppler or heatmap pipelines.

### LibreVNA wall-pad

The optional 1.2-3.5 GHz SFCW pod is consumed through a non-blocking USB serial
sweep adapter; this driver does not control the VNA sweep itself. The adapter
must convert its measurement into lines of this form:

```
WALLPAD,range_min_m,range_max_m,bearing_center_deg,bearing_width_deg,confidence
```

Bearing is relative to +Y forward and positive toward +X right. The driver
validates the range (up to 30 m), sector width (5-170 degrees), bearing, and
confidence before emitting `PresenceZone` data. Fusion only uses an overlapping
sector to support an already-occluded track. No track is created from this
input.

### ICM-42688

The IMU driver talks to the sensor over Linux I2C (`/dev/i2c-1`, address
`0x68` by default) and exposes raw acceleration, angular rate, and
temperature. Mounting orientation and gyro bias are configuration concerns;
the attitude estimator handles short-term integration.

### DWM3000 UWB

A small MCU or firmware bridge owns the DWM3000 SPI radio. The daemon consumes
one ranging result per line, with optional range-rate and fixed-anchor fields:

```
tag_id,range_m,aoa_deg,confidence[,range_rate_mps][,is_fixed_anchor,anchor_x_m,anchor_y_m]
```

Empty or `nan` AoA fields mean the optional angle estimate is unavailable.
Because the bridge performs the radio-specific two-way ranging, this project
does not include a bare-metal DWM3000 SPI stack.
An optional fixed-map anchor suffix is
`[,is_fixed_anchor,anchor_x_m,anchor_y_m]`. Fixed anchors apply a soft
range-circle correction to the device pose and are excluded from person-tag
association.

### Novelda / XeThru

The XeThru module also sits behind a firmware bridge and emits:

```
NOV,range_m,motion,breathing,confidence[,bearing_deg,bearing_width_deg[,through_barrier]]
```

The result is presence evidence only. The driver never produces a track, and
`through_barrier=1` lowers the zone confidence. This is not imaging.

### MaxBotix ultrasonic

The optional MaxBotix bridge sends `R<mm>\r` at 9600 baud (for example,
`R1234\r`). The driver parses the range in millimetres and supplies the forward
collision range used by haptics.

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
- wrist packet validation, startup zeroing, proportional pointing, silent
  out-of-cone behavior, and orientation record/replay
- sensor preset/list parsing and raw-frame-preserving filtering
- ToF surface-only mapping and deterministic ToF record/replay
- LD19 CRC/angle parsing and timestamped burst spin-up/deadline behavior
- confidence-gated ToF scan matching, burst loop closure, and fixed-anchor pose correction
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
    --map maps/example_sa.txt --port 8765
curl --data-binary @maps/example_sa.txt http://127.0.0.1:8765/api/map
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
- The TMP117 is part of the locked hardware BOM and uses address `0x48`, but
  the daemon does not currently sample it or expose a TMP117 CLI option.
- The red-marker camera, wrist actuator, and earpiece/speaker bridge are also
  external hardware. The daemon's side is implemented and tested, but no
  physical device is bundled or validated here.
- Red-marker detection is a simple red-dominance blob detector. Strong red
  lighting, reflective tape, or tiny markers can produce false results; it is a
  cue, not a certified targeting system.
- DXF schematic parsing supports ASCII `LINE` and straight-segment
  `LWPOLYLINE` entities only. PDF and raster images are not parsed. The LiDAR
  driver accepts the documented serial bridge format, not vendor-specific raw
  sensor protocols; a schematic alone cannot discover rooms that were never
  described.
- There is no global map-to-layout registration. Schematic coordinates must
  already align with the device's world frame (the frame in which the device
  starts near the plan origin); slowly drifting yaw will visibly offset the plan
  from sensor returns.
- Yaw is not absolute without a magnetometer and can drift slowly between
  radar corrections.
- Behind-wall/behind-concrete sector-presence evidence is experimental and may
  be weak, delayed, or absent depending on the barrier; it is not an image or
  a standalone track source.
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
