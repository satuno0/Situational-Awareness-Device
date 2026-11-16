# Copilot instructions

## Frozen Spec v1.0

Treat these constraints as hard requirements for every change:

- Use the right-handed sensor/world frame: +X right, +Y forward, +Z up;
  bearings are `atan2(x, y)`.
- Keep replay deterministic. Pass explicit `timestamp_ms` and `dt`; do not read
  wall-clock time in `FusionEngine`, `TrackManager`, `OccupancyGrid`, or
  `KalmanTrack`.
- Keep UART, SPI, I2C, audio, GPIO, and localhost socket I/O non-blocking.
  Telemetry must bind only to `127.0.0.1:8765`.
- Use C++20 with `-Wall -Wextra -Werror -std=c++20`.
- Do not add Eigen, OpenCV, PCL, Boost, ROS, or other heavy runtime
  dependencies. Use lightweight, fixed-size math where practical.
- Keep static map geometry separate from person tracks. Only classify tracks
  as `SAFE_TAGGED`, `UNKNOWN_PERSON`, or `PRESENCE_ONLY`.
- Do not add threat assessment, combatant classification, emotion/intent
  inference, face or untagged-person identity recognition, 3D through-concrete
  tracking/imaging, or X-ray wall imaging. A 1D/sector breathing and motion
  estimate behind concrete is permitted as a `PresenceZone` from the external
  USB LibreVNA sweep adapter, but it may only reinforce a compatible existing
  occluded track and must never create a track.

## Locked hardware BOM, pinout, and bridge protocols (v1.0)

Do not change sensor roles, bus assignments, GPIO mappings, or serial bridge
formats unless the user explicitly requests it.

- **Core:** Radxa Zero 3W, 4 GB RAM / 32 GB eMMC.
- **Power:** 1S2P 21700 pack (2 x 5000 mAh), 15 A 1S BMS, off-only USB-C
  charger, 8 A fuse, latching switch, 6 A synchronous boost to `5V_SYS`, and
  1.5 A buck to `3V3_SENS`. Keep `3V3_SENS` strictly isolated from Radxa 3.3 V
  pins 1 and 17. Use shunt-damped C-L-C pi-filters for `5V_RADAR_A` and
  `5V_RADAR_B`, with device-side USB VBUS injection.
- **Radars:** two TI IWR6843AOPEVM. Front data/config are `/dev/ttyUSB0` and
  `/dev/ttyUSB1`, `yaw_offset_deg = 0.0f`; rear data/config are `/dev/ttyUSB2`
  and `/dev/ttyUSB3`, `yaw_offset_deg = 180.0f`. Rotate detections and
  velocities into the canonical frame before merging.
- **Shared I2C:** `I2C3_M0` is `/dev/i2c-1`, physical pin 3 SDA and pin 5 SCL.
  ICM-42688-P is `0x68`, TMP117 is `0x48`, and main DRV2605L is `0x5A`; the
  main LRA writes run on the haptics worker.
- **ToF:** two VL53L8CX 8x8 arrays on `/dev/i2c-1`. Left is assigned 7-bit
  address `0x30` at boot (ST ULD encoded address `0x60`), XSHUT/LPn is
  `/dev/gpiochip3` line 10, physical pin 18, yaw `+30 deg`. Right stays at
  default 7-bit `0x29` (ST encoded `0x52`), XSHUT/LPn is line 11, physical
  pin 29, yaw `-30 deg`.
- **LiDAR:** LDROBOT LD19 on UART2 `/dev/ttyS2` at 230400 baud, physical pin
  10; disable the Linux debug console. RPLIDAR S2 / YDLIDAR TG30 are supported
  through the ASCII bridge, not their native binary protocols. The TPS22965
  high-side power switch is `/dev/gpiochip3` line 12, physical pin 31,
  active-high. It is low at startup; `CMD:SCAN_360` requests a 1.5 s burst,
  with the first 300 ms discarded.
- **Wearable wrist pointer:** separate battery-powered Seeed XIAO ESP32-C3 or
  nRF52840 Sense with IMU and DRV2605L/LRA. The BLE-UART bridge uses
  `--wrist-module` at 115200 baud. Inbound orientation is sent at 50 Hz as
  newline-delimited `WRIST,<yaw_deg>,<pitch_deg>`; outbound haptic commands
  are `HAPTIC,<duty_pct>,<freq_hz>,<pattern>`. Startup yaw zero is straight
  ahead (+Y). Haptic labels remain strictly `SAFE_TAGGED`, `UNKNOWN_PERSON`,
  and `PRESENCE_ONLY`; no threat labels.
- **Audio:** paired Bluetooth earbuds; no wired I2S DAC/amplifier circuit.
- **Optional non-blocking external serial bridges:** unplugged devices must
  degrade gracefully. DWM3000 (`--uwb`) sends
  `tag_id,range_m,aoa_deg,confidence[,range_rate_mps][,is_fixed_anchor,anchor_x_m,anchor_y_m]`;
  it confirms tags or constrains pose with a fixed anchor, never creates
  tracks. Novelda/XeThru X4M200 (`--novelda`) sends
  `NOV,range_m,motion,breathing,confidence[,bearing_deg,bearing_width_deg[,through_barrier]]`
  and only reinforces existing occluded tracks. LibreVNA wall-pad (`--wallpad`,
  1.2-3.5 GHz SFCW) sends
  `WALLPAD,range_min_m,range_max_m,bearing_center_deg,bearing_width_deg,confidence`;
  it provides 1D/sector `PresenceZone` evidence only, never tracks or 3D
  through-concrete images. MaxBotix (`--ultrasonic`) sends `R<mm>\r` at
  9600 baud for close-range collision override.

## Architecture and mapping

- Radar detections feed `TrackManager`; do not use radar clutter as primary
  surface geometry while surface sensors are healthy.
- LiDAR and ToF provide surface rays to `OccupancyGrid`. The default map is
  280 x 280 cells at 0.25 m, centered at the origin (70 m x 70 m coverage).
- Keep scan-to-map matching deterministic and run it against the existing map
  before inserting the current frame's LiDAR/ToF rays. Start local matching
  only when `environment_confidence > 0.50`; use the bounded configured search.
- A completed 1.5 s LiDAR burst may run bounded loop closure only against
  established high-confidence map cells.
- Preserve Bayesian probability decay toward the unknown prior (`0.5`) with
  the configured default factor `0.999`.
- Fixed UWB anchors may softly constrain device pose; ordinary UWB tags only
  confirm an already-existing track.
- Novelda presence may reinforce a compatible occluded track but never spawn
  one.
- LibreVNA wall-pad sectors follow the same confirmation-only policy: consume
  validated `PresenceZone` records, and let fusion reinforce only an existing
  occluded track whose uncertainty overlaps the sector.

Prefer the smallest focused change that preserves these contracts. Update
directly related tests and documentation, and run the relevant CMake build and
CTest suite with strict warnings.
