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

## Final hardware profile

- Radxa Zero 3W, 4 GB RAM / 32 GB eMMC.
- Two TI IWR6843AOPEVM radars: front data/config ports
  `/dev/ttyUSB0` and `/dev/ttyUSB1`, yaw `0 deg`; rear ports `/dev/ttyUSB2` and
  `/dev/ttyUSB3`, yaw `180 deg`. Rotate detections and velocities into the
  canonical frame before merging.
- RPLIDAR S2 or YDLIDAR TG30 for 30 m on-demand 360-degree scans. The current daemon
  accepts the ASCII `LIDAR,x_m,y_m[,z_m]` bridge format; native binary parsing
  is LD19-specific.
- Two VL53L8CX 8x8 ToF arrays for continuous close-range surface mapping.
- ICM-42688-P IMU, two DWM3000 UWB modules, TMP117 temperature sensor,
  DRV2605L + LRA, and Bluetooth earbuds for optional spoken guidance.
- 1S2P 21700 battery pack, nominally 10,000 mAh, with suitable battery
  protection and regulated power rails.
- No camera is required by this hardware profile. The optional LibreVNA
  SFCW wall-pad sweeps 1.2-3.5 GHz through an external USB serial adapter; it
  supplies 1D/sector `PresenceZone` estimates only, not a 3D image or a
  standalone track.
- The separate wrist-worn IMU/LRA module may exchange newline-delimited
  `WRIST,yaw_deg,pitch_deg` and `HAPTIC,duty_pct,freq_hz,pattern` packets over
  a non-blocking serial/BLE-UART bridge. Calibrate yaw at startup with the
  wrist pointing +Y; replay must record the orientation samples. Pointing
  haptics may distinguish only `SAFE_TAGGED`, `UNKNOWN_PERSON`, and
  `PRESENCE_ONLY` tracks and must not introduce threat labels.

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
