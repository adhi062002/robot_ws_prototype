# Orion UGV — Interface Contract

Single source of truth for names, frames, units and file formats, so hardware
wiring and software agree. **If you change something here, change it in code in
the same commit.** Status tags: **[exists]** in the repo and tested, **[planned]**
agreed here but not yet written, **[TBD]** needs a decision or a measurement.

## 1. Conventions

- SI units everywhere: metres, seconds, radians, rad/s, m/s². Depth PNGs are the one exception (uint16 millimetres, see §5).
- ROS 2 Humble, Ubuntu 22.04, Jetson Orin Nano.
- Body frame (REP-103): x forward, y left, z up. Camera optical frame: x right, y down, z forward.
- Wheel order in every array/packet: **0 = FL, 1 = FR, 2 = RL, 3 = RR**. Positive wheel velocity = robot moving forward. Per-wheel direction/encoder inversion is handled by the two invert masks in the Teensy config, never by sign tricks elsewhere.
- Timestamps: ROS messages use ROS time. The Teensy sends its own `micros()`; the bridge node maps it to ROS time (see §6) and must not stamp with arrival time.

## 2. TF tree **[planned]**

```
map ─(slam_toolbox)→ odom ─(robot_localization EKF)→ base_link
base_link ─(static, measured)→ lidar_link
base_link ─(static, measured)→ imu_link
base_link ─(static, measured)→ camera_link ─(realsense2_camera)→ camera_color_optical_frame, camera_depth_optical_frame …
```

Wheel odometry is published as a message only (no TF); the EKF owns `odom→base_link`.
`base_link→camera_link`, `lidar_link`, `imu_link` come from the URDF and **must be measured** (§8).

## 3. Topics **[planned unless noted]**

| Topic | Type | Frame / notes | Producer |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | linear.x m/s, angular.z rad/s | Nav / teleop / wall-follower |
| `/wheel/odom` | `nav_msgs/Odometry` | child `base_link`, from encoders | teensy_bridge |
| `/imu/data_raw` | `sensor_msgs/Imu` | `imu_link`, gyro rad/s, accel m/s² | teensy_bridge |
| `/imu/data` | `sensor_msgs/Imu` | filtered orientation | imu_filter_madgwick |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF output | robot_localization |
| `/scan` | `sensor_msgs/LaserScan` | `lidar_link` | lidar driver **[TBD model]** |
| `/camera/camera/color/image_raw` **[exists]** | `sensor_msgs/Image` | color | realsense2_camera |
| `/camera/camera/aligned_depth_to_color/image_raw` **[exists]** | `sensor_msgs/Image` | 16UC1 mm | realsense2_camera (`align_depth.enable:=true`) |
| `/camera/camera/color/camera_info` **[exists]** | `sensor_msgs/CameraInfo` | | realsense2_camera |
| `/estop` | `std_msgs/Bool` | true = stopped | teensy_bridge / supervisor |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | link, encoders, faults | teensy_bridge |
| `/dents/detections` | `vision_msgs/Detection2DArray` | live YOLO, original image stamp | yolo_live_node |

## 4. Actions **[exists]** (`dent_mapping_interfaces`)

| Action | Server | Purpose |
|---|---|---|
| `capture_scan` | `realsense_frame_capture_node` | create a run folder and save frames |
| `detect_dents` | `dent_yolo_node` | YOLO over a saved run, writes `dent/` |
| `reconstruct_scan` | `dent_reconstruction_node` | register frames, fuse, mark dents |

## 5. Scan session on disk

A run folder (default under `~/datasets/`):

```
<run>/
  intrinsics.json        [exists]  fx fy cx cy width height   (+ optional depth_scale, default 1000)
  rgb/000000.png …       [exists]  8-bit color
  depth/000000.png …     [exists]  uint16, millimetres, aligned to color, same size as rgb
  poses.json             [planned] per-frame robot/camera pose, see below
  dent/dent_000.json/.png [exists] detection record + crop (written by detect_dents)
  reconstruction.ply     [exists]  largest connected segment, dents as red outlines, y-up
  reconstruction_seg1.ply … [exists] other segments, only when registration failed between them
  quality_report.json    [exists]  per-pair results, segments, dents marked/skipped, warnings
```

Rules (enforced in code): `rgb/` and `depth/` must match 1:1 by filename; image size must equal the size in
`intrinsics.json` (mismatch is an error); a missing `intrinsics.json` is an error (no silent default camera).

`dent/dent_NNN.json`: `{"frame": "000012.png", "bbox": [x1, y1, x2, y2], "confidence": 0.91, ...}` in full-resolution pixels.

`poses.json` **[planned]** — one entry per saved frame, so reconstruction can use odometry/lidar as a prior:

```json
{"frames": [{"file": "000012.png", "stamp": {"sec": 1730000000, "nanosec": 5000},
             "T_map_cam": [16 floats, row-major, camera optical frame in map],
             "T_odom_cam": [16 floats], "source": "ekf|slam", "valid": true}]}
```

`quality_report.json` fields: `n_pairs_ok/failed`, `segments[]`, `dents_marked[]` (name, frame, segment),
`dents_skipped[]` (name, reason), `pairs[]` (ok, method, n_correspondences, icp_fitness, icp_rmse_m,
normal_spread, reason), `warnings[]`, `config{}`. **Read the warnings before trusting a scan.**

## 6. Teensy ↔ Jetson serial protocol **[exists, tested]**

Definition: `firmware/teensy41_ugv/ugv_protocol.h` (C++) and `src/ugv_hardware/ugv_hardware/protocol.py`
(Python). `src/ugv_hardware/test/test_protocol.py` compiles the C++ and cross-checks every message.

Frame, little-endian: `A5 5A | type | len | seq | payload | crc16` — CRC-16/CCITT-FALSE over `type..payload`.

| Type | Dir | Payload |
|---|---|---|
| `0x01 TELEMETRY` | T→J, 100 Hz | `t_us`, 4× `enc_ticks` (cumulative i32), 4× `wheel_vel` rad/s, gyro[3], accel[3], `imu_t_us`, `status`, `cmd_age_ms`, `batt_mv`, `enc_ok_mask` (71 B) |
| `0x02 CONFIG_REPORT` | T→J | echo of the active config (24 B) |
| `0x10 CMD_WHEELS` | J→T, **≥ 20 Hz** | `left_rad_s`, `right_rad_s`, `flags` (bit0 = enable) (9 B). Doubles as the heartbeat |
| `0x11 CONFIG_SET` | J→T, at startup | `ticks_per_wheel_rev`, `kp`, `ki`, `kd`, `max_wheel_rad_s`, `watchdog_ms`, `enc_invert_mask`, `motor_invert_mask` (24 B) |
| `0x12 CLEAR_FAULT` | J→T | — clears latched e-stop (if released) and stall faults |
| `0x13 REQUEST_CONFIG` | J→T | — |
| `0x14 CMD_SERVOS` | J→T | `servo1_deg`, `servo2_deg` (int16 ×2, 4 B). Clamped on the Teensy to 0–105° / 0–180° and moved with the sigmoid smoothing from the old sketch |

Status bits: `ESTOP, CMD_WATCHDOG, IMU_STALE, ENC_FAULT, MOTOR_FAULT, CONFIG_VALID, LOW_BATTERY, ARMED, OPEN_LOOP`.
`OPEN_LOOP` = firmware built with `ENCODERS_FITTED 0` (no encoders mounted yet): duty = target / `max_wheel_rad_s`, speeds are not measured, there is no stall detection. A mode, not a fault.

Safety behaviour (firmware): the motors **stay off** until a valid `CONFIG_SET` has arrived, `enable` is set, a
`CMD_WHEELS` arrived within `watchdog_ms`, all four encoders read OK, and no e-stop/stall fault is latched.
Three things latch `MOTOR_FAULT` until `CLEAR_FAULT`: a stall (≥ 35 % duty and < 0.05 rad/s for 1 s), a **runaway** (driven toward a target while the encoder reports motion the opposite way for 1 s: this is what a wrong `enc_invert_mask` looks like), and gross overspeed (> 1.5 × `max_wheel_rad_s` for 200 ms). A single encoder read glitch (< 3 consecutive bad reads, about 15 ms) is tolerated.
Left side = wheels 0 and 2, right side = wheels 1 and 3.

**Wiring the firmware assumes (from your old `Teensy_code_dc_motor.ino`):** FL IN 14/15 EN 9, FR IN 18/19 EN 2, RL IN 17/16 EN 10, RR IN 20/21 EN 3 (IN_A/IN_B + PWM enable per motor), servos on 7 and 8, Nano IMU on Serial1 at 230400 baud sending `$` + ax ay az (g) + gx gy gz (deg/s) + XOR of the 24 payload bytes. Your old `LEFT_INVERT = -1` is `motor_invert_mask = 0b0101`. Pins 18/19 are the Teensy's default I2C pins, so the AS5600/TCA9548A bus is on **`Wire2` (pins 25 SDA / 24 SCL)**: verify on your pinout card. E-stop input is pin 32 (pulled up, LOW = pressed).

Time mapping: `t_us` wraps every 71.6 min (`MicrosUnwrapper` handles it). The bridge node (**[planned]**) computes
`offset = ros_now − t_us` on the first packets and tracks the minimum to remove USB jitter; sample time is `t_us`, not arrival time.

## 7. What is tested, and how

| Area | Command | What it proves |
|---|---|---|
| Reconstruction geometry + node glue | `cd src/dent_mapping && python -m pytest test -q` | Real intrinsics used; keypoint scaling; pose chain vs ground truth (≤ 4 mm / 0.3° per pair on synthetic data); frame-0 dents marked; failed pair splits segments; report written |
| Protocol | `cd src/ugv_hardware && python -m pytest test/test_protocol.py -q` | CRC, framing, resync, C++↔Python byte-exact agreement |
| Firmware logic (host) | `cd src/ugv_hardware && python -m pytest test/test_firmware_host.py -q` | Real `.ino` code against a simulated motor/encoder/IMU/e-stop: PID tracking, 12-bit wrap, watchdog, e-stop latch, stall, wrong-encoder-sign runaway, I2C glitch handling, IMU parse + resync, servo clamping, open-loop build. Also the fixed legacy sketch (command parser + watchdog). Mutation-checked: removing any one safety feature makes a check fail. **Not** a hardware test |

**Not yet verified on real data/hardware:** SuperGlue/TEASER++ on your frames, the Teensy firmware on a Teensy, and any threshold in `RegConfig` (they are starting values).

## 8. Open items — need your input **[TBD]**

Known now (from your old sketch, no longer open): motor pin map and driver scheme, servo pins/limits, IMU packet format and baud.

1. Wheel diameter (m) and track width (m, left–right wheel centres).
2. Encoder ticks per **wheel** revolution = 4096 × (gear ratio encoder-gear : wheel). Which shaft does each magnet sit on?
3. Are the 4 AS5600 + TCA9548A mounted yet? If not, build with `ENCODERS_FITTED 0` for now. Once mounted, find each `enc_invert_mask` bit by turning the wheel forward by hand and watching the tick count (it must increase); a wrong bit is caught by the runaway latch but should not be relied on.
4. Lidar model (selects the driver) and its mount pose.
5. `base_link→camera_link`, `lidar_link`, `imu_link`: x, y, z (m) and roll/pitch/yaw (deg), measured from CAD/calipers.
6. Jetson↔Teensy link: stay on USB serial now, or move to UART (changes one `#define`).
7. E-stop wiring (signal pin 32 assumed) and battery divider on a free ADC pin (pin 14 is a motor pin here).
8. Capture resolution: the pipeline supports any size as long as `intrinsics.json` matches. 640×480 for speed, or 1280×720 for depth accuracy?
9. Does your Jetson teleop publish continuously (≥ 10 Hz, including zeros when idle)? The watchdog stops the motors if commands go quiet for 500 ms (legacy sketch) / 250 ms (v2).
