# Yahboom 10-Axis IMU ROS 2 Driver

ROS 2 Humble C++ driver for the Yahboom 10-axis serial IMU. The device uses the
Yahboom/WitMotion-style binary serial protocol documented in
`docs/protocol.pdf`.

The driver reads fixed 11-byte sensor frames, validates checksums, converts core
IMU fields into ROS standard messages, and configures the device output rate on
startup.

## Features

- C++17 `ament_cmake` ROS 2 package.
- POSIX `termios` serial transport.
- No custom ROS message definitions.
- Parses documented `0x55` read frames.
- Publishes IMU data only after acceleration, angular velocity, and orientation
  are all available.
- Supports quaternion orientation frames and Euler-angle fallback.
- Sends startup write frames for output rate and bandwidth.

## Published Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/msg/Imu` | Acceleration, angular velocity, and orientation packaged together. |
| `imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetometer values are published as raw device units because the PDF does not define a Tesla scale. |
| `imu/pressure` | `sensor_msgs/msg/FluidPressure` | Pressure in Pa. |
| `imu/altitude` | `std_msgs/msg/Float64` | Altitude converted from cm to m. |
| `imu/temperature` | `std_msgs/msg/Float64` | Temperature in Celsius. |

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/dev/imu_usb` | Serial device path. |
| `baud_rate` | Node default: `9600`, launch default: `115200` | Serial baud rate. Use the launch value unless your device is configured differently. |
| `frame_id` | `imu_link` | Header frame for published sensor messages. |
| `use_euler_orientation_fallback` | `true` | If no quaternion frame is emitted, use angle frame `0x53` roll/pitch/yaw as orientation. |
| `output_rate_hz` | `100.0` | Requested device output rate. |
| `save_configuration` | `false` | If true, sends `SAVE` after startup configuration. Defaults false to avoid unnecessary flash writes. |

Supported `output_rate_hz` values are:

```text
0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200
```

Unsupported values log an error and fall back to `100 Hz`.

## Startup Device Configuration

After opening the serial port, the node writes these register commands:

| Purpose | Frame |
| --- | --- |
| Unlock writes | `FF AA 69 88 B5` |
| Set RRATE to 100 Hz by default | `FF AA 03 09 00` |
| Set BANDWIDTH to highest documented value, 256 Hz | `FF AA 1F 00 00` |
| Save configuration, only when enabled | `FF AA 00 00 00` |

The node logs each register write with `RCLCPP_INFO`.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select yahboom_10_axis_imu
source install/setup.bash
```

## Run

Recommended launch:

```bash
ros2 launch yahboom_10_axis_imu yahboom_10_axis_imu.launch.py
```

Example with explicit serial settings:

```bash
ros2 launch yahboom_10_axis_imu yahboom_10_axis_imu.launch.py \
  port:=/dev/imu_usb \
  baud_rate:=115200 \
  output_rate_hz:=100.0 \
  save_configuration:=false
```

Run the node directly:

```bash
ros2 run yahboom_10_axis_imu yahboom_10_axis_imu_node \
  --ros-args \
  -p port:=/dev/imu_usb \
  -p baud_rate:=115200 \
  -p frame_id:=imu_link
```

## Diagnostics

The node logs:

- valid acquired frame types, such as `0x51` acceleration and `0x52` angular velocity;
- IMU readiness mask updates for acceleration, gyro, and orientation;
- why `imu/data` is waiting if the readiness mask is incomplete;
- published message values, throttled to keep logs readable;
- startup register writes.

If `imu/data` is not publishing, check the readiness logs. The packet requires:

```text
0b001 acceleration
0b010 angular velocity
0b100 orientation
0b111 publish imu/data
```

If the device emits `0x53` angle frames instead of `0x59` quaternion frames,
leave `use_euler_orientation_fallback:=true`.

## Tests

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select yahboom_10_axis_imu
colcon test-result --verbose
```

Current tests cover:

- parser frame conversion and checksum handling;
- IMU readiness mask behavior;
- write-frame construction and output-rate mapping.

