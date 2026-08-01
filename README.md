# robstride_sdk

## 1. Introduction

Standalone C++17 SDK for the RobStride CAN actuator private protocol (RS00/RS01/RS02/RS03/RS04/RS05/RS06). No ROS or rclcpp dependency — this package only talks to SocketCAN. It is consumed by [robstride_hardware_interface](https://github.com/kmj060703/robstride_hardware_interface), the same way [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) is consumed by `dynamixel_hardware_interface`.

Designed to sustain a **≥300 Hz** control loop against multiple motors on a single bus:

- One non-blocking `SOCK_RAW`/`CAN_RAW` socket per CAN interface, with `SO_SNDBUF`/`SO_RCVBUF` tuned to 1 MiB.
- A dedicated background read thread per bus, driven by `epoll` (level-triggered) and draining frames with `recvmmsg()` in batches (up to 64 frames/syscall) instead of one blocking `recv()` per frame.
- `SendFrames()` submits a whole batch of outgoing motion-control frames with a single `sendmmsg()` call instead of one `write()` per motor.
- Each motor's decoded state (`MotorState`) is a set of plain `std::atomic` fields, written only by the read thread and read by the control loop — no locks, no waiting for CAN I/O on the `read()`/`write()` hot path.

## 2. Installation

Usually pulled in as a dependency of [robstride_hardware_interface](https://github.com/kmj060703/robstride_hardware_interface) — see that package's README for the full multi-repo clone/build. To build this package on its own:

```bash
cd ~/${WORKSPACE}/src
git clone https://github.com/kmj060703/RobstrideSDK.git

cd ~/${WORKSPACE}
colcon build --packages-select robstride_sdk
source install/setup.bash
```

## 3. Layout

```
include/robstride_sdk/
  robstride_protocol.hpp   Wire-format constants: comm types, per-model limits (position/velocity/
                            torque/kp/kd/current), parameter index table, fault bits, float<->uint16
                            quantization, extended-ID pack/parse.
  robstride_motor.hpp       RobstrideMotor: per-motor frame encoders (motion command, enable/disable,
                            set-zero, param read/write) and HandleFrame() decoder. MotorState: the
                            lock-free telemetry snapshot (position/velocity/torque/temperature/
                            run_state/fault/last_update_ns).
  can_bus.hpp               CanBus: one SocketCAN interface, epoll read thread, sendmmsg/recvmmsg.
src/
  can_bus.cpp               CanBus implementation.
```

## 4. Building

Standard `ament_cmake` package:

```bash
cd ~/${WORKSPACE}
colcon build --packages-select robstride_sdk
source install/setup.bash
```

Consume it from another `ament_cmake` package exactly like `dynamixel_sdk`/`hardware_interface` are consumed elsewhere in this workspace — via the exported target list, not a namespaced target:

```cmake
find_package(robstride_sdk REQUIRED)
...
target_link_libraries(your_target ${robstride_sdk_TARGETS})
target_include_directories(your_target PRIVATE ${robstride_sdk_INCLUDE_DIRS})
```

## 5. Actuator models and limits

`GetActuatorLimits(ActuatorType)` returns the quantization range (position/velocity/torque/kp/kd/current) used to encode/decode Type-1 (motion control) and Type-2 (feedback) frames, sourced from the official RobStride user manuals. Position range is ±4π for every model; the rest differ per model (RS00/RS01/RS02/RS03/RS04/RS05/RS06).

`ActuatorTypeFromString()` accepts `"00"`..`"06"`, bare `"0"`..`"6"` (xacro numeric literals drop leading zeros), or `"RS00"`..`"RS06"`.

## 6. CAN interface setup

RobStride motors run at 1 Mbps CAN 2.0 (extended/29-bit ID). Bring the interface up with the tuning [robstride_hardware_interface/scripts/setup_can.sh](https://github.com/kmj060703/robstride_hardware_interface/blob/main/scripts/setup_can.sh) applies (bitrate, `restart-ms`, `txqueuelen`) before opening a `CanBus` against it.

## 7. License

Apache License 2.0. See [LICENSE](LICENSE) if present, or the workspace root.
