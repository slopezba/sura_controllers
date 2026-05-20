# sura_controllers

ROS 2 Humble controllers for SURA vehicles.

This package merges the previous AUV and USV controller plugins into one package:

- common controllers shared by vehicle types
- AUV controllers
- USV controllers

The goal is to keep the controller code in one place while preserving clear ownership of vehicle-specific behavior.

## File Map

Key package files:

- [`package.xml`](/home/cirtesu/cirtesub_ws/src/sura_controllers/package.xml)
- [`CMakeLists.txt`](/home/cirtesu/cirtesub_ws/src/sura_controllers/CMakeLists.txt)
- [`sura_controllers_plugins.xml`](/home/cirtesu/cirtesub_ws/src/sura_controllers/sura_controllers_plugins.xml)

Controller sources:

- `src/common_controllers`: controllers shared by AUV and USV
- `src/auv_controllers`: AUV-specific controllers
- `src/usv_controllers`: USV-specific controllers

Controller headers:

- `include/sura_controllers/common`
- `include/sura_controllers/auv`
- `include/sura_controllers/usv`

The general rule is:

- `.hpp`: declares the controller class, members, parameters, and realtime state
- `.cpp`: implements lifecycle hooks and control behavior
- `sura_controllers_plugins.xml`: exposes controller classes to `pluginlib`
- `CMakeLists.txt`: builds the shared library and exports the plugin description
- `package.xml`: declares build and runtime dependencies

## ROS 2 Control Overview

`ros2_control` connects three pieces:

- robot hardware or simulator interfaces
- controller plugins
- the ROS graph

At runtime, `controller_manager` loads plugins, configures them, activates them, and calls their update loop at the configured rate.

Controllers in this package are loaded by type names from YAML files, for example:

```yaml
body_force:
  type: sura_controllers/common/BodyForceController
```

Those type names must exist in [`sura_controllers_plugins.xml`](/home/cirtesu/cirtesub_ws/src/sura_controllers/sura_controllers_plugins.xml).

## Plugin Layout

Current plugins:

| Plugin type | Class | Purpose |
| --- | --- | --- |
| `sura_controllers/common/BodyForceController` | `sura_controllers::common::BodyForceController` | Convert a body wrench into thruster efforts |
| `sura_controllers/auv/BodyForceController` | `sura_controllers::common::BodyForceController` | Alias for AUV configs |
| `sura_controllers/usv/BodyForceController` | `sura_controllers::common::BodyForceController` | Alias for USV configs |
| `sura_controllers/auv/BodyVelocityController` | `sura_controllers::auv::BodyVelocityController` | AUV 6-DoF velocity controller |
| `sura_controllers/auv/StabilizeController` | `sura_controllers::auv::StabilizeController` | AUV attitude stabilization |
| `sura_controllers/auv/DepthHoldController` | `sura_controllers::auv::DepthHoldController` | AUV depth and yaw hold |
| `sura_controllers/auv/PositionHoldController` | `sura_controllers::auv::PositionHoldController` | AUV position hold |
| `sura_controllers/auv/StateObserverController` | `sura_controllers::auv::StateObserverController` | AUV state observer and model identification |
| `sura_controllers/auv/Mpc4dofController` | `sura_controllers::auv::Mpc4dofController` | AUV 4-DoF MPC |
| `sura_controllers/usv/BodyVelocityController` | `sura_controllers::usv::BodyVelocityController` | USV surge and yaw-rate controller |
| `sura_controllers/usv/BodyPositionController` | `sura_controllers::usv::BodyPositionController` | USV position-to-velocity controller |

## Topic Mode And Chained Mode

Most controllers inherit from:

```cpp
controller_interface::ChainableControllerInterface
```

That gives two operating modes:

- topic mode: the controller receives commands from ROS topics
- chained mode: another controller writes directly into exported reference interfaces

Typical AUV chain:

```text
position_hold -> body_velocity -> stabilize/depth_hold -> body_force -> thrusters
```

Typical USV chain:

```text
body_position -> body_velocity -> body_force -> thrusters
```

The chain depends on controller names. If a controller consumes references from `body_force`, it requests interfaces such as:

```text
body_force/force.x
body_force/torque.z
```

So YAML names and `body_*_controller_name` parameters must match.

## Common Controller

### `BodyForceController`

Location:

- [`src/common_controllers/body_force_controller.cpp`](/home/cirtesu/cirtesub_ws/src/sura_controllers/src/common_controllers/body_force_controller.cpp)
- [`include/sura_controllers/common/body_force_controller.hpp`](/home/cirtesu/cirtesub_ws/src/sura_controllers/include/sura_controllers/common/body_force_controller.hpp)

Purpose:

Convert a desired body wrench into individual thruster effort commands.

How it works:

- Reads `robot_description`
- Parses thruster joints from the `<ros2_control>` section
- Finds each thruster pose relative to `base_link`
- Builds the thruster allocation matrix
- Computes the pseudoinverse once during configure
- Converts wrench references into thruster efforts during update

Input:

- Topic mode: `geometry_msgs/msg/Wrench`
- Chained mode: exported wrench references

Exported references:

- `force.x`
- `force.y`
- `force.z`
- `torque.x`
- `torque.y`
- `torque.z`

Output:

- One `effort` command per thruster joint

Debug:

- Publishes `sura_msgs/msg/ControllerDebug` when `debug.enabled` is true
- Uses atomics in the update loop
- Publishes debug stats from a timer outside the realtime update path

## AUV Controllers

### `BodyVelocityController`

Purpose:

Track 6-DoF body velocity references and output body wrench references.

Input:

- `geometry_msgs/msg/Twist` setpoint in topic mode
- navigator state for measured body velocity and acceleration

Output:

- Chained body wrench references for `BodyForceController`

Notes:

- Runs PID loops for linear `x/y/z` and angular `roll/pitch/yaw`
- Supports debug telemetry and realtime-safe timing counters

### `StabilizeController`

Purpose:

Stabilize roll, pitch, and yaw while passing translational feedforward to the next stage.

Input:

- body wrench feedforward
- navigator attitude and angular velocity

Output:

- body wrench references

### `DepthHoldController`

Purpose:

Hold depth and yaw while allowing XY feedforward.

Input:

- body wrench feedforward
- navigator depth and attitude state

Output:

- body wrench references

### `PositionHoldController`

Purpose:

Generate body velocity commands to hold or reach a 3D pose.

Input:

- pose setpoint
- optional velocity feedforward
- navigator pose and velocity

Output:

- body velocity references

### `StateObserverController`

Purpose:

Estimate AUV state and identify a diagonal dynamic model.

Input:

- odometry-like navigation sources
- IMU and wrench data

Output:

- estimated state and model-related messages

### `Mpc4dofController`

Purpose:

Compute 4-DoF wrench references for `x`, `y`, `z`, and yaw using MPC.

Input:

- pose references
- navigator state

Output:

- body wrench references for `BodyForceController`

## USV Controllers

### `BodyVelocityController`

Location:

- [`src/usv_controllers/body_velocity_controller.cpp`](/home/cirtesu/cirtesub_ws/src/sura_controllers/src/usv_controllers/body_velocity_controller.cpp)
- [`include/sura_controllers/usv/body_velocity_controller.hpp`](/home/cirtesu/cirtesub_ws/src/sura_controllers/include/sura_controllers/usv/body_velocity_controller.hpp)

Purpose:

Track USV surge velocity and yaw rate.

Input:

- Topic mode: `geometry_msgs/msg/Twist`
- Navigator body velocity

References:

- `linear.x`
- `angular.z`

Output:

- `force.x`
- `torque.z`
- remaining wrench axes are set to zero

Realtime notes:

- The update loop only reads pre-existing buffers, computes PID terms, writes command interfaces, and updates atomics
- Debug publishing happens from a timer, not from the realtime update loop

### `BodyPositionController`

Location:

- [`src/usv_controllers/body_position_controller.cpp`](/home/cirtesu/cirtesub_ws/src/sura_controllers/src/usv_controllers/body_position_controller.cpp)
- [`include/sura_controllers/usv/body_position_controller.hpp`](/home/cirtesu/cirtesub_ws/src/sura_controllers/include/sura_controllers/usv/body_position_controller.hpp)

Purpose:

Convert a position target into USV body velocity references.

Input:

- `geometry_msgs/msg/PoseStamped` setpoint
- navigator pose and yaw

Output:

- `linear.x`
- `angular.z`

Behavior:

- Uses APPROACH mode while far from the target
- Enters HOLD mode inside `position_hold_radius`
- Leaves HOLD mode when distance exceeds `position_release_radius`
- In HOLD mode, corrects longitudinal drift and yaw without forcing orbiting around the target

Realtime notes:

- Debug counters are preallocated atomics
- `ControllerDebug` publication happens outside the update loop
- Marker publication only happens when targets change or the controller initializes a hold target

## Debug Parameters

Controllers with debug support use:

```yaml
debug.enabled: true
debug.topic: debug
```

When enabled, the controller publishes:

- controller name
- active state
- chained mode state when applicable
- desired update period
- last update duration
- average update duration
- max update duration
- min update duration
- deadline miss count
- cycle count

The message type is:

```text
sura_msgs/msg/ControllerDebug
```

## Realtime Guidelines

The update loop should avoid:

- dynamic allocation
- blocking calls
- non-realtime publishers
- file I/O
- parameter reads
- expensive logging

Preferred patterns:

- allocate publishers, timers, vectors, and buffers during `on_configure`
- reset state during `on_activate`
- use `realtime_tools::RealtimeBuffer` for subscriber data
- use `realtime_tools::RealtimePublisher` when publishing from update
- use atomics for lightweight debug counters
- publish periodic debug summaries from timers outside the update loop

## Build

Always build from the workspace root:

```bash
cd /home/cirtesu/cirtesub_ws
colcon build --packages-select sura_controllers
```

If `ccache` points to a read-only location in a sandbox, use:

```bash
CCACHE_DIR=/tmp/ccache CCACHE_TEMPDIR=/tmp/ccache-tmp colcon build --packages-select sura_controllers
```
