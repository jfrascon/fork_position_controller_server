# joint_position_controller_server

This package exposes the movement of one scalar joint as a ROS 2 action over a topic-based backend.
It adds goal validation, feedback, cancellation, convergence checks, and terminal results to a backend that only exchanges target and measured positions through ROS topics.

## Why this package exists

`ros2_control` is the preferred framework when an application can represent the robot hardware through well-defined state and command interfaces and can manage the corresponding controller lifecycle.

This package targets integrations where the application computer does not own the actuator connection or the low-level controller.
Another component controls the physical joint and exposes only two ROS 2 interfaces: a topic that receives an absolute target position and a `sensor_msgs/msg/JointState` topic that reports the measured position.

The package adds action semantics on top of that topic-based boundary.
A client sends a target through the `JointPosition` action, and the server validates it, publishes commands continuously, reports feedback, handles cancellation, checks convergence, and produces a terminal result.

The backend remains responsible for moving the actuator, enforcing hardware safety, and publishing valid joint-state measurements.
This package is therefore an adapter for a constrained integration boundary, not a replacement for the complete `ros2_control` framework.

The package also provides example action clients and a serial backend for a prismatic joint.
The serial backend converts meter-based positions to millimeters for comparison with the sensor and then sends one-byte motor commands to the connected firmware.

## Architecture

```mermaid
flowchart LR
    client["JointPosition action client"]
    server["joint_position_controller_server"]
    backend["Position backend or prismatic serial driver"]

    client -->|"Goal"| server
    server -->|"std_msgs/msg/Float64<br/>Target position"| backend
    backend -->|"sensor_msgs/msg/JointState<br/>Measured position"| server
    server -->|"Feedback and terminal result"| client
```

The action server and every client use `joint_position_controller_interfaces/action/JointPosition`.
Positions use meters for prismatic joints and radians for revolute joints.
The configured limits, command backend, and `JointState` publisher must use the same unit.
The action server uses the relative action name `joint_position`.

## Installed executables

- `joint_position_controller_server_node`: action server and backend topic adapter.
- `prismatic_joint_position_serial_driver_node`: serial backend for a prismatic joint.
- `joint_position_controller_client_node`: single-goal example client.
- `joint_position_controller_client_single_goal_node`: alias of the single-goal example client.
- `joint_position_controller_client_single_goal_sequential_node`: two-goal sequential example.

## Action server

The server accepts one active goal at a time.
It rejects non-finite positions and positions outside `lower_limit` and `upper_limit`.
A goal succeeds when the measured position is within `position_tolerance` of its target.
A goal aborts when its execution timeout expires or its measured joint state is invalid or stale.
A canceled, timed-out, or completed goal leaves the last valid measured position as a holding command.
If the measured position is invalid, the server stops publishing because it has no safe holding position.
In that case, the action result reports `NaN` as `final_position` because no valid value is available.

The server subscribes to `joint_states_topic`, selects the entry named by `joint_name`, and publishes commands on `command_topic`.
It ignores `JointState` messages that do not contain the controlled joint and uses `joint_state_timeout` to detect missing measurements for that joint.
It publishes the requested position without applying backend-specific rounding or unit conversion.

## Prismatic serial driver

The serial driver is a backend for a prismatic joint whose ROS-facing positions use meters.
Its incoming sensor frames use millimeters, while its outgoing commands are individual ASCII bytes.

The firmware sends newline-terminated frames in this form:

```text
<motor_status>,<ir_distance_mm>\n
```

The motor status values are:

- `0`: stopped.
- `1`: moving toward the upper joint limit.
- `2`: moving toward the lower joint limit.

The driver writes one ASCII command byte using the same three values.
It deliberately writes no newline because the firmware consumes one command byte at a time.
During shutdown, the driver attempts to send `0` before closing the serial port.

`lower_ir_value` is the sensor distance measured at `lower_limit`.
`upper_ir_value` is the sensor distance measured at `upper_limit`.
The sensor values may increase or decrease with joint position, but they must be different.

The driver rejects non-finite or out-of-range targets before they reach the serial command logic.
It publishes the calibrated and filtered position through `sensor_msgs/msg/JointState`.

## Launch files

- `joint_position_controller_server.launch.py` starts the action server.
- `prismatic_joint_position_serial_driver.launch.py` starts the serial backend.

Both launch files accept an optional parameter file and explicit parameter overrides.
`params_file_allow_substs` controls whether ROS launch substitutions are evaluated in that file.
It defaults to `False` because the installed example files contain no substitutions.
`node_args` configures the corresponding ROS node action through `ros2_launch_helpers`.

Inspect all arguments with:

```bash
ros2 launch joint_position_controller_server joint_position_controller_server.launch.py --show-args
ros2 launch joint_position_controller_server prismatic_joint_position_serial_driver.launch.py --show-args
```
