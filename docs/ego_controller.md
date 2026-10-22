# EgoPlanner command adapter

Implementation: `src/px4_bridge/px4_bridge/ego_position_command_bridge.py`.

This is a message adapter, not a new low-level flight controller. It forwards
`quadrotor_msgs/PositionCommand` to `px4_bridge_msgs/RealtimeControl`; the existing
PX4 bridge handles Offboard mode and setpoint publication. It does not arm the
vehicle. Do not run two adapters or competing realtime command publishers.

## Contract

- Input default: `/drone_0_planning/pos_cmd`; output:
  `/px4_bridge/in/realtime_control`.
- Only READY commands are forwarded. Position, velocity, acceleration, yaw and
  yaw rate are copied without coordinate transformation. PX4 bridge expects ENU.
- `expected_frame` defaults to `world`. This checks the label only, not whether
  the numbers actually use ENU.
- A nonzero ROS timestamp is required. Commands older than `max_command_age_s`
  (default 0.3 s), or ahead by more than `future_tolerance_s` (default 0.05 s),
  are dropped. Publishers and this node must use the same clock configuration.
- NaN, infinity and values outside float32 range are dropped without crashing.
- DDS input and output queues keep only the latest message.
- No repeat timer refreshes old commands. The existing bridge's configured
  realtime timeout handles loss of incoming commands (default 300 ms).
- Velocity and acceleration forwarding can be disabled with `publish_velocity`
  and `publish_acceleration`. The existing bridge clips velocity components.

## Unresolved integration constraints

The current `getLidarData.py` publishes AirSim positions as `(x, -y, -z)`, which
is not standard ENU. Do not interpret the adapter's `world` label check as a
coordinate alignment fix. Align planner coordinates, external odometry, PX4's
estimated local origin/heading and yaw before flight testing. Changing only
command axes can make an already misaligned estimator worse.

The adapter does not monitor planner liveness. A surviving traj_server may keep
publishing fresh-stamped commands after the planner dies; timestamp validation
alone cannot detect that. Resolve planner failures before resuming control.

## Verification

Build from the workspace root after sourcing ROS and the workspace:

```bash
cd ~/1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select px4_bridge --symlink-install
```

Run the regression tests in a separate DDS domain (no PX4 node is started):

```bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  python3 ~/1_ws/src/ris-ctrl/src/px4_bridge/test/test_ego_position_command_bridge.py
```

Tests cover field mapping, READY filtering, NaN/Inf/float32 overflow rejection,
old/zero/future timestamps, frame rejection and actual ROS message roundtrip.
These tests do not establish flight stability or estimator-frame alignment.

Code changes require restarting the existing adapter process to take effect.
Do not start a second adapter alongside the old process.
