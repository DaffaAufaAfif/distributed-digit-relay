# Distributed Digit Relay (ROS 2)

A ROS 2 Python package for coordinating turtlesim digit drawing through member
nodes and a turn topic. The repository currently includes one runnable node,
Member 01, which draws glyph `1` in two cycles, calls `/clear` before cycle two,
and exits automatically. `member_xx.py` is a starter template for teammates;
it is not registered in the launch file or executable list.

## Documentation

The detailed guide is in [`doc/documentation.md`](doc/documentation.md). It
describes the node lifecycle, available methods, ROS topics and services,
member implementation checklist, and Git collaboration workflow.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws
rm -rf build/digit_relay install/digit_relay
colcon build --packages-select digit_relay
source install/setup.bash
ros2 launch digit_relay relay.launch.py
```

The package-specific cleanup removes stale executables left by an earlier
multi-member build. It does not remove other packages from the workspace.

## Inspect ROS communication

While the launch is running, use another terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws
source install/setup.bash
ros2 node list
ros2 topic info /digit_relay/turn --verbose
ros2 topic echo /digit_relay/turn
ros2 topic echo /digit_relay/status
```

See the [technical documentation](doc/documentation.md) for the method
reference, execution flow, contributor checklist for `member_xx.py`, and Git
workflow designed to keep parallel contributions from conflicting.
