# Distributed Digit Relay (ROS 2)

A ROS 2 Python package where **nine independent member nodes** coordinate through a shared
turn-token topic to draw digits sequentially on a single turtlesim canvas. The relay runs
**two full cycles**: each member draws its assigned single-digit glyph in cycle one, then
draws its assigned two-digit glyph in cycle two. All nine members draw in parallel processes
and communicate only through ROS topics—no shared memory, no direct calls between members.

For full technical details, architecture, flow diagrams, and module reference, see
[`doc/documentation.md`](doc/documentation.md).

---

## Requirements

| Dependency | Version |
|---|---|
| ROS 2 | Jazzy (or compatible) |
| Python | 3.10+ |
| turtlesim | Bundled with ROS 2 desktop |

---

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws

# Remove stale build artifacts from any previous build
rm -rf build/digit_relay install/digit_relay

colcon build --packages-select digit_relay
source install/setup.bash
```

---

## Run

```bash
ros2 launch digit_relay relay.launch.py
```

This starts turtlesim and all nine member nodes simultaneously. The relay runs
automatically—no manual input is needed. The launch exits after Member 09 completes
the final turn (turn 18).

---

## Inspect While Running

Open a second terminal with the workspace sourced:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws
source install/setup.bash

ros2 node list
ros2 topic info /digit_relay/turn --verbose
ros2 topic echo /digit_relay/turn
ros2 topic echo /digit_relay/status
ros2 service list
```

---

## What You Will See

A 3×3 grid of digit drawings appears on the turtlesim canvas as the relay progresses:

| Slot (row, col) | Member | Cycle 1 | Cycle 2 |
|---|---|---|---|
| (1,1) | Member 01 | `1` | `10` |
| (1,2) | Member 02 | `2` | `11` |
| (1,3) | Member 03 | `3` | `12` |
| (2,1) | Member 04 | `4` | `13` |
| (2,2) | Member 05 | `5` | `14` |
| (2,3) | Member 06 | `6` | `15` |
| (3,1) | Member 07 | `7` | `16` |
| (3,2) | Member 08 | `8` | `17` |
| (3,3) | Member 09 | `9` | `18` |

The canvas is cleared between cycle one and cycle two. Each member draws with its own
distinct pen color.
