# Distributed Digit Relay — Technical Documentation

## 1. Project overview

Distributed Digit Relay is a ROS 2 Python package that coordinates independent
member nodes to draw digits in turtlesim. Each member owns its topic callbacks,
service clients, pen configuration, and glyph implementation. The abstract base
class defines the required member interface and chains asynchronous service and
drawing operations. A shared drawing controller is available for members that
want pose-feedback motion without duplicating the low-level controller.

The repository currently contains a **single-member runnable example**:

1. Member 01 waits for its turn-topic subscription to be ready and publishes
   token `[1, 1]`.
2. Member 01 configures its pen and draws glyph `1` in slot 1.
3. It publishes `[2, 1]` for the second cycle.
4. Before drawing again, it calls `/clear`, lifts its pen, and draws glyph `1`.
5. Member 01 shuts ROS down after turn 2; the launch event then closes
   turtlesim.

`member_02.py` and `member_03.py` are intentionally absent. `member_xx.py` is a
non-launched starter template for teammates. The running example uses
`TOTAL_MEMBERS = 1`; the project must be integrated and configured consistently
before additional members are launched.

## 2. Repository layout

```text
distributed-digit-relay/
├── .gitignore
├── README.md
├── doc/
│   └── documentation.md
└── src/
    └── digit_relay/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/digit_relay
        ├── launch/relay.launch.py
        └── digit_relay/
            ├── __init__.py
            ├── relay_node.py
            ├── digit_drawing.py
            ├── digit_paths.py
            ├── member_01.py
            └── member_xx.py
```

| File | Responsibility |
|---|---|
| `relay_node.py` | Abstract node contract, shared identity/slot state, asynchronous hook sequencing, and spin/cleanup entry point. It creates no topics or service clients. |
| `digit_drawing.py` | Reusable glyph-to-slot mapping and pose-feedback motion controller. Concrete member classes create the ROS publishers and subscriptions it uses. |
| `digit_paths.py` | Canvas dimensions, grid constants, `DigitSlot`, and the `slot_for_member()` allocator. |
| `member_01.py` | Runnable member implementation: own topic endpoints and service clients, turn state machine, glyph configuration, and startup/shutdown behavior. |
| `member_xx.py` | Contributor template. Copy it to a uniquely named `member_NN.py`; do not launch the template as-is. |
| `launch/relay.launch.py` | Starts turtlesim and the currently implemented Member 01, then shuts down the launch when that member exits. |
| `setup.py` | Installs package metadata, launch files, and the `digit_relay_member_01` executable. |
| `package.xml` | Declares ROS package metadata and runtime dependencies. |
| `.gitignore` | Excludes ROS build products, Python caches, virtual environments, and common editor/OS files from Git. |

## 3. Execution model and communication

### Turn topic

Members coordinate on `/digit_relay/turn`, type
`std_msgs/msg/Int32MultiArray`:

```text
data[0] = current_number
data[1] = active_member
```

For example, `[2, 1]` means the second number/turn is assigned to Member 01.
Each concrete member is expected to create its own publisher and subscriber.
The topic uses reliable, volatile QoS with depth 1. Volatile durability prevents
multiple publishers from replaying their individual stale tokens to a late
subscriber. In a multi-member configuration, the integration owner must ensure
that every member subscription is discovered before Member 01 publishes the
initial token; the current Member 01 checks publisher subscription count.

### Status topic

`/digit_relay/status`, type `std_msgs/msg/String`, is for human-readable
diagnostics, not turn coordination. The template includes a publisher. Useful
states include `WAITING_FOR_TURN`, `WAITING_FOR_MEMBER_XX`, `DRAWING`, `ERROR`,
and `COMPLETE`. In the one-member demo, turn 1 and turn 2 are both assigned to
Member 01, so no other member waits.

### Turtle topics and services

| Name | Type | Purpose |
|---|---|---|
| `/turtle1/pose` | `turtlesim/msg/Pose` | Feedback for turtle position and heading. |
| `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | Motion commands while a member is drawing or moving to its slot. |
| `/turtle1/set_pen` | `turtlesim/srv/SetPen` | Set pen RGB/width and lower (`off=False`) or lift (`off=True`) the pen. Each member owns its own client. |
| `/clear` | `std_srvs/srv/Empty` | Clear the canvas before cycle two. Member 01 calls it at the beginning of turn 2 in this one-member configuration. |

The member calls services asynchronously. If a service is not ready, a
`STEADY_TIME` timer checks service availability until it can issue the request;
there is no fixed sleep used to assume readiness. Drawing motion is driven by
pose feedback rather than a guessed duration.

## 4. Object-oriented contracts and available methods

### `DigitRelayNode` — `relay_node.py`

This abstract class extends `rclpy.node.Node`. It stores the member identity,
group size, assigned `DigitSlot`, and busy flag. It deliberately does not own
the turn topic or any member-specific service client.

| Method | Kind | Contract / behavior |
|---|---|---|
| `__init__(member_id, total_members)` | Concrete | Names the ROS node, stores IDs, and calculates the assigned slot. |
| `prepare_turn(number)` | Abstract | Member-specific asynchronous service preparation before a glyph is drawn. Must return a `Future`. |
| `set_pen(off=...)` | Abstract | Member-specific `/turtle1/set_pen` call. Must return a `Future`. |
| `draw_digit(number, slot)` | Abstract | Member-specific drawing hook. The `Future` must complete only when drawing is finished. |
| `execute_turn(number)` | Concrete | Calls `prepare_turn()`, then calls `draw_digit()` after preparation succeeds. Returns a Future representing the full turn. |
| `_fail_turn(reason)` | Concrete helper | Clears the busy flag and logs failure. A failed turn must not pass its token. |
| `run_member(member_type)` | Module function | Initializes ROS, constructs and spins one member node, then destroys the node and shuts ROS down if still active. |

Implementations should keep the ROS event loop non-blocking: return Futures for
service and drawing work instead of calling `sleep()` or waiting synchronously.

### `DigitDrawingMember` — `digit_drawing.py`

This helper subclass adds glyph configuration and the pose-feedback drawing
entry point. It expects each concrete member to create `_cmd_pub` and subscribe
to pose with `_on_pose`.

| Method / helper | Behavior |
|---|---|
| `__init__(...)` | Stores pen color, glyph label, allowed turns, normalized strokes, and pose/job state. |
| `_on_pose(pose)` | Stores the latest pose; logs when feedback first arrives. |
| `_stop_turtle()` | Publishes a zero `Twist` command. |
| `_draw_configured_glyph(number, slot)` | Validates the turn, maps normalized strokes into the slot, starts `_DigitDrawingJob`, and returns a completion Future. |
| `_normalize_angle(angle)` | Wraps heading error into `[-π, π]`. |
| `_map_glyph_to_slot(strokes, slot)` | Scales normalized glyph points into a safe area inside the member's slot. |

The internal `_DigitDrawingJob` has these steps:

| Method | Behavior |
|---|---|
| `_set_pen(off, next_state)` | Requests a pen state change and proceeds after its Future completes. |
| `_pen_done(future, next_state)` | Advances the motion state or fails the drawing Future. |
| `_tick()` | Timer callback: waits for pose and a turtlesim command subscriber, then advances the current stroke state. |
| `_drive_to(target, pose)` | Computes heading and distance errors, publishes bounded velocity, and reports when the target is reached. |
| `_finish()` | Stops the turtle, cancels the timer, and completes the Future successfully. |
| `_fail(error)` | Stops the turtle, cancels the timer, and completes the Future with an error. |

### `Member01` — `member_01.py`

| Method | When / what it does |
|---|---|
| `__init__()` | Calls the drawing base constructor; creates its own service clients, motion endpoints, turn endpoints, and status publisher; publishes an initial waiting state and starts readiness discovery. |
| `_publish_initial_turn()` | Timer callback. Once its expected turn subscription count is present, cancels the timer and publishes `[1, 1]`. |
| `_on_turn(message)` | Validates the token, stays idle for a token assigned elsewhere, rejects invalid/duplicate/busy turns, and starts `execute_turn()` for its own valid turn. |
| `_after_turn(future, current_number)` | Handles errors; after success publishes the next token, or marks complete and shuts down after the final turn. |
| `_publish_turn(current_number, active_member)` | Publishes the two integer fields on `/digit_relay/turn`. |
| `_publish_status(state, current_number, active_member)` | Publishes diagnostics to `/digit_relay/status` and logs the same status. |
| `set_pen(off=...)` | Builds a SetPen request using Member 01's RGB color and width, then calls its service helper. |
| `prepare_turn(number)` | Lifts the pen before drawing; on the first turn of cycle two, first calls `/clear` and waits for it to complete. |
| `_call_service_when_ready(client, request, service_name)` | Waits asynchronously for service discovery, calls `call_async()`, and resolves a standard Future from the ROS response Future. |
| `draw_digit(number, slot)` | Invokes the shared drawing controller for Member 01's glyph 1. |
| `main()` | Starts this node through `run_member(Member01)`. |

Current constants are `MEMBER_ID = 1`, `TOTAL_MEMBERS = 1`, and
`VALID_TURNS = (1, 2)`. Thus turn 2 is both the beginning of cycle two and the
final turn. Its preparation calls `/clear` before drawing glyph 1 again.

## 5. Member 01 call sequence

```text
relay.launch.py
  -> turtlesim_node + digit_relay_member_01
  -> member_01.main()
  -> run_member(Member01)
  -> rclpy.init()
  -> Member01.__init__(): create clients/endpoints, begin discovery check
  -> rclpy.spin()
  -> _publish_initial_turn(): publish [1, 1] once subscription is ready
  -> _on_turn([1, 1])
  -> execute_turn(1)
       -> prepare_turn(1) -> set_pen(off=True)
       -> draw_digit(1, slot) -> _draw_configured_glyph() -> movement/drawing
  -> _after_turn(): publish [2, 1]
  -> _on_turn([2, 1])
  -> execute_turn(2)
       -> prepare_turn(2) -> /clear -> set_pen(off=True)
       -> draw_digit(2, slot) -> draw glyph 1 again
  -> _after_turn(): publish COMPLETE, call rclpy.shutdown()
  -> run_member() destroys the node
  -> launch event shuts down turtlesim
```

The executor is how a node waits: `rclpy.spin()` keeps it alive while it
processes incoming messages, timers, and service responses. It does not need a
blocking wait loop. In the future multi-member configuration, a member whose
ID does not match `active_member` should publish its waiting status and return
from `_on_turn()` without calling `execute_turn()`.

## 6. Contributor instructions for `member_xx.py`

Please treat `member_xx.py` as a **shared starter template**, not as your work
file. Keeping one canonical template unchanged lets everyone start from the
same contract and avoids teammates overwriting each other's edits. Follow this
sequence:

1. **Create your own file.** Copy the template to a unique name, for example
   `member_02.py` or `member_07.py`. Do not rename or edit another teammate's
   member file.
2. **Choose your identity.** Set `MEMBER_ID` to your assigned 1-based member
   number and `TOTAL_MEMBERS` to the agreed group size. These values must match
   the integration configuration. Do not guess or independently change the
   group size.
3. **Set your two turn numbers.** The template formula is
   `(MEMBER_ID, TOTAL_MEMBERS + MEMBER_ID)`: one turn in each cycle. Confirm the
   group's agreed numbering scheme before changing it.
4. **Choose a unique pen color.** Set `PEN_COLOR` to an RGB tuple with each
   channel in the 0–255 range. Coordinate colors with the group so each member
   is visually distinct.
5. **Define a glyph.** Add normalized stroke paths, where each point is an
   `(x, y)` pair between 0 and 1. To reuse the shared drawing controller, change
   the template class to inherit from `DigitDrawingMember`, pass your glyph and
   stroke paths to that class's constructor, and implement `draw_digit()` by
   calling `_draw_configured_glyph(number, slot)`. Alternatively, implement an
   equivalent pose-feedback routine yourself. Keep the glyph inside its slot
   and return a Future that completes only when the turtle has stopped.
6. **Keep member-owned ROS code in your file.** Create your own `/set_pen`
   client, `/cmd_vel` publisher, `/pose` and turn subscribers, turn publisher,
   and status publisher. Implement `_on_turn()`, `_after_turn()`, service
   readiness handling, `prepare_turn()`, and `set_pen()` for your node. Do not
   put your service client or topic callbacks in `relay_node.py`.
7. **Preserve the turn protocol.** Subscribe to `/digit_relay/turn` and read
   `[current_number, active_member]`. If the token is for another member,
   publish a waiting status and return without moving. If it is yours, reject
   duplicate/invalid tokens, run `execute_turn()`, and publish the next token
   only after your drawing Future succeeds. Do not pass the token on failure.
8. **Handle the last cycle correctly.** Only the node responsible for
   `2 * TOTAL_MEMBERS` should publish `COMPLETE` and request ROS shutdown. The
   launch integration must monitor that same final executable.
9. **Ask the integration owner to wire it in.** The owner should add your
   console entry in `setup.py`, add your node to `launch/relay.launch.py`,
   update the initial-subscription readiness count, align all members'
   `TOTAL_MEMBERS`, and select the final-node shutdown target. Avoid editing
   those shared files in parallel unless the owner assigns that task to you.
10. **Verify locally.** Build the package and run the one-command launch after
    integration. Confirm node/topic endpoints, turn sequence, wait/draw status,
    unique pen color, clear between cycles, and automatic shutdown.

The template uses `raise NotImplementedError` in `draw_digit()` deliberately:
it prevents an incomplete copy from appearing to implement drawing. Fill in
the glyph/controller work before registering the new executable in the launch.

## 7. Git collaboration without avoidable conflicts

Git can merge work cleanly when contributors work in separate files and keep
shared integration changes coordinated. A recommended division is:

| Contributor | Files they should normally own |
|---|---|
| Each member | Only their new `member_NN.py` file, created from the template. |
| Integration owner | `setup.py`, `launch/relay.launch.py`, shared member-count/turn configuration, and final documentation. |
| Group together | Agree IDs, colors, glyph assignments, turn numbering, and shutdown owner before implementation. |

### Repository owner: publish the project first

The project directory is not currently initialized as a Git repository. Create
an empty GitHub repository (without an automatically generated README or
license), then publish the project once:

```bash
cd /path/to/distributed-digit-relay
git init -b main
git add .
git commit -m "Add distributed digit relay ROS 2 package"
git remote add origin https://github.com/USERNAME/REPOSITORY.git
git push -u origin main
```

Replace the URL with the group's repository URL. Configure the commit identity
for each contributor before committing:

```bash
git config user.name "Your Name"
git config user.email "you@example.com"
```

### Per-member workflow

Start from the latest shared branch and create a named branch:

```bash
git switch main
git pull --ff-only
git switch -c member-02-glyph
```

Create and edit only your own member file, then commit that focused change:

```bash
git add src/digit_relay/digit_relay/member_02.py
git commit -m "Add Member 02 digit drawing node"
git push -u origin member-02-glyph
```

Open a pull request into `main`. The integration owner reviews and merges it,
then performs the shared `setup.py` and launch wiring in a separate integration
change. Replace `02` and the branch name with your assigned member number.

### Practices that prevent conflicts

- Do not have every teammate edit `member_xx.py`; copy it and edit a unique
  `member_NN.py` file.
- Do not have multiple people edit `setup.py`, `relay.launch.py`, or the same
  documentation section at once. Assign one integration owner for those files.
- Pull the latest `main` before creating a branch. If your branch becomes old,
  coordinate with the integration owner, then rebase or merge the updated
  `main` before requesting review.
- Keep each commit focused on one member or one integration task. Avoid mixing
  unrelated formatting changes across everyone else's files.
- Do not commit `build/`, `install/`, `log/`, `__pycache__/`, or editor files;
  `.gitignore` already excludes these common generated files.
- Use your own Git author identity so the commit history records who made each
  contribution: `git config user.name` and `git config user.email`.
- If the same shared file must be changed by two people, stop parallel edits
  and have the integration owner combine the changes in sequence. A merge
  conflict is resolved by agreeing on the intended final content, not by
  blindly choosing one side.

## 8. Build and run the current one-member example

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws
rm -rf build/digit_relay install/digit_relay
colcon build --packages-select digit_relay
source install/setup.bash
ros2 launch digit_relay relay.launch.py
```

If the workspace previously installed Member 02/03 executables, remove the
package-specific build/install directories as shown above so those stale
scripts do not remain in the install space. This cleanup does not remove other
packages from the workspace. The launch starts turtlesim and Member 01; it
exits automatically after two drawings. To inspect the graph while it is
running, use another terminal with the ROS environment sourced:

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

The current demo has one turn publisher and one turn subscriber because it has
one participant. Once members are integrated, endpoint counts should match the
number of configured members. The status topic reports each member's observed
state while the launch is active.

## 9. Extending the example to a group

Adding member files alone is not sufficient to create a distributed relay.
Before running a multi-member group, the integration owner must ensure that:

- every member agrees on `TOTAL_MEMBERS` and has a unique ID and slot;
- turn numbers cover exactly two cycles and map to the intended member;
- Member 01 emits the initial token only after all turn subscriptions are
  discovered;
- each member has one console script and one launch action;
- all members create compatible volatile turn-topic endpoints;
- cycle-two clearing happens once, before the first second-cycle glyph;
- the last member owns the final turn and is the launch shutdown target.

Until those shared changes are integrated, the checked-in launch intentionally
runs only the single-member Member 01 demonstration.
