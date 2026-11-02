# Distributed Digit Relay — Technical Documentation

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Repository Layout](#2-repository-layout)
3. [System Architecture](#3-system-architecture)
4. [ROS Communication Layer](#4-ros-communication-layer)
5. [Canvas and Slot System](#5-canvas-and-slot-system)
6. [Class Hierarchy and Inheritance](#6-class-hierarchy-and-inheritance)
7. [Module Reference](#7-module-reference)
   - [digit_paths.py](#71-digit_pathspy)
   - [relay_node.py](#72-relay_nodepy)
   - [digit_drawing.py](#73-digit_drawingpy)
   - [member_01.py through member_09.py](#74-member_01py-through-member_09py)
   - [relay.launch.py](#75-relaylaunchpy)
   - [setup.py and package.xml](#76-setuppy-and-packagexml)
8. [Full Execution Flow](#8-full-execution-flow)
9. [Turn Protocol State Machine](#9-turn-protocol-state-machine)
10. [Drawing Engine Internals](#10-drawing-engine-internals)
11. [Member Identity and Glyph Table](#11-member-identity-and-glyph-table)
12. [Error Handling and Safety](#12-error-handling-and-safety)
13. [Build and Run](#13-build-and-run)
14. [Observability and Debugging](#14-observability-and-debugging)

---

## 1. Project Overview

**Distributed Digit Relay** is a ROS 2 Python package that coordinates nine independent
nodes—called *members*—to draw digits on a single shared turtlesim canvas in a strict,
turn-based sequence.

The core idea is a **relay baton**: a turn token passed between members with ROS service
calls. Only the member that holds the token draws; all others wait for their `take_turn`
service to be called. When a member finishes drawing, it calls the next member's service
to pass the token. This continues for two full cycles (18 turns total), after which the
system shuts down automatically.

**Key design goals:**
- Each member node is fully autonomous and owns its own ROS endpoints.
- No shared memory, no direct calls between members—only ROS service calls and topics.
- The drawing controller uses real-time pose feedback from turtlesim, not timed sleeps.
- All asynchronous operations (service calls, drawing) return `concurrent.futures.Future`
  objects and are chained with callbacks—the ROS event loop is never blocked.

**What the relay draws:**

| Cycle | Turn sequence | Each member draws |
|---|---|---|
| Cycle 1 | Turns 1–9 | Its own single-digit glyph (Member 01 → `1`, Member 02 → `2`, …, Member 09 → `9`) |
| Cycle 2 | Turns 10–18 | Its own two-digit glyph (Member 01 → `10`, Member 02 → `11`, …, Member 09 → `18`) |

Member 01 clears the turtlesim canvas at the start of cycle two (turn 10). Member 09
handles the final turn (turn 18) and triggers ROS shutdown.

---

## 2. Repository Layout

```text
distributed-digit-relay/
├── .gitignore
├── README.md                          ← Project summary and quick-start
├── doc/
│   └── documentation.md              ← This file
└── src/
    ├── digit_relay_interfaces/        ← Service definitions (ament_cmake + rosidl)
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   └── srv/
    │       └── TakeTurn.srv           ← Turn-token handoff request/response
    └── digit_relay/                   ← ROS 2 Python package root
        ├── package.xml                ← ROS package metadata and dependencies
        ├── setup.py                   ← Python package entry points and install rules
        ├── setup.cfg                  ← setuptools configuration
        ├── resource/
        │   └── digit_relay            ← ament resource marker (required by ROS 2)
        ├── launch/
        │   └── relay.launch.py        ← Starts turtlesim + all 9 members; shuts on completion
        └── digit_relay/               ← Python package (importable as `digit_relay`)
            ├── __init__.py
            ├── digit_paths.py         ← Canvas constants and slot allocation
            ├── relay_node.py          ← Abstract base class: turn services + entry point
            ├── digit_drawing.py       ← Pose-feedback drawing controller
            ├── member_01.py           ← Concrete member: draws 1 / 10
            ├── member_02.py           ← Concrete member: draws 2 / 11
            ├── member_03.py           ← Concrete member: draws 3 / 12
            ├── member_04.py           ← Concrete member: draws 4 / 13
            ├── member_05.py           ← Concrete member: draws 5 / 14
            ├── member_06.py           ← Concrete member: draws 6 / 15
            ├── member_07.py           ← Concrete member: draws 7 / 16
            ├── member_08.py           ← Concrete member: draws 8 / 17
            └── member_09.py           ← Concrete member: draws 9 / 18 (final)
```

**File responsibilities at a glance:**

| File | Responsibility |
|---|---|
| `digit_paths.py` | Canvas dimensions, 3×3 grid constants, `DigitSlot` type, and `slot_for_member()` allocator. Pure math, no ROS. |
| `relay_node.py` | Abstract base class `DigitRelayNode`: stores member identity and slot, owns the `take_turn` service server plus a client to every peer, runs the turn state machine (`_on_take_turn` service callback, `_after_turn`, `_handoff_turn`), implements `execute_turn()` Future chaining, and provides `run_member()` entry point. |
| `digit_drawing.py` | `DigitDrawingMember` subclass: stores pen color and normalized stroke paths; exposes `_draw_configured_glyph()`. Inner class `_DigitDrawingJob` runs the actual pose-feedback motion loop. |
| `member_NN.py` | Concrete node: creates its turtle endpoints (cmd_vel publisher, pose subscriber, pen/clear service clients), implements `prepare_turn()`/`draw_digit()`, and provides glyph stroke data. |
| `relay.launch.py` | ROS 2 launch description: starts `turtlesim_node` and all nine member executables simultaneously; registers a shutdown event on Member 09 exit. |
| `setup.py` | Registers nine console script entry points (one per member) and installs the launch file. |
| `digit_relay_interfaces/srv/TakeTurn.srv` | Service definition used to hand the turn token between members. |

---

## 3. System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    relay.launch.py                      │
│  ┌──────────┐  ┌────────┐  ┌────────┐     ┌────────┐   │
│  │turtlesim │  │member01│  │member02│ ... │member09│   │
│  │  _node   │  │  node  │  │  node  │     │  node  │   │
│  └──────────┘  └────────┘  └────────┘     └────────┘   │
└─────────────────────────────────────────────────────────┘
          │             │           │              │
          │   /digit_relay/member_NN/take_turn  (digit_relay_interfaces/TakeTurn)
          │      ←────────────── (previous member calls next member)
          │
          │   /digit_relay/status (String)
          │      ←────────────────────────────────
          │
          │   /turtle1/pose  (turtlesim/Pose)
          │      ──────────→ (all members subscribe)
          │
          │   /turtle1/cmd_vel  (geometry_msgs/Twist)
          │      ←──────────── (active member publishes)
          │
          │   /turtle1/set_pen  (turtlesim/SetPen service)
          │      ←──────────── (active member calls)
          │
          │   /clear  (std_srvs/Empty service)
          │      ←──────────── (Member 01 calls at turn 10)
```

All nine member nodes are **always running**. Each exposes its own
`/digit_relay/member_NN/take_turn` service. Only the node currently holding the turn
token executes its drawing; when it finishes it calls the next member's `take_turn`
service with the incremented turn number, and publishes a `WAITING_FOR_MEMBER_NN` status
while waiting for that handoff to be accepted.

---

## 4. ROS Communication Layer

### 4.1 Turn Service — `/digit_relay/member_NN/take_turn`

| Field | Value |
|---|---|
| Type | `digit_relay_interfaces/srv/TakeTurn` |
| Servers | One per member (`/digit_relay/member_01/take_turn` … `/digit_relay/member_09/take_turn`) |
| Clients | Every member keeps a client to every peer |

**Service definition (`TakeTurn.srv`):**
```
int32 current_number    # Which turn number is being handed over (1–18)
---
bool accepted           # True when the member started the turn
string message          # Acceptance/rejection explanation
```

The turn token travels as a service call instead of a broadcast topic: the member that
finishes a turn calls `take_turn` on the member that owns the next turn number. The
service callback validates the request, starts drawing asynchronously, and returns
`accepted=True` immediately so the executor stays free for pose feedback and timers.

Member 01 waits until every peer's `take_turn` service is discoverable (polls
`client.service_is_ready()` every 100 ms) before claiming turn 1 locally. This
guarantees all nodes are ready before the relay begins.

### 4.2 Status Topic — `/digit_relay/status`

| Field | Value |
|---|---|
| Type | `std_msgs/msg/String` |
| QoS Reliability | `RELIABLE` |
| QoS Depth | `1` |

**Message format:**
```
member=NN state=STATE current_number=N active_member=N
```

Valid states published by any member:

| State | Meaning |
|---|---|
| `WAITING_FOR_TURN` | Node has initialized and is waiting for its turn service to be called. |
| `WAITING_FOR_MEMBER_NN` | This member finished its turn and handed the token to member NN; it is idle. |
| `DRAWING` | This member received its token and is executing its turn. |
| `COMPLETE` | The relay has finished all 18 turns. |
| `ERROR` | A service call or drawing operation failed on this member's turn. |

### 4.3 Turtle Topics and Services

| ROS name | Type | Direction | Purpose |
|---|---|---|---|
| `/turtle1/pose` | `turtlesim/msg/Pose` | turtlesim → members | Position (`x`, `y`) and heading (`theta`) feedback used by the motion controller. |
| `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | active member → turtlesim | Velocity commands that move the turtle. Only the currently drawing member publishes. |
| `/turtle1/set_pen` | `turtlesim/srv/SetPen` | members → turtlesim | Set pen RGB color, width, and lift/lower state. Each member owns its own client. |
| `/clear` | `std_srvs/srv/Empty` | Member 01 → turtlesim | Clear the canvas. Called once by Member 01 at the start of cycle two. |

### 4.4 Service Discovery — No Fixed Sleeps

Members do not assume a service is available immediately on startup. The helper
`_call_service_when_ready(client, request, service_name)` checks
`client.service_is_ready()` and, if not ready, creates a 100 ms `STEADY_TIME` timer
that polls until the service appears. This avoids any `time.sleep()` that would block
the ROS event loop.

---

## 5. Canvas and Slot System

### Canvas dimensions (`digit_paths.py`)

```
CANVAS_WIDTH  = 11.0  (turtlesim x range: 0.0 to ~11.0)
CANVAS_HEIGHT = 11.0  (turtlesim y range: 0.0 to ~11.0)
CANVAS_MARGIN = 0.8   (border inset on all sides)
SLOT_GAP_X    = 0.35  (horizontal gap between columns)
SLOT_GAP_Y    = 0.35  (vertical gap between rows)
```

### 3×3 slot grid

The canvas is divided into a fixed 3×3 grid. Each cell is assigned to exactly one
member based on its ID:

```
┌──────────────────────────────────────┐
│  Slot(1) M01  │ Slot(2) M02  │ Slot(3) M03  │  Row 0 (top)
├──────────────────────────────────────┤
│  Slot(4) M04  │ Slot(5) M05  │ Slot(6) M06  │  Row 1 (middle)
├──────────────────────────────────────┤
│  Slot(7) M07  │ Slot(8) M08  │ Slot(9) M09  │  Row 2 (bottom)
└──────────────────────────────────────┘
```

Turtlesim's Y axis increases upward, so row 0 maps to the highest Y values.

### `DigitSlot` NamedTuple

```python
class DigitSlot(NamedTuple):
    x: float      # center X of the slot
    y: float      # center Y of the slot
    width: float  # usable width of the slot
    height: float # usable height of the slot
```

### Slot allocation formula

```python
def slot_for_member(member_id: int, total_members: int) -> DigitSlot:
    row, column = divmod(member_id - 1, SLOT_COLUMNS)   # 0-based row and column
    x = CANVAS_MARGIN + SLOT_WIDTH / 2 + column * (SLOT_WIDTH + SLOT_GAP_X)
    y = CANVAS_HEIGHT - CANVAS_MARGIN - SLOT_HEIGHT / 2 - row * (SLOT_HEIGHT + SLOT_GAP_Y)
    return DigitSlot(x=x, y=y, width=SLOT_WIDTH, height=SLOT_HEIGHT)
```

### Glyph-to-slot mapping

Normalized stroke coordinates (0.0–1.0) are scaled into each slot with a safe inner
margin:

```python
glyph_width  = min(1.25, slot.width  - 0.3)
glyph_height = min(1.9,  slot.height - 0.3)
left_x   = slot.x - glyph_width / 2.0
bottom_y = slot.y - glyph_height / 2.0

mapped_x = left_x   + x * glyph_width
mapped_y = bottom_y + y * glyph_height
```

This guarantees glyphs stay inside their slot and never overlap neighboring cells.

---

## 6. Class Hierarchy and Inheritance

```
rclpy.node.Node  (ABC)
    │
    └── DigitRelayNode            ← relay_node.py
            │  - member_id, total_members, slot, _busy
            │  - execute_turn()        (concrete)
            │  - prepare_turn()        (abstract)
            │  - set_pen()             (abstract)
            │  - draw_digit()          (abstract)
            │
            └── DigitDrawingMember    ← digit_drawing.py
                    │  - pen_color, glyph, valid_turn_numbers
                    │  - _normalized_strokes, pose, _active_job
                    │  - _on_pose()
                    │  - _stop_turtle()
                    │  - _draw_configured_glyph()
                    │
                    ├── Member01       ← member_01.py
                    ├── Member02       ← member_02.py
                    ├── Member03       ← member_03.py
                    ├── Member04       ← member_04.py
                    ├── Member05       ← member_05.py
                    ├── Member06       ← member_06.py
                    ├── Member07       ← member_07.py
                    ├── Member08       ← member_08.py
                    └── Member09       ← member_09.py
```

Each concrete `MemberNN` class:
- Calls the drawing base constructor with its unique `member_id`, pen color, glyph
  label, valid turns, and initial stroke paths.
- Creates its turtle endpoints: pen/clear service clients, `cmd_vel` publisher, and
  `pose` subscriber.
- Implements `prepare_turn()`, `set_pen()`, and `draw_digit()`.
- Inherits turn coordination (`take_turn` service, handoff, status publishing) from
  `DigitRelayNode`.

---

## 7. Module Reference

### 7.1 `digit_paths.py`

**Purpose:** Shared canvas geometry. No ROS imports.

#### Constants

| Name | Value | Description |
|---|---|---|
| `CANVAS_WIDTH` | `11.0` | Turtlesim X range |
| `CANVAS_HEIGHT` | `11.0` | Turtlesim Y range |
| `CANVAS_MARGIN` | `0.8` | Border inset |
| `SLOT_COLUMNS` | `3` | Grid columns |
| `SLOT_ROWS` | `3` | Grid rows |
| `SLOT_GAP_X` | `0.35` | Horizontal gap between slots |
| `SLOT_GAP_Y` | `0.35` | Vertical gap between slots |
| `MAX_MEMBERS` | `9` | Maximum supported members |
| `SLOT_WIDTH` | computed | Width of one slot cell |
| `SLOT_HEIGHT` | computed | Height of one slot cell |

#### `DigitSlot`

```python
class DigitSlot(NamedTuple):
    x: float      # Center X in canvas coordinates
    y: float      # Center Y in canvas coordinates
    width: float  # Usable slot width
    height: float # Usable slot height
```

#### `slot_for_member(member_id, total_members) -> DigitSlot`

Returns the pre-computed slot for the given 1-based member ID. Validates that
`1 ≤ member_id ≤ total_members ≤ 9`. Raises `ValueError` otherwise.

---

### 7.2 `relay_node.py`

**Purpose:** Abstract base class that owns turn coordination (service server, peer
clients, status topic) and defines the member drawing contract.

#### `class DigitRelayNode(Node, ABC)`

Inherits from `rclpy.node.Node` and `ABC`. Names the node
`digit_relay_member_NN` based on `member_id`.

**Constructor** `__init__(*, member_id, total_members, valid_turn_numbers)`

| Sets | Description |
|---|---|
| `self.member_id` | 1-based integer ID |
| `self.total_members` | Total group size (9) |
| `self.valid_turn_numbers` | Turn numbers this member may draw |
| `self.slot` | Pre-computed `DigitSlot` from `slot_for_member()` |
| `self._busy` | Boolean guard to prevent duplicate turn execution |
| `self._turn_service` | `take_turn` service server for this member |
| `self._turn_clients` | Dict of `take_turn` clients, one per peer |
| `self._status_pub` | Publisher for `/digit_relay/status` |

**Abstract methods** — must be implemented by every concrete member:

| Method | Signature | Contract |
|---|---|---|
| `prepare_turn` | `(number: int) -> Future` | Run service calls (pen lift, canvas clear) before drawing begins. Must return a Future that resolves when preparation is complete. |
| `set_pen` | `(*, off: bool) -> Future` | Call `/turtle1/set_pen` using this member's own service client. Must return a Future. |
| `draw_digit` | `(number: int, slot: DigitSlot) -> Future` | Execute the glyph drawing. The Future must resolve only after the turtle has stopped and the glyph is complete. |

**Concrete methods:**

| Method | Behavior |
|---|---|
| `start_relay()` | Member 01 only: polls until every peer's `take_turn` service is ready, then claims turn 1 locally. |
| `_on_take_turn(request, response)` | Service callback: validates the token, starts `execute_turn()`, and replies with `accepted` without blocking the executor. |
| `_after_turn(future, number)` | On success computes the next turn/member and calls `_handoff_turn()`; on failure publishes `ERROR` and intentionally does not pass the token. On turn 18 publishes `COMPLETE` and shuts ROS down. |
| `_handoff_turn(number, member)` | Calls the next member's `take_turn` service and logs acceptance or rejection. |
| `execute_turn(number)` | Chains `prepare_turn()` and `draw_digit()` using Future callbacks. Returns a single outer Future that resolves when both steps complete. If either step raises, the outer Future is resolved with that exception. |
| `_fail_turn(reason)` | Clears `_busy`, logs an error. The turn token is intentionally not passed on failure. |
| `_publish_status(...)` | Formats and publishes to `/digit_relay/status` and logs the same string. |
| `_call_service_when_ready(client, request, name)` | Calls a service immediately if ready, otherwise polls with a 100 ms `STEADY_TIME` timer. Returns a Future. |

#### `run_member(member_type)`

Module-level function. Calls `rclpy.init()`, constructs a node of the given type,
calls `rclpy.spin()` (blocking), then destroys the node and shuts ROS down after
spin exits. Used as the `main()` body in every member file.

---

### 7.3 `digit_drawing.py`

**Purpose:** Adds glyph configuration and the pose-feedback drawing engine to the
abstract base. Members that inherit from this class get the full drawing controller
for free.

#### `class DigitDrawingMember(DigitRelayNode)`

**Constructor** `__init__(*, member_id, total_members, pen_color, glyph, valid_turn_numbers, normalized_strokes)`

| Parameter | Type | Description |
|---|---|---|
| `pen_color` | `tuple[int,int,int]` | RGB pen color, each channel 0–255 |
| `glyph` | `str` | Human-readable glyph label for logging |
| `valid_turn_numbers` | `tuple[int,...]` | Turns this member is allowed to draw on |
| `normalized_strokes` | `Sequence[Stroke]` | Initial stroke paths normalized to `[0,1]²` |

Additional state:
- `self.pose` — latest `turtlesim/Pose` received; `None` until the first pose arrives.
- `self._pose_seen` — logs once when the first pose is received.
- `self._active_job` — the current `_DigitDrawingJob` instance, or `None`.

**Methods:**

| Method | Description |
|---|---|
| `_on_pose(pose)` | Stores the latest pose; logs when first received. Called by each member's `/turtle1/pose` subscription. |
| `_stop_turtle()` | Publishes a zero `Twist` to `/turtle1/cmd_vel`, stopping the turtle immediately. |
| `_draw_configured_glyph(number, slot)` | Validates that `number` is in `valid_turn_numbers`. Maps normalized strokes into the slot, creates a `_DigitDrawingJob`, and returns its completion `Future`. |

**Module-level helpers:**

| Helper | Description |
|---|---|
| `_normalize_angle(angle)` | Wraps any angle to `[−π, π]` using `atan2(sin, cos)`. |
| `_map_glyph_to_slot(strokes, slot)` | Scales all normalized `(x,y)` points into the safe drawing area inside the given slot. Returns a new list of world-coordinate strokes. |

#### `class _DigitDrawingJob`

Internal class, created once per `_draw_configured_glyph()` call. Controls the turtle
through all strokes via a 30 Hz timer driven by `STEADY_TIME` (wall clock, not sim
time).

**State machine within `_tick()`:**

```
  [move_to_stroke_start]
         │
         │ turtle reached stroke[i][0]
         ▼
  [waiting_for_pen]  ←── set_pen(off=False)
         │
         │ pen lowered
         ▼
  [drawing_stroke]
         │
         │ all points in stroke[i] drawn
         ▼
  [waiting_for_pen]  ←── set_pen(off=True)
         │
         │ pen lifted
         ▼
  [next_stroke]
         │
         ├── more strokes? → back to [move_to_stroke_start] (stroke_index++)
         │
         └── no more strokes? → _finish()
```

**`_drive_to(target, pose) -> bool`**

Computes heading error and distance to `target`, publishes a `Twist` command, returns
`True` when the turtle is within 0.035 units of the target:

```
heading_error = normalize_angle(atan2(dy, dx) - pose.theta)

if |heading_error| > 0.12:
    # Pure rotation mode
    angular.z = clamp(3.0 * heading_error, -1.5, 1.5)
else:
    # Combined forward + angular correction
    linear.x  = min(1.2, 1.5 * distance)
    angular.z = clamp(2.5 * heading_error, -1.0, 1.0)
```

---

### 7.4 `member_01.py` through `member_09.py`

All nine member files follow the same pattern. The differences are the `MEMBER_ID`,
pen color, glyph stroke paths, and—for Member 01—the canvas-clear call in
`prepare_turn()`.

#### Common pattern for every member

**Module constants:**
```python
MEMBER_ID     = N               # 1-based
TOTAL_MEMBERS = 9
PEN_COLOR     = (R, G, B)
FIRST_TURN    = MEMBER_ID       # Turn in cycle 1
SECOND_TURN   = TOTAL_MEMBERS + MEMBER_ID  # Turn in cycle 2
VALID_TURNS   = (FIRST_TURN, SECOND_TURN)
GLYPH_X_STROKES  = (...)       # Cycle-1 glyph, normalized
GLYPH_XY_STROKES = (...)       # Cycle-2 glyph, normalized
```

**`__init__()` — creates this member's turtle endpoints:**
```
_pen_client   ← create_client(SetPen, '/turtle1/set_pen')
_cmd_pub      ← create_publisher(Twist, '/turtle1/cmd_vel', 10)
_pose_sub     ← create_subscription(Pose, '/turtle1/pose', _on_pose, 10)
(Member 01 also: _clear_client ← create_client(Empty, '/clear'), start_relay())
```

Turn endpoints (`_turn_service`, `_turn_clients`, `_status_pub`) are created by the
`DigitRelayNode` base constructor.

**`_on_take_turn(request, response)` — turn token service callback:**

```
Receive TakeTurn.Request(current_number)
│
├── _busy?
│       → accepted=False ("member is busy")
│
├── current_number not in VALID_TURNS?
│       → accepted=False (turn belongs to another member)
│
├── current_number ≤ _last_completed_number?
│       → accepted=False (already processed)
│
└── set _busy = True
    publish DRAWING status
    call execute_turn(current_number)
        .add_done_callback(_after_turn)
    return accepted=True   ← replied immediately, drawing runs in background
```

**`_after_turn(future, current_number)` — turn completion handler:**

```
Receive Future result
│
├── Exception? → publish ERROR, call _fail_turn(), return
│                (token is NOT handed off — relay stalls by design)
│
├── current_number == 2 * total_members (final turn)?
│       → publish COMPLETE, call rclpy.shutdown()
│
└── compute next token:
        next_number  = current_number + 1
        next_member  = (next_number - 1) % total_members + 1
    publish WAITING_FOR_MEMBER_NN status
    _handoff_turn(next_number, next_member)
        → call /digit_relay/member_NN/take_turn(next_number)
        → log accepted / rejected
```

**`prepare_turn(number)` → Future:**

- For Members 02–09: lifts the pen (`set_pen(off=True)`).
- For Member 01 on turn `TOTAL_MEMBERS + 1` (turn 10): calls `/clear` first, waits
  for it to complete, then lifts the pen. On turn 1, lifts the pen only.

**`draw_digit(number, slot)` → Future:**

- Selects the correct stroke path based on whether `number` is the first or second turn.
- Calls `_draw_configured_glyph(number, slot)` with the updated strokes.

**`set_pen(*, off)` → Future:**

- Builds a `SetPen.Request` with the member's own `PEN_COLOR` and width `2`.
- Delegates to `_call_service_when_ready()`.

**`_call_service_when_ready(client, request, service_name)` → Future:**

- If `client.service_is_ready()`: calls `call_async(request)` immediately.
- Otherwise: starts a 100 ms `STEADY_TIME` timer that polls until the service is
  ready, then calls it. The returned Future resolves with the service response.

**`_publish_status(state, current_number, active_member)`** — formats and publishes
to `/digit_relay/status` and logs the same string.

#### Member 01 — special role: relay initiator and cycle-two clearer

Member 01 is responsible for:
1. Waiting until all 8 peer `take_turn` services are discoverable
   (polls `client.service_is_ready()` every 100 ms via `start_relay()`).
2. Claiming turn 1 locally once every peer is ready — the relay begins.
3. Calling `/clear` at the start of cycle two (turn 10) before lifting its pen.

No other member performs these actions.

#### Member 09 — special role: relay terminator

Member 09 processes the final turn (turn 18). After its drawing Future completes,
`_after_turn()` detects `current_number == 2 * total_members`, publishes `COMPLETE`,
and calls `rclpy.shutdown()`. The launch file monitors Member 09's process exit and
shuts down turtlesim.

---

### 7.5 `relay.launch.py`

```python
def generate_launch_description():
    turtlesim = Node(package='turtlesim', executable='turtlesim_node', ...)
    member_01 = Node(package='digit_relay', executable='digit_relay_member_01', ...)
    member_02 = ...
    ...
    member_09 = ...
    shutdown_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=member_09,
            on_exit=[Shutdown(reason='Distributed digit relay completed.')]
        )
    )
    return LaunchDescription([turtlesim, member_01, ..., member_09, shutdown_when_complete])
```

All ten processes start simultaneously. The shutdown event is triggered when
`digit_relay_member_09` exits (which only happens after it calls `rclpy.shutdown()`).
At that point, ROS terminates turtlesim and all remaining member processes.

---

### 7.6 `setup.py` and `package.xml`

`setup.py` registers nine console script entry points:

```python
'console_scripts': [
    'digit_relay_member_01 = digit_relay.member_01:main',
    'digit_relay_member_02 = digit_relay.member_02:main',
    ...
    'digit_relay_member_09 = digit_relay.member_09:main',
]
```

`package.xml` declares runtime dependencies:
- `digit_relay_interfaces` — `TakeTurn` service definition for token handoffs
- `rclpy` — ROS 2 Python client library
- `std_msgs`, `std_srvs` — status message type, `/clear` service
- `geometry_msgs` — `Twist` for velocity commands
- `turtlesim` — `Pose`, `SetPen`

---

## 8. Full Execution Flow

```
relay.launch.py
  │
  ├── Starts: turtlesim_node
  ├── Starts: digit_relay_member_01  (calls member_01.main())
  ├── Starts: digit_relay_member_02  (calls member_02.main())
  │   ...
  └── Starts: digit_relay_member_09  (calls member_09.main())

All members: run_member(MemberNN)
  → rclpy.init()
  → MemberNN.__init__()
      → Create take_turn service + peer clients, turtle endpoints
      → Publish WAITING_FOR_TURN
  → rclpy.spin()                  ← event loop starts

Member 01 only: start_relay() [timer, 100ms poll]
  → Wait until all 8 peer take_turn services are ready
  → Claim turn 1 locally         ← relay begins

─────────────────── CYCLE 1 (Turns 1–9) ───────────────────

Turn 1 → Member 01:
  _on_take_turn(1)                     ← called locally by start_relay()
    set _busy = True, publish DRAWING
    execute_turn(1)
      prepare_turn(1) → set_pen(off=True)          ← lifts pen
      [pen ready] → draw_digit(1, slot_01)
        _draw_configured_glyph()
          create _DigitDrawingJob
            _tick() at 30Hz:
              move to stroke start (pen up)
              set_pen(off=False)                    ← lowers pen
              drive through stroke points
              set_pen(off=True)                     ← lifts pen
              ... repeat for each stroke
            _finish() → Future resolves
  _after_turn() → call member_02 take_turn(2)

Turn 2 → Member 02:
  (same pattern as Turn 1, Member 02 draws glyph '2')
  _after_turn() → call member_03 take_turn(3)

...

Turn 9 → Member 09:
  (draws glyph '9')
  _after_turn() → call member_01 take_turn(10)

─────────────────── CYCLE 2 (Turns 10–18) ──────────────────

Turn 10 → Member 01:
  _on_take_turn(10)                   ← called by Member 09's handoff
    execute_turn(10)
      prepare_turn(10):
        call /clear → canvas wiped
        [clear done] → set_pen(off=True)
      draw_digit(10, slot_01)
        glyph = '10', strokes = GLYPH_10_STROKES
        _draw_configured_glyph() → draws "10" in slot 1
  _after_turn() → call member_02 take_turn(11)

Turn 11 → Member 02: draws '11'
Turn 12 → Member 03: draws '12'
...
Turn 17 → Member 08: draws '17'

Turn 18 → Member 09:
  _on_take_turn(18)                   ← called by Member 08's handoff
    execute_turn(18) → draws '18'
  _after_turn():
    18 == 2 * 9 → final turn
    publish COMPLETE
    rclpy.shutdown()

Member 09 process exits
  → launch RegisterEventHandler fires
  → Shutdown(reason='Distributed digit relay completed.')
  → turtlesim and all other member processes terminate
```

---

## 9. Turn Protocol State Machine

Every member node runs this implicit state machine:

```
                         ┌──────────────────────────────┐
           Startup       │                              │
         ─────────────── ▼                              │
        │           [WAITING_FOR_TURN]                  │
        │                │                              │
        │      take_turn service called                 │
        │                ▼                              │
        │        _busy or invalid turn?                 │
        │           /         \                         │
        │         Yes           No                      │
        │          │            │                       │
        │     (reject call) [DRAWING]                   │
        │                           │                   │
        │                      success?                │
        │                    /        \                 │
        │                  No          Yes              │
        │                   │            │              │
        │               [ERROR]    final turn?          │
        │                          /       \            │
        │                        Yes        No          │
        │                         │          │          │
        │                  [COMPLETE]   call next       │
        │                  shutdown   member's          │
        │                           take_turn ──────────┘
        └──────────────────────────────┘
```

---

## 10. Drawing Engine Internals

### Timer and clock

`_DigitDrawingJob` creates a 30 Hz ROS timer using `STEADY_TIME` (wall clock). This
is essential: if the node is launched with `use_sim_time=True` but turtlesim is not
publishing `/clock`, a `ROS_TIME` or `SIM_TIME` timer would stall indefinitely.

### Waiting for prerequisites

On each `_tick()`:
1. If `self.completion.done()` or `state == 'waiting_for_pen'` → do nothing.
2. If `pose is None` → log once, return. (Turtlesim not yet publishing `/pose`.)
3. If `_cmd_pub.get_subscription_count() == 0` → log once, return. (Turtlesim not
   yet subscribing to `/cmd_vel`.)

Only when both pose and subscriber are present does the motion logic advance.

### Motion control (`_drive_to`)

The controller uses a simple proportional approach:

```
dx = target.x - pose.x
dy = target.y - pose.y
distance = hypot(dx, dy)

if distance < 0.035:           # arrived — stop and return True
    stop_turtle()
    return True

desired_heading = atan2(dy, dx)
heading_error = normalize_angle(desired_heading - pose.theta)

if |heading_error| > 0.12:    # large angle error — rotate in place
    angular.z = clamp(3.0 * heading_error, -1.5, 1.5)
    linear.x = 0
else:                          # small angle error — move forward
    linear.x  = min(1.2, 1.5 * distance)
    angular.z = clamp(2.5 * heading_error, -1.0, 1.0)

publish Twist
return False
```

### Stroke sequence

A glyph is a list of strokes. Each stroke is a list of waypoints:
```
stroke 0: [p0, p1, p2, ...]
stroke 1: [p0, p1, ...]
...
```

Between strokes the pen is lifted. Within a stroke the pen is lowered. The job
always starts with the pen lifted (done by `prepare_turn`).

```
For each stroke[i]:
    1. Pen is UP (from prepare_turn or previous stroke end)
    2. Drive to stroke[i][0]                  (pen up move)
    3. set_pen(off=False)                     (lower pen)
    4. Drive to stroke[i][1], [2], ..., [n]   (draw)
    5. set_pen(off=True)                      (lift pen)

After all strokes: _finish() → complete Future
```

---

## 11. Member Identity and Glyph Table

| Member | ID | Pen Color (RGB) | Color Name | Cycle-1 Glyph | Cycle-2 Glyph | Slot (row,col) | Turns |
|---|---|---|---|---|---|---|---|
| Member 01 | 1 | (255, 60, 60) | Red | `1` | `10` | (1,1) | 1, 10 |
| Member 02 | 2 | (255, 105, 180) | Hot Pink | `2` | `11` | (1,2) | 2, 11 |
| Member 03 | 3 | (0, 255, 0) | Green | `3` | `12` | (1,3) | 3, 12 |
| Member 04 | 4 | (255, 200, 40) | Yellow | `4` | `13` | (2,1) | 4, 13 |
| Member 05 | 5 | (180, 90, 255) | Purple | `5` | `14` | (2,2) | 5, 14 |
| Member 06 | 6 | (255, 140, 0) | Orange | `6` | `15` | (2,3) | 6, 15 |
| Member 07 | 7 | (0, 255, 255) | Cyan | `7` | `16` | (3,1) | 7, 16 |
| Member 08 | 8 | (128, 0, 0) | Dark Red | `8` | `17` | (3,2) | 8, 17 |
| Member 09 | 9 | (255, 255, 255) | White | `9` | `18` | (3,3) | 9, 18 |

**Turn number formula:**
```
FIRST_TURN  = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID    (= 9 + MEMBER_ID)
```

**Next-member formula (used in `_after_turn`):**
```python
next_number = current_number + 1
next_member = (next_number - 1) % total_members + 1
```
This wraps correctly: after turn 9 (Member 09), next is turn 10 assigned to Member 01.

---

## 12. Error Handling and Safety

| Situation | Behavior |
|---|---|
| Service call fails | Future resolves with exception. `_after_turn()` catches it, publishes `ERROR`, calls `_fail_turn()` which clears `_busy`. The turn token is **not passed**; the relay stalls. |
| Drawing motion fails | Same as above—`_DigitDrawingJob._fail()` rejects the Future. |
| `take_turn` called with a turn this member does not own | Service replies `accepted=False` with an explanation; the caller logs an error. |
| Duplicate turn token (same number received twice) | `current_number ≤ _last_completed_number` guard rejects the call with `accepted=False`. |
| `_busy` is `True` when `take_turn` arrives | Service replies `accepted=False`. Prevents concurrent drawing if a token is somehow re-received. |
| Pose not yet available when drawing starts | `_tick()` logs a single warning and waits. Drawing starts as soon as pose arrives. |
| `/turtle1/cmd_vel` has no subscriber (turtlesim not ready) | `_tick()` logs a single warning and waits. |
| `KeyboardInterrupt` during spin | `run_member()` catches it in the `finally` block, destroys the node cleanly, and calls `rclpy.shutdown()` if still active. |

---

## 13. Build and Run

### Build

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws

# Remove stale build artifacts (important after adding/renaming members)
rm -rf build/digit_relay build/digit_relay_interfaces install/digit_relay install/digit_relay_interfaces

colcon build --packages-select digit_relay_interfaces digit_relay
source install/setup.bash
```

### Run

```bash
ros2 launch digit_relay relay.launch.py
```

The relay starts all nodes, runs two full cycles (18 turns), and exits automatically.

### Verify endpoints before launch (optional)

After building:
```bash
ros2 run digit_relay digit_relay_member_01 &
ros2 node list
ros2 topic list
kill %1
```

---

## 14. Observability and Debugging

While the launch is running, open a second terminal with the workspace sourced:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ros2_ws
source install/setup.bash
```

### Watch the turn handoffs

```bash
ros2 service list | grep take_turn
# Shows /digit_relay/member_01/take_turn through .../member_09/take_turn

ros2 interface show digit_relay_interfaces/srv/TakeTurn
# Shows the request (current_number) and response (accepted, message)
```

### Watch all member statuses

```bash
ros2 topic echo /digit_relay/status
# Shows: member=NN state=STATE current_number=N active_member=N
```

### Check all running nodes

```bash
ros2 node list
# Should show: /digit_relay_member_01 through /digit_relay_member_09 + /turtlesim
```

### Inspect service details

```bash
ros2 service type /digit_relay/member_01/take_turn
# digit_relay_interfaces/srv/TakeTurn

ros2 service list
# Should show /digit_relay/member_NN/take_turn, /turtle1/set_pen, /clear, etc.
```

### Check single member logs

Each member node prints its status to the console (`output='screen'` in the launch
file). You can filter for a specific member:

```bash
# (In the launch terminal output, each line is prefixed with [member_NN])
```

### Common issues

| Symptom | Likely cause |
|---|---|
| Relay never starts (hangs at "Waiting until all 9...") | Not all 9 members are running or their `take_turn` services are not discoverable. Check `ros2 node list` and `ros2 service list | grep take_turn`. |
| Turtle moves erratically | Multiple members publishing to `/cmd_vel` simultaneously (shouldn't happen in correct implementation). |
| Turn stalls after a certain number | A member's `prepare_turn()` or `draw_digit()` Future never resolved (service unavailable or exception caught and swallowed). Check console for `ERROR` state. |
| Canvas not cleared between cycles | Member 01's `/clear` client could not reach the service. Check turtlesim is running. |
