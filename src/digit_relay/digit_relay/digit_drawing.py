"""Reusable pose-feedback drawing controller for member-specific glyphs."""

from concurrent.futures import Future
import math
from typing import Sequence

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from turtlesim.msg import Pose

from .digit_paths import DigitSlot
from .relay_node import DigitRelayNode

Stroke = Sequence[tuple[float, float]]


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _map_glyph_to_slot(
    strokes: Sequence[Stroke], slot: DigitSlot
) -> list[list[tuple[float, float]]]:
    """Map one normalized glyph into a centered, safely sized slot area."""
    glyph_width = min(1.25, slot.width - 0.3)
    glyph_height = min(1.9, slot.height - 0.3)
    left_x = slot.x - glyph_width / 2.0
    bottom_y = slot.y - glyph_height / 2.0
    return [
        [
            (left_x + x * glyph_width, bottom_y + y * glyph_height)
            for x, y in stroke
        ]
        for stroke in strokes
    ]


class DigitDrawingMember(DigitRelayNode):
    """Base for members that draw one fixed glyph on their configured turns."""

    def __init__(
        self,
        *,
        member_id: int,
        total_members: int,
        pen_color: tuple[int, int, int],
        glyph: str,
        valid_turn_numbers: tuple[int, ...],
        normalized_strokes: Sequence[Stroke],
    ) -> None:
        super().__init__(
            member_id=member_id,
            total_members=total_members,
        )
        self.pen_color = pen_color
        self.glyph = glyph
        self.valid_turn_numbers = valid_turn_numbers
        self._normalized_strokes = normalized_strokes
        self.pose: Pose | None = None
        self._pose_seen = False
        self._active_job: _DigitDrawingJob | None = None

    def _on_pose(self, pose: Pose) -> None:
        self.pose = pose
        if not self._pose_seen:
            self._pose_seen = True
            self.get_logger().info(
                f'Received /turtle1/pose at ({pose.x:.2f}, {pose.y:.2f}); '
                'motion feedback is ready.'
            )

    def _stop_turtle(self) -> None:
        self._cmd_pub.publish(Twist())

    def _draw_configured_glyph(self, number: int, slot: DigitSlot) -> Future:
        """Start the shared pose-feedback controller for this member's glyph."""
        completion: Future = Future()
        if number not in self.valid_turn_numbers:
            completion.set_exception(
                ValueError(
                    f'Member {self.member_id:02d} draws glyph {self.glyph} only '
                    f'on turns {self.valid_turn_numbers}; received {number}.'
                )
            )
            return completion

        strokes = _map_glyph_to_slot(self._normalized_strokes, slot)
        self.get_logger().info(
            f'Turn {number}: drawing glyph {self.glyph} in slot '
            f'centered at ({slot.x:.2f}, {slot.y:.2f}).'
        )
        self._active_job = _DigitDrawingJob(self, strokes, completion)
        return completion


class _DigitDrawingJob:
    """Follow pose feedback through pen-up and pen-down stroke paths."""

    def __init__(
        self,
        node: DigitDrawingMember,
        strokes: Sequence[Sequence[tuple[float, float]]],
        completion: Future,
    ) -> None:
        self.node = node
        self.strokes = strokes
        self.completion = completion
        self.stroke_index = 0
        self.point_index = 0
        self._logged_first_command = False
        self._logged_missing_pose = False
        self._logged_missing_subscriber = False
        # DigitRelayNode already lifted the pen before calling draw_digit().
        self.state = 'move_to_stroke_start'
        # Motion control must advance on wall time. A node launched with
        # use_sim_time=True would otherwise wait forever if turtlesim does not
        # publish /clock.
        self.timer = node.create_timer(
            1.0 / 30.0,
            self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _set_pen(self, *, off: bool, next_state: str) -> None:
        self.state = 'waiting_for_pen'
        future = self.node.set_pen(off=off)
        future.add_done_callback(lambda completed: self._pen_done(completed, next_state))

    def _pen_done(self, future, next_state: str) -> None:
        try:
            future.result()
        except Exception as exc:
            self._fail(RuntimeError(f'Could not change pen state: {exc}'))
            return
        if not self.completion.done():
            self.state = next_state
            if next_state == 'move_to_stroke_start':
                self.point_index = 0
            elif next_state == 'drawing_stroke':
                self.point_index = 1

    def _tick(self) -> None:
        if self.completion.done() or self.state == 'waiting_for_pen':
            return
        pose = self.node.pose
        if pose is None:
            if not self._logged_missing_pose:
                self._logged_missing_pose = True
                self.node.get_logger().warning(
                    'Waiting for /turtle1/pose; check that turtlesim_node is running '
                    'and both nodes use the same ROS_DOMAIN_ID.'
                )
            return

        if self.node._cmd_pub.get_subscription_count() == 0:
            if not self._logged_missing_subscriber:
                self._logged_missing_subscriber = True
                self.node.get_logger().warning(
                    'Pose is available, but /turtle1/cmd_vel has no subscriber; '
                    'waiting for turtlesim_node.'
                )
            return

        points = self.strokes[self.stroke_index]
        if self.state == 'move_to_stroke_start':
            if self._drive_to(points[0], pose):
                self._set_pen(off=False, next_state='drawing_stroke')
            return

        if self.state == 'drawing_stroke':
            if self._drive_to(points[self.point_index], pose):
                self.point_index += 1
                if self.point_index >= len(points):
                    self._set_pen(off=True, next_state='next_stroke')
            return

        if self.state == 'next_stroke':
            self.stroke_index += 1
            if self.stroke_index >= len(self.strokes):
                self._finish()
            else:
                # The previous stroke's pen-off request completed before here.
                self.point_index = 0
                self.state = 'move_to_stroke_start'

    def _drive_to(self, target: tuple[float, float], pose: Pose) -> bool:
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        distance = math.hypot(dx, dy)

        if distance < 0.035:
            self.node._stop_turtle()
            return True

        if not self._logged_first_command:
            self._logged_first_command = True
            self.node.get_logger().info(
                f'Motion control active: moving from ({pose.x:.2f}, {pose.y:.2f}) '
                f'toward ({target[0]:.2f}, {target[1]:.2f}).'
            )

        desired_heading = math.atan2(dy, dx)
        heading_error = _normalize_angle(desired_heading - pose.theta)
        command = Twist()

        if abs(heading_error) > 0.12:
            command.angular.z = max(-1.5, min(1.5, 3.0 * heading_error))
        else:
            command.linear.x = min(1.2, 1.5 * distance)
            command.angular.z = max(-1.0, min(1.0, 2.5 * heading_error))

        self.node._cmd_pub.publish(command)
        return False

    def _finish(self) -> None:
        self.node._stop_turtle()
        self.timer.cancel()
        if not self.completion.done():
            self.completion.set_result(None)

    def _fail(self, error: Exception) -> None:
        self.node._stop_turtle()
        self.timer.cancel()
        if not self.completion.done():
            self.completion.set_exception(error)
