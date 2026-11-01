"""Member 04: draw glyph 4 on turn 4 and glyph 13 on turn 13."""

from concurrent.futures import Future

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray, String
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

SERVICE_DISCOVERY_POLL_PERIOD = 0.1
TURN_TOPIC = '/digit_relay/turn'
STATUS_TOPIC = '/digit_relay/status'
TURN_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

MEMBER_ID = 4
TOTAL_MEMBERS = 9
PEN_COLOR = (255, 200, 40)
FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
VALID_TURNS = (FIRST_TURN, SECOND_TURN)

GLYPH_4_STROKES = (
    ((0.0, 1.0), (0.0, 0.35), (1.0, 0.35)),
    ((0.7, 1.0), (0.7, 0.0)),
)
GLYPH_13_STROKES = (
    ((0.1, 0.0), (0.1, 1.0)),
    ((0.45, 1.0), (1.0, 1.0), (1.0, 0.0), (0.45, 0.0)),
    ((0.65, 0.5), (1.0, 0.5)),
)


class Member04(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='4',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_4_STROKES,
        )
        self._pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self._on_pose, 10
        )
        self._turn_pub = self.create_publisher(Int32MultiArray, TURN_TOPIC, TURN_QOS)
        self._turn_sub = self.create_subscription(
            Int32MultiArray, TURN_TOPIC, self._on_turn, TURN_QOS
        )
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, STATUS_QOS)
        self._last_completed_number = 0
        self._publish_status('WAITING_FOR_TURN')

    def _on_turn(self, message: Int32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().error('Ignoring malformed turn token; expected [number, member].')
            return
        current_number, active_member = message.data[:2]
        if active_member != self.member_id:
            self._publish_status(
                f'WAITING_FOR_MEMBER_{active_member:02d}', current_number, active_member
            )
            return
        if self._busy or current_number not in VALID_TURNS:
            return
        if current_number <= self._last_completed_number:
            return

        self._busy = True
        self._publish_status('DRAWING', current_number, active_member)
        self.get_logger().info(f'My turn: drawing {current_number} in slot {self.member_id}.')
        self.execute_turn(current_number).add_done_callback(
            lambda future: self._after_turn(future, current_number)
        )

    def _after_turn(self, future, current_number: int) -> None:
        try:
            future.result()
        except Exception as exc:
            self._publish_status('ERROR', current_number, self.member_id)
            self._fail_turn(f'Turn {current_number} failed: {exc}')
            return

        self._busy = False
        self._last_completed_number = current_number
        if current_number == 2 * self.total_members:
            self.get_logger().info('Relay complete: two full cycles have finished.')
            self._publish_status('COMPLETE', current_number, self.member_id)
            rclpy.shutdown()
            return

        next_number = current_number + 1
        next_member = (next_number - 1) % self.total_members + 1
        self._publish_turn(next_number, next_member)
        self._publish_status(
            f'WAITING_FOR_MEMBER_{next_member:02d}', next_number, next_member
        )

    def _publish_turn(self, current_number: int, active_member: int) -> None:
        message = Int32MultiArray()
        message.data = [current_number, active_member]
        self._turn_pub.publish(message)
        self.get_logger().info(
            f'Turn token published: number={current_number}, member={active_member}.'
        )

    def _publish_status(
        self,
        state: str,
        current_number: int | None = None,
        active_member: int | None = None,
    ) -> None:
        message = String()
        message.data = (
            f'member={self.member_id:02d} state={state} '
            f'current_number={current_number} active_member={active_member}'
        )
        self._status_pub.publish(message)
        self.get_logger().info(f'Relay status: {message.data}')

    def set_pen(self, *, off: bool) -> Future:
        request = SetPen.Request()
        request.r, request.g, request.b = self.pen_color
        request.width = 2
        request.off = int(off)
        return self._call_service_when_ready(
            self._pen_client, request, '/turtle1/set_pen'
        )

    def prepare_turn(self, number: int) -> Future:
        """Set this member's pen and lift it before drawing."""
        return self.set_pen(off=True)

    def _call_service_when_ready(
        self, client, request, service_name: str
    ) -> Future:
        """Wait for this node's service client, then call it asynchronously."""
        result: Future = Future()

        def start_call() -> None:
            if result.done():
                return
            try:
                service_future = client.call_async(request)
            except Exception as exc:
                result.set_exception(exc)
                return

            def copy_response(completed) -> None:
                try:
                    result.set_result(completed.result())
                except Exception as exc:
                    result.set_exception(exc)

            service_future.add_done_callback(copy_response)

        if client.service_is_ready():
            start_call()
            return result

        self.get_logger().info(f'Waiting for service {service_name} to become available.')
        timer_ref = {}

        def check_service_ready() -> None:
            if not client.service_is_ready():
                return
            timer = timer_ref['timer']
            timer.cancel()
            self.destroy_timer(timer)
            start_call()

        timer_ref['timer'] = self.create_timer(
            SERVICE_DISCOVERY_POLL_PERIOD,
            check_service_ready,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        return result

    def draw_digit(self, number: int, slot: DigitSlot) -> Future:
        """Draw glyph 4 on the first turn and glyph 13 on the second turn."""
        if number == FIRST_TURN:
            self.glyph = '4'
            self._normalized_strokes = GLYPH_4_STROKES
        else:
            self.glyph = '13'
            self._normalized_strokes = GLYPH_13_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member04)


if __name__ == '__main__':
    main()