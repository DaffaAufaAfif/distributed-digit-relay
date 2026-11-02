"""Member 01: Daffa Aufa Afif (505451022)"""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 1
TOTAL_MEMBERS = 9
VALID_TURNS = (MEMBER_ID, TOTAL_MEMBERS + MEMBER_ID)

# One vertical stroke, normalized to the glyph box.
GLYPH_1_STROKES = (
    ((0.5, 0.0), (0.5, 1.0)),
)

# Digit 10 fits in Member 01's slot: draw the 1 on the left and the 0 on the right.
GLYPH_10_STROKES = (
    ((0.16, 0.0), (0.16, 1.0)),
    (
        (0.53, 0.10),
        (0.86, 0.10),
        (0.96, 0.24),
        (0.96, 0.76),
        (0.86, 0.90),
        (0.53, 0.90),
        (0.43, 0.76),
        (0.43, 0.24),
        (0.53, 0.10),
    ),
)


class Member01(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=(255, 60, 60),
            glyph='1',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_1_STROKES,
        )
        # Service clients belong to this member node. Member 01 owns the
        # /clear call at the start of cycle two and its own pen configuration.
        self._pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self._clear_client = self.create_client(Empty, '/clear')
        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self._on_pose, 10
        )
        # Member 01 kicks off the relay once every take_turn service exists.
        self.start_relay()

    def set_pen(self, *, off: bool) -> Future:
        request = SetPen.Request()
        request.r, request.g, request.b = self.pen_color
        request.width = 2
        request.off = int(off)
        return self._call_service_when_ready(
            self._pen_client, request, '/turtle1/set_pen'
        )

    def prepare_turn(self, number: int) -> Future:
        """Lift the pen; clear the canvas first on Member 01's second-cycle turn."""
        if number == TOTAL_MEMBERS + 1:
            clear_request = Empty.Request()
            clear_future = self._call_service_when_ready(
                self._clear_client, clear_request, '/clear'
            )
            preparation: Future = Future()

            def after_clear(completed) -> None:
                try:
                    completed.result()
                    pen_future = self.set_pen(off=True)
                except Exception as exc:
                    preparation.set_exception(exc)
                    return

                def after_pen(pen_completed) -> None:
                    try:
                        preparation.set_result(pen_completed.result())
                    except Exception as exc:
                        preparation.set_exception(exc)

                pen_future.add_done_callback(after_pen)

            clear_future.add_done_callback(after_clear)
            return preparation
        return self.set_pen(off=True)

    def draw_digit(self, number: int, slot: DigitSlot) -> Future:
        """Draw 1 on turn 1 and 10 on Member 01's cycle-two turn."""
        if number == TOTAL_MEMBERS + MEMBER_ID:
            self.glyph = '10'
            self._normalized_strokes = GLYPH_10_STROKES
        else:
            self.glyph = '1'
            self._normalized_strokes = GLYPH_1_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member01)


if __name__ == '__main__':
    main()
