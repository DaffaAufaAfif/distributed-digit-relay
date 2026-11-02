"""Member 04: draw glyph 4 on turn 4 and glyph 13 on turn 13."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

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