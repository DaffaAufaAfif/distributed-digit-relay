"""Member 05: draw glyph 5 on turn 5 and glyph 14 on turn 14."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 5
TOTAL_MEMBERS = 9
PEN_COLOR = (180, 90, 255)  # Ungu
FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
VALID_TURNS = (FIRST_TURN, SECOND_TURN)

GLYPH_5_STROKES = (
    ((1.0, 1.0), (0.0, 1.0), (0.0, 0.5), (1.0, 0.5), (1.0, 0.0), (0.0, 0.0)),
)
GLYPH_14_STROKES = (
    ((0.1, 0.0), (0.1, 1.0)),                # Angka 1
    ((0.45, 1.0), (0.45, 0.4), (1.0, 0.4)),  # Angka 4 (garis L)
    ((0.8, 1.0), (0.8, 0.0)),                # Angka 4 (garis tegak)
)


class Member05(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='5',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_5_STROKES,
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
        """Draw glyph 5 on the first turn and glyph 14 on the second turn."""
        if number == FIRST_TURN:
            self.glyph = '5'
            self._normalized_strokes = GLYPH_5_STROKES
        else:
            self.glyph = '14'
            self._normalized_strokes = GLYPH_14_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member05)


if __name__ == '__main__':
    main()