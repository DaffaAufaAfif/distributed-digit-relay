"""Member 08 (Vincent Valentino): 8 & 17."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 8
TOTAL_MEMBERS = 9
PEN_COLOR = (128, 0, 0)
FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
VALID_TURNS = (FIRST_TURN, SECOND_TURN)
GLYPH_8_STROKES = (
    (
        (0.50, 0.50),
        (0.85, 0.70),
        (0.85, 0.88),
        (0.70, 1.00),
        (0.30, 1.00),
        (0.15, 0.88),
        (0.15, 0.70),
        (0.50, 0.50),
        (0.88, 0.32),
        (0.88, 0.12),
        (0.72, 0.00),
        (0.28, 0.00),
        (0.12, 0.12),
        (0.12, 0.32),
        (0.50, 0.50),
    ),
)

GLYPH_17_STROKES = (
    # Digit '1' (kiri)
    ((0.08, 0.80), (0.18, 1.00), (0.18, 0.00)),
    # Digit '7' (kanan)
    ((0.45, 1.00), (0.95, 1.00), (0.60, 0.00)),
)


class Member08(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='8',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_8_STROKES,
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
        if number == FIRST_TURN:
            self.glyph = '8'
            self._normalized_strokes = GLYPH_8_STROKES
        else:
            self.glyph = '17'
            self._normalized_strokes = GLYPH_17_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member08)


if __name__ == '__main__':
    main()
