"""Member 06: draw glyph 6 on turn 6 and glyph 15 on turn 15."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 6
TOTAL_MEMBERS = 9
FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
PEN_COLOR = (255, 140, 0)  # Orange
VALID_TURNS = (FIRST_TURN, SECOND_TURN)

# Single continuous stroke for digit '6', normalized to [0.0, 1.0] glyph box
# where (0.0, 0.0) is bottom-left and (1.0, 1.0) is top-right.
GLYPH_6_STROKES = (
    (
        (0.85, 0.90),
        (0.60, 1.00),
        (0.30, 1.00),
        (0.12, 0.80),
        (0.12, 0.25),
        (0.30, 0.00),
        (0.70, 0.00),
        (0.88, 0.20),
        (0.88, 0.42),
        (0.70, 0.60),
        (0.30, 0.60),
        (0.12, 0.42),
    ),
)

# Two-digit '15': '1' in the left part, '5' in the right part of the glyph box.
GLYPH_15_STROKES = (
    # Digit '1' (kiri)
    ((0.12, 0.80), (0.25, 1.00), (0.25, 0.00)),
    # Digit '5' (kanan)
    (
        (0.90, 1.00),
        (0.58, 1.00),
        (0.55, 0.55),
        (0.75, 0.62),
        (0.90, 0.50),
        (0.90, 0.15),
        (0.75, 0.00),
        (0.58, 0.00),
        (0.52, 0.12),
    ),
)


class Member06(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='6',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_6_STROKES,
        )
        # Each member creates its own client for /turtle1/set_pen.
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
        """Draw glyph 6 on the first turn and glyph 15 on the second turn."""
        if number == FIRST_TURN:
            self.glyph = '6'
            self._normalized_strokes = GLYPH_6_STROKES
        else:
            self.glyph = '15'
            self._normalized_strokes = GLYPH_15_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member06)


if __name__ == '__main__':
    main()