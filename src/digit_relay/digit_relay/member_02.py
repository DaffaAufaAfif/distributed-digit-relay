"""Member 02: draw glyph 2 on its configured turns in cycle one and cycle two."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 2
TOTAL_MEMBERS = 9
FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
PEN_COLOR = (255, 105, 180)  # Pink (HotPink)
VALID_TURNS = (FIRST_TURN, SECOND_TURN)

# Single continuous stroke for digit '2', normalized to [0.0, 1.0] glyph box
# where (0.0, 0.0) is bottom-left and (1.0, 1.0) is top-right.
GLYPH_2_STROKES = (
    (
        (0.15, 0.80),
        (0.35, 1.00),
        (0.70, 1.00),
        (0.88, 0.82),
        (0.88, 0.62),
        (0.12, 0.00),
        (0.90, 0.00),
    ),
)

# Two parallel vertical strokes with slanted head for two-digit '11'
GLYPH_11_STROKES = (
    # Digit '1' pertama (kiri)
    ((0.20, 0.80), (0.30, 1.00), (0.30, 0.00)),
    # Digit '1' kedua (kanan)
    ((0.60, 0.80), (0.70, 1.00), (0.70, 0.00)),
)


class Member02(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='2',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_2_STROKES,
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
        """Draw glyph 2 on the first turn and glyph 11 on the second turn."""
        if number == FIRST_TURN:
            self.glyph = '2'
            self._normalized_strokes = GLYPH_2_STROKES
        else:
            self.glyph = '11'
            self._normalized_strokes = GLYPH_11_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member02)


if __name__ == '__main__':
    main()
