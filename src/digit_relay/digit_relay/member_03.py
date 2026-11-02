"""Member 03: draw glyph 3 on its configured turns in cycle one and cycle two."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 3
TOTAL_MEMBERS = 9
PEN_COLOR = (0, 255, 0)  # Hijau

FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
VALID_TURNS = (FIRST_TURN, SECOND_TURN)

# Strokes for digit '3', normalized to [0.0, 1.0] glyph box
# where (0.0, 0.0) is bottom-left and (1.0, 1.0) is top-right.
GLYPH_3_STROKES = (
    # Garis atas ke tengah
    ((0.0, 1.0), (1.0, 1.0), (1.0, 0.5), (0.0, 0.5)),
    # Garis tengah ke bawah
    ((0.0, 0.5), (1.0, 0.5), (1.0, 0.0), (0.0, 0.0)),
)

# Two-digit glyph '12'
GLYPH_12_STROKES = (
    # Digit '1' kiri
    (
        (0.10, 0.80),
        (0.20, 1.00),
        (0.20, 0.00),
    ),
    # Digit '2' kanan
    (
        (0.48, 0.80),
        (0.60, 1.00),
        (0.80, 1.00),
        (0.92, 0.80),
        (0.92, 0.62),
        (0.48, 0.00),
        (0.94, 0.00),
    ),
)


class Member03(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='3',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_3_STROKES,
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
        """Draw glyph 3 on the first turn and glyph 12 on the second turn."""
        if number == FIRST_TURN:
            self.glyph = '3'
            self._normalized_strokes = GLYPH_3_STROKES
        else:
            self.glyph = '12'
            self._normalized_strokes = GLYPH_12_STROKES

        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member03)


if __name__ == '__main__':
    main()