"""Member 09: draw glyph 9 on turn 9 and glyph 18 on the final turn 18."""

from concurrent.futures import Future

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from .digit_drawing import DigitDrawingMember
from .digit_paths import DigitSlot
from .relay_node import run_member

MEMBER_ID = 9
TOTAL_MEMBERS = 9
PEN_COLOR = (255, 255, 255)  # Putih
FIRST_TURN = MEMBER_ID
SECOND_TURN = TOTAL_MEMBERS + MEMBER_ID
VALID_TURNS = (FIRST_TURN, SECOND_TURN)

GLYPH_9_STROKES = (
    # Lingkaran atas, lalu ekor turun ke bawah
    (
        (0.90, 0.60), (0.90, 0.82), (0.72, 1.00), (0.28, 1.00), (0.10, 0.82),
        (0.10, 0.60), (0.28, 0.42), (0.72, 0.42), (0.90, 0.60),
        (0.90, 0.18), (0.72, 0.00), (0.28, 0.00), (0.10, 0.16),
    ),
)
GLYPH_18_STROKES = (
    ((0.02, 0.82), (0.14, 1.00), (0.14, 0.00)),  # Angka 1
    # Angka 8: loop atas lalu loop bawah, menyilang di (0.72, 0.52)
    (
        (0.72, 0.52), (0.96, 0.66), (0.96, 0.88), (0.84, 1.00), (0.60, 1.00),
        (0.48, 0.88), (0.48, 0.66), (0.72, 0.52), (0.96, 0.38), (0.96, 0.12),
        (0.84, 0.00), (0.60, 0.00), (0.48, 0.12), (0.48, 0.38), (0.72, 0.52),
    ),
)


class Member09(DigitDrawingMember):
    def __init__(self) -> None:
        super().__init__(
            member_id=MEMBER_ID,
            total_members=TOTAL_MEMBERS,
            pen_color=PEN_COLOR,
            glyph='9',
            valid_turn_numbers=VALID_TURNS,
            normalized_strokes=GLYPH_9_STROKES,
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
        """Draw glyph 9 on the first turn and glyph 18 on the second turn."""
        if number == FIRST_TURN:
            self.glyph = '9'
            self._normalized_strokes = GLYPH_9_STROKES
        else:
            self.glyph = '18'
            self._normalized_strokes = GLYPH_18_STROKES
        return self._draw_configured_glyph(number, slot)


def main(args=None) -> None:
    run_member(Member09)


if __name__ == '__main__':
    main()
