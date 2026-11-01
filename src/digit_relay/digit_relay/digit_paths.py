"""Shared canvas dimensions and slot allocation for member drawing code."""

# Turtlesim's visible canvas uses coordinates from 0.0 to about 11.0.
CANVAS_WIDTH = 11.0
CANVAS_HEIGHT = 11.0
CANVAS_MARGIN = 0.8

# A fixed 3x3 grid reserves a predictable slot for each of nine members.
SLOT_COLUMNS = 3
SLOT_ROWS = 3
SLOT_GAP_X = 0.35
SLOT_GAP_Y = 0.35
MAX_MEMBERS = SLOT_COLUMNS * SLOT_ROWS

from typing import NamedTuple


class DigitSlot(NamedTuple):
    """One non-overlapping canvas cell, centered at ``(x, y)``."""

    x: float
    y: float
    width: float
    height: float


def _slot_size(canvas_size: float, margin: float, gap: float, count: int) -> float:
    """Calculate a cell's usable dimension after margins and inter-cell gaps."""
    return (canvas_size - 2 * margin - gap * (count - 1)) / count


SLOT_WIDTH = _slot_size(CANVAS_WIDTH, CANVAS_MARGIN, SLOT_GAP_X, SLOT_COLUMNS)
SLOT_HEIGHT = _slot_size(CANVAS_HEIGHT, CANVAS_MARGIN, SLOT_GAP_Y, SLOT_ROWS)


def slot_for_member(member_id: int, total_members: int) -> DigitSlot:
    """Return a stable row-major 3x3 slot for a 1-based member ID.

    Members 1–3 occupy the top row, 4–6 the middle row, and 7–9 the bottom
    row. Slot centers are derived from the canvas dimensions and margins.
    """
    if total_members < 1 or total_members > MAX_MEMBERS:
        raise ValueError(f'total_members must be between 1 and {MAX_MEMBERS}')
    if member_id < 1 or member_id > total_members:
        raise ValueError('member_id must be between 1 and total_members')

    row, column = divmod(member_id - 1, SLOT_COLUMNS)
    x = CANVAS_MARGIN + SLOT_WIDTH / 2 + column * (SLOT_WIDTH + SLOT_GAP_X)
    # Row zero is the top row in turtlesim's upward-increasing y coordinates.
    y = CANVAS_HEIGHT - CANVAS_MARGIN - SLOT_HEIGHT / 2 - row * (SLOT_HEIGHT + SLOT_GAP_Y)
    return DigitSlot(x=x, y=y, width=SLOT_WIDTH, height=SLOT_HEIGHT)
