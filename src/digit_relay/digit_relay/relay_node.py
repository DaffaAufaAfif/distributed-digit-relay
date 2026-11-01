"""Abstract base class for one participant in the digit relay."""

from abc import ABC, abstractmethod
from concurrent.futures import Future

import rclpy
from rclpy.node import Node

from .digit_paths import DigitSlot, slot_for_member


class DigitRelayNode(Node, ABC):
    """Share member identity and Future sequencing without owning ROS topics.

    Each concrete member creates its own topic publisher/subscriber, processes
    turn tokens, calls its services, and implements its drawing method.
    """

    def __init__(self, *, member_id: int, total_members: int) -> None:
        super().__init__(f'digit_relay_member_{member_id:02d}')
        self.member_id = member_id
        self.total_members = total_members
        self.slot: DigitSlot = slot_for_member(member_id, total_members)
        self._busy = False

    @abstractmethod
    def prepare_turn(self, number: int) -> Future:
        """Run this member's service calls before drawing and return a Future."""
        raise NotImplementedError

    @abstractmethod
    def set_pen(self, *, off: bool) -> Future:
        """Call ``/turtle1/set_pen`` using this member's own service client."""
        raise NotImplementedError

    @abstractmethod
    def draw_digit(self, number: int, slot: DigitSlot) -> Future:
        """Draw this member's glyph and resolve the Future when complete."""
        raise NotImplementedError

    def execute_turn(self, number: int) -> Future:
        """Run the member's preparation and drawing hooks in order.

        Topic receipt and publication stay in the concrete member. This method
        only chains the local asynchronous service and drawing operations.
        """
        completed_turn: Future = Future()

        try:
            preparation = self.prepare_turn(number)
        except Exception as exc:
            completed_turn.set_exception(exc)
            return completed_turn

        def after_preparation(future) -> None:
            try:
                future.result()
                drawing = self.draw_digit(number, self.slot)
            except Exception as exc:
                completed_turn.set_exception(exc)
                return

            def after_drawing(drawing_future) -> None:
                try:
                    completed_turn.set_result(drawing_future.result())
                except Exception as exc:
                    completed_turn.set_exception(exc)

            drawing.add_done_callback(after_drawing)

        preparation.add_done_callback(after_preparation)
        return completed_turn

    def _fail_turn(self, reason: str) -> None:
        self._busy = False
        self.get_logger().error(reason)
        # Do not pass the turn after a failed service or drawing operation.


def run_member(member_type: type[DigitRelayNode]) -> None:
    """Initialize ROS, spin one member node, and clean up on exit."""
    rclpy.init()
    node = member_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
