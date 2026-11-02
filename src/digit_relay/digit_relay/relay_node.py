"""Abstract base class for one participant in the digit relay."""

from abc import ABC, abstractmethod
from concurrent.futures import Future

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from digit_relay_interfaces.srv import TakeTurn

from .digit_paths import DigitSlot, slot_for_member

SERVICE_DISCOVERY_POLL_PERIOD = 0.1
STATUS_TOPIC = '/digit_relay/status'
FIRST_TURN = 1
FINAL_TURN_MULTIPLIER = 2


def turn_service_name(member_id: int) -> str:
    """Return the take_turn service name that belongs to one member."""
    return f'/digit_relay/member_{member_id:02d}/take_turn'


class DigitRelayNode(Node, ABC):
    """Share member identity, turn handoffs, and Future sequencing.

    Every member exposes its own ``take_turn`` service and keeps a client to
    each peer, so the turn token travels as a service call instead of a
    shared topic. Concrete members still own their turtle clients and
    implement their drawing method.
    """

    def __init__(
        self,
        *,
        member_id: int,
        total_members: int,
        valid_turn_numbers: tuple[int, ...],
    ) -> None:
        super().__init__(f'digit_relay_member_{member_id:02d}')
        self.member_id = member_id
        self.total_members = total_members
        self.valid_turn_numbers = valid_turn_numbers
        self.slot: DigitSlot = slot_for_member(member_id, total_members)
        self._busy = False
        self._last_completed_number = 0
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._turn_service = self.create_service(
            TakeTurn, turn_service_name(member_id), self._on_take_turn
        )
        self._turn_clients = {
            member: self.create_client(TakeTurn, turn_service_name(member))
            for member in range(1, total_members + 1)
            if member != member_id
        }
        self._publish_status('WAITING_FOR_TURN')

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

    def start_relay(self) -> None:
        """Wait until every peer's take_turn service exists, then claim turn 1."""
        self.get_logger().info(
            f'Waiting until all {self.total_members} take_turn services are available.'
        )
        self._start_timer = self.create_timer(
            SERVICE_DISCOVERY_POLL_PERIOD,
            self._try_start_relay,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _try_start_relay(self) -> None:
        missing = [
            member
            for member, client in self._turn_clients.items()
            if not client.service_is_ready()
        ]
        if missing:
            return
        self._start_timer.cancel()
        self.destroy_timer(self._start_timer)
        request = TakeTurn.Request()
        request.current_number = FIRST_TURN
        response = self._on_take_turn(request, TakeTurn.Response())
        if not response.accepted:
            self.get_logger().error(f'Could not start the relay: {response.message}')

    def _on_take_turn(self, request, response):
        """Accept a turn token, start drawing, and reply without blocking.

        The reply only reports token acceptance; drawing keeps running through
        the node's executor so pose feedback and timers stay alive.
        """
        current_number = request.current_number
        if self._busy:
            response.accepted = False
            response.message = f'member {self.member_id:02d} is busy'
            return response
        if current_number not in self.valid_turn_numbers:
            response.accepted = False
            response.message = (
                f'turn {current_number} is not assigned to member {self.member_id:02d}'
            )
            return response
        if current_number <= self._last_completed_number:
            response.accepted = False
            response.message = f'turn {current_number} was already completed'
            return response

        self._busy = True
        self.get_logger().info(f'My turn: drawing {current_number} in slot {self.member_id}.')
        self._publish_status('DRAWING', current_number, self.member_id)
        self.execute_turn(current_number).add_done_callback(
            lambda future: self._after_turn(future, current_number)
        )
        response.accepted = True
        response.message = 'turn accepted'
        return response

    def _after_turn(self, future, current_number: int) -> None:
        try:
            future.result()
        except Exception as exc:
            self._publish_status('ERROR', current_number, self.member_id)
            self._fail_turn(f'Turn {current_number} failed: {exc}')
            return

        self._busy = False
        self._last_completed_number = current_number
        if current_number == FINAL_TURN_MULTIPLIER * self.total_members:
            self.get_logger().info('Relay complete: two full cycles have finished.')
            self._publish_status('COMPLETE', current_number, self.member_id)
            rclpy.shutdown()
            return

        next_number = current_number + 1
        next_member = (next_number - 1) % self.total_members + 1
        self._handoff_turn(next_number, next_member)

    def _handoff_turn(self, current_number: int, active_member: int) -> None:
        """Pass the token to the next member by calling its take_turn service."""
        request = TakeTurn.Request()
        request.current_number = current_number
        self._publish_status(
            f'WAITING_FOR_MEMBER_{active_member:02d}', current_number, active_member
        )
        service_name = turn_service_name(active_member)
        handoff = self._call_service_when_ready(
            self._turn_clients[active_member], request, service_name
        )

        def after_handoff(completed) -> None:
            try:
                response = completed.result()
            except Exception as exc:
                self.get_logger().error(
                    f'Handoff to member {active_member:02d} failed: {exc}'
                )
                return
            if not response.accepted:
                self.get_logger().error(
                    f'Member {active_member:02d} rejected turn {current_number}: '
                    f'{response.message}'
                )
                return
            self.get_logger().info(
                f'Turn token handed to member {active_member:02d}: '
                f'number={current_number}.'
            )

        handoff.add_done_callback(after_handoff)

    def execute_turn(self, number: int) -> Future:
        """Run the member's preparation and drawing hooks in order.

        Service receipt and handoff stay in this base class. This method only
        chains the local asynchronous service and drawing operations.
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

    def _publish_status(
        self,
        state: str,
        current_number: int | None = None,
        active_member: int | None = None,
    ) -> None:
        message = String()
        message.data = (
            f'member={self.member_id:02d} state={state} '
            f'current_number={current_number} active_member={active_member}'
        )
        self._status_pub.publish(message)
        self.get_logger().info(f'Relay status: {message.data}')

    def _call_service_when_ready(
        self, client, request, service_name: str
    ) -> Future:
        """Wait for this node's service client, then call it asynchronously."""
        result: Future = Future()

        def start_call() -> None:
            if result.done():
                return
            try:
                service_future = client.call_async(request)
            except Exception as exc:
                result.set_exception(exc)
                return

            def copy_response(completed) -> None:
                try:
                    result.set_result(completed.result())
                except Exception as exc:
                    result.set_exception(exc)

            service_future.add_done_callback(copy_response)

        if client.service_is_ready():
            start_call()
            return result

        self.get_logger().info(f'Waiting for service {service_name} to become available.')
        timer_ref = {}

        def check_service_ready() -> None:
            if not client.service_is_ready():
                return
            timer = timer_ref['timer']
            timer.cancel()
            self.destroy_timer(timer)
            start_call()

        timer_ref['timer'] = self.create_timer(
            SERVICE_DISCOVERY_POLL_PERIOD,
            check_service_ready,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        return result


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
