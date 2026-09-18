"""Policy-independent asynchronous Franka Hand `Move & Grasp` lifecycle.

Call request/check from the scheduler and tick from a single ROS executor.
No future wait or spin occurs here. A replacement waits for both cancellation
acknowledgement and the old terminal result; sending goals is not preemption.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Any, Callable


class FrankaHand:
    def __init__(self, transport: Any, *, maximum_width: float,
                 speed: float, timeout: float = 3.0,
                 now: Callable[[], float] = time.monotonic) -> None:
        if any(not math.isfinite(v) or v <= 0
               for v in (maximum_width, speed, timeout)):
            raise ValueError('Positive finite width, speed and timeout required')
        self.transport = transport
        self.maximum_width = maximum_width
        self.speed = speed
        self.timeout = timeout
        self.now = now
        self.lock = threading.RLock()
        self.desired = None
        self.active = None
        self.goal = self.result = self.cancel = self.stop_future = None
        self.release_stop_future = None
        self.holding_grasp = False
        self.handle = None
        self.deadline = None
        self.error = None
        self.stopping = False
        self._stop_sent = False
        self.cancel_logged = False
        self.events: list[dict[str, Any]] = []
        self.requests: list[dict[str, Any]] = []

    def _request(
        self,
        *,
        command_type: str,
        width: float,
        force: float | None,
        epsilon_inner: float | None,
        epsilon_outer: float | None,
    ) -> dict[str, Any]:
        if command_type not in ('move', 'grasp'):
            raise ValueError('Invalid gripper command type')
        if not math.isfinite(width) or not 0 <= width <= self.maximum_width:
            raise ValueError('Invalid gripper width')

        command = dict(
            command_type=command_type,
            target_width_m=float(width),
            speed_m_s=self.speed,
            force_n=force,
            epsilon_inner_m=epsilon_inner,
            epsilon_outer_m=epsilon_outer,
        )

        with self.lock:
            self.check()

            duplicate = (
                self.desired is not None
                and all(
                    self.requests[self.desired].get(key) == value
                    for key, value in command.items()
                )
            )

            record = dict(
                request_id=len(self.requests),
                **command,
                requested_monotonic_seconds=self.now(),
                disposition='duplicate' if duplicate else 'queued',
                command_issued=False,
                issue_monotonic_seconds=None,
            )

            if not duplicate:
                if self.desired is not None and self.desired != self.active:
                    old = self.requests[self.desired]
                    if old['disposition'] == 'queued':
                        old['disposition'] = 'superseded_before_issue'
                self.desired = record['request_id']
            else:
                record['duplicate_of'] = self.desired

            self.requests.append(record)
            return dict(record)

    def event(self, kind: str, **fields: Any) -> None:
        self.events.append(dict(kind=kind, monotonic_seconds=self.now(),
                                request_id=self.active, **fields))

    def check(self) -> None:
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.stopping:
                raise RuntimeError('gripper_stopping')

    def request_move(self, width: float) -> dict[str, Any]:
        return self._request(
            command_type='move',
            width=width,
            force=None,
            epsilon_inner=None,
            epsilon_outer=None,
        )

    def request_grasp(
        self,
        width: float,
        *,
        force: float,
        epsilon_inner: float,
        epsilon_outer: float,
    ) -> dict[str, Any]:
        for name, value in (
            ('force', force),
            ('epsilon_inner', epsilon_inner),
            ('epsilon_outer', epsilon_outer),
        ):
            if not math.isfinite(value):
                raise ValueError(f'Non-finite gripper {name}')
        if force <= 0:
            raise ValueError('Gripper grasp force must be positive')
        if epsilon_inner < 0 or epsilon_outer < 0:
            raise ValueError('Gripper grasp epsilon must be non-negative')

        return self._request(
            command_type='grasp',
            width=width,
            force=float(force),
            epsilon_inner=float(epsilon_inner),
            epsilon_outer=float(epsilon_outer),
        )

    def stop(self) -> None:
        with self.lock:
            self.stopping = True
            self.desired = None

    def fail(self, message: str) -> None:
        if self.error is None:
            self.error = message
            self.event('error', error=message)
        self.stopping = True
        self.desired = None

    @property
    def settled(self) -> bool:
        with self.lock:
            return (
                self.active is None
                and self.goal is None
                and self.stop_future is None
                and self.release_stop_future is None
                and (
                    self.desired is None
                    or self.requests[self.desired]['disposition'] != 'queued'
                )
                and (not self.stopping or self._stop_sent)
            )

    def tick(self) -> None:
        with self.lock:
            try:
                self._tick()
            except Exception as error:
                # Transport exceptions are latched for the scheduler, never lost
                # in an executor callback. Next tick attempts the stop service.
                self.fail(f'{type(error).__name__}: {error}')

    def _tick(self) -> None:

        if self.deadline is not None and self.now() >= self.deadline:
            self.fail('gripper_lifecycle_timeout')
            self.deadline = None

        if self.stopping and not self._stop_sent:
            self._stop_sent = True
            self.stop_future = self.transport.stop()
            self.event('stop_requested')

        if self.stop_future is not None and self.stop_future.done():
            response = self.stop_future.result()
            self.stop_future = None
            self.event('stop_result', success=response.success,
                       message=response.message)
            if not response.success:
                self.fail('gripper_stop_failed: ' + response.message)

        if self.release_stop_future is not None and self.release_stop_future.done():
            response = self.release_stop_future.result()
            self.release_stop_future = None
            self.event(
                'release_stop_result',
                success=response.success,
                message=response.message,
            )

            if not response.success:
                self.fail('gripper_release_stop_failed: ' + response.message)
                return

            self.holding_grasp = False
            self.deadline = None

            # If shutdown was requested while this stop was in flight,
            # this successful physical stop also satisfies that request.
            if self.stopping:
                self._stop_sent = True

        if self.goal is not None and self.goal.done():
            future, self.goal = self.goal, None
            self.handle = future.result()
            self.event('goal_response', accepted=self.handle.accepted)
            if not self.handle.accepted:
                self.active = self.handle = None
                self.deadline = None
                self.fail('gripper_goal_rejected')
            else:
                self.result = self.handle.get_result_async()
        replacing = self.active is not None and self.desired != self.active

        if replacing and self.handle is not None and self.cancel is None:
            self.cancel = self.handle.cancel_goal_async()
            self.event('cancel_requested')
            self.deadline = self.now() + self.timeout

        if self.cancel is not None and self.cancel.done() and not self.cancel_logged:
            cancellation = self.cancel.result()
            self.event('cancel_response', return_code=cancellation.return_code,
                       goals_canceling=len(cancellation.goals_canceling))
            self.cancel_logged = True

        if self.result is not None and self.result.done():
            # Do not release the slot while cancel may still reach the server.
            if self.cancel is not None and not self.cancel.done():
                return
            response = self.result.result()
            self.event('result', status=response.status,
                       success=response.result.success,
                       error=response.result.error)

            expected_cancel = self.cancel is not None and response.status == 5
            success = response.status == 4 and response.result.success
            command_type = self.requests[self.active]['command_type']

            if success:
                if command_type == 'grasp':
                    self.holding_grasp = True
                elif command_type == 'move':
                    self.holding_grasp = False

            if expected_cancel:
                self.holding_grasp = False

            if not success and not expected_cancel and not self.stopping:
                self.fail(f'gripper_{command_type}_failed: ' + response.result.error)

            self.requests[self.active]['disposition'] = (
                'completed' if success else 'cancelled' if expected_cancel
                else 'failed')
            self.active = self.handle = self.result = self.cancel = None
            self.cancel_logged = False
            self.deadline = None

        if self.stopping or self.active is not None or self.desired is None:
            return

        record = self.requests[self.desired]
        if record['disposition'] != 'queued':
            return

        if self.holding_grasp:
            if self.release_stop_future is None:
                self.release_stop_future = self.transport.stop()
                self.event(
                    'release_stop_requested',
                    next_command_type=record['command_type'],
                )
                self.deadline = self.now() + self.timeout
            return

        self.active = self.desired
        record.update(command_issued=True, disposition='issued',
                      issue_monotonic_seconds=self.now())
        self.deadline = self.now() + self.timeout
        command_type = record['command_type']

        if command_type == 'move':
            self.event(
                'move_requested',
                width=record['target_width_m'],
                speed=record['speed_m_s'],
            )
            self.goal = self.transport.move(
                record['target_width_m'],
                record['speed_m_s'],
            )

        elif command_type == 'grasp':
            self.event(
                'grasp_requested',
                width=record['target_width_m'],
                speed=record['speed_m_s'],
                force=record['force_n'],
                epsilon_inner=record['epsilon_inner_m'],
                epsilon_outer=record['epsilon_outer_m'],
            )
            self.goal = self.transport.grasp(
                record['target_width_m'],
                record['speed_m_s'],
                record['force_n'],
                record['epsilon_inner_m'],
                record['epsilon_outer_m'],
            )

        else:
            raise RuntimeError(f'Unsupported gripper command: {command_type}')

    def evidence(self) -> dict[str, Any]:
        with self.lock:
            return copy.deepcopy(dict(requests=self.requests, events=self.events,
                                      error=self.error, settled=self.settled))


class RosFrankaHand(FrankaHand):
    """Attach to an existing node/executor; caller owns executor lifetime."""

    def __init__(self, node: Any, *, maximum_width: float, speed: float,
                 timeout: float = 3.0, namespace: str = '/franka_gripper') -> None:
        from franka_msgs.action import Move, Grasp
        from rclpy.action import ActionClient
        from std_srvs.srv import Trigger

        move_client = ActionClient(node, Move, namespace + '/move')
        grasp_client = ActionClient(node, Grasp, namespace + '/grasp')
        stop_client = node.create_client(Trigger, namespace + '/stop')

        class Transport:
            def move(self, width: float, speed: float) -> Any:
                if not move_client.server_is_ready():
                    raise RuntimeError('gripper_move_unavailable')
                goal = Move.Goal(width=width, speed=speed)
                return move_client.send_goal_async(goal)

            def grasp(
                self,
                width: float,
                speed: float,
                force: float,
                epsilon_inner: float,
                epsilon_outer: float,
            ) -> Any:
                if not grasp_client.server_is_ready():
                    raise RuntimeError('gripper_grasp_unavailable')

                goal = Grasp.Goal(
                    width=width,
                    speed=speed,
                    force=force,
                )
                goal.epsilon.inner = epsilon_inner
                goal.epsilon.outer = epsilon_outer
                return grasp_client.send_goal_async(goal)

            def stop(self) -> Any:
                if not stop_client.service_is_ready():
                    raise RuntimeError('gripper_stop_unavailable')
                return stop_client.call_async(Trigger.Request())

        super().__init__(Transport(), maximum_width=maximum_width,
                         speed=speed, timeout=timeout)
        self.move_client = move_client
        self.grasp_client = grasp_client
        self.stop_client = stop_client
        self.timer = node.create_timer(.005, self.tick)

    def ready(self) -> bool:
        return (
            self.move_client.server_is_ready()
            and self.grasp_client.server_is_ready()
            and self.stop_client.service_is_ready()
        )
