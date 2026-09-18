"""Deterministic lifecycle tests; no ROS, network or physical hardware."""

from concurrent.futures import Future
from pathlib import Path
import runpy
from types import SimpleNamespace as NS
import unittest

FrankaHand = runpy.run_path(str(Path(__file__).resolve().parents[1] /
    'fr3_lab_stack_runtime/franka_hand.py'))['FrankaHand']


def done(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancel = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return self.cancel

    def finish(self, status=4, success=True):
        self.result.set_result(NS(status=status, result=NS(success=success,
                                                          error='test')))


class Transport:
    def __init__(self):
        self.moves = []
        self.grasps = []
        self.goals = []
        self.stops = 0
        self.stop_response = NS(success=True, message='')

    def move(self, width, speed):
        self.moves.append((width, speed))
        future = Future()
        self.goals.append(future)
        return future

    def grasp(self, width, speed, force, epsilon_inner, epsilon_outer):
        self.grasps.append(
            (width, speed, force, epsilon_inner, epsilon_outer)
        )
        future = Future()
        self.goals.append(future)
        return future

    def stop(self):
        self.stops += 1
        return done(self.stop_response)


class HandTests(unittest.TestCase):
    def setUp(self):
        self.transport = Transport()
        self.time = 0.
        self.hand = FrankaHand(self.transport, maximum_width=.08, speed=.1,
                               now=lambda: self.time)

    def accept(self):
        handle = Handle()
        self.transport.goals[-1].set_result(handle)
        self.hand.tick()
        return handle

    def test_queue_is_nonblocking_and_duplicates_are_suppressed(self):
        first = self.hand.request_move(.08)
        self.assertEqual(self.transport.moves, [])
        self.assertFalse(self.hand.settled)
        second = self.hand.request_move(.08)
        self.assertEqual(second['duplicate_of'], first['request_id'])
        self.hand.tick()
        handle = self.accept()
        handle.finish()
        self.hand.tick()
        self.hand.request_move(.08)
        self.hand.tick()
        self.assertEqual(self.transport.moves, [(.08, .1)])

    def test_reversal_waits_for_accept_cancel_and_terminal_result(self):
        self.hand.request_move(0.)
        self.hand.tick()
        self.hand.request_move(.08)
        self.hand.tick()
        self.assertEqual(len(self.transport.moves), 1)
        handle = self.accept()
        self.assertEqual(handle.cancels, 1)
        handle.finish(5, False)
        self.hand.tick()
        self.assertEqual(len(self.transport.moves), 1)
        handle.cancel.set_result(NS(return_code=0, goals_canceling=[1]))
        self.hand.tick()
        self.assertEqual(self.transport.moves, [(0., .1), (.08, .1)])

    def test_latest_request_wins_during_cancellation(self):
        self.hand.request_move(0.)
        self.hand.tick()
        handle = self.accept()
        self.hand.request_move(.08)
        self.hand.tick()
        self.hand.request_move(0.)
        handle.cancel.set_result(NS(return_code=0, goals_canceling=[1]))
        handle.finish(5, False)
        self.hand.tick()
        self.assertEqual(self.transport.moves, [(0., .1), (0., .1)])

    def test_rejection_latches_failure_and_stops(self):
        self.hand.request_move(0.)
        self.hand.tick()
        self.transport.goals[0].set_result(NS(accepted=False))
        self.hand.tick()
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.hand.request_move(.08)
        self.hand.tick()
        self.assertEqual(self.transport.stops, 1)

    def test_unsuccessful_result_aborts_and_does_not_reopen_automatically(self):
        self.hand.request_move(0.)
        self.hand.tick()
        handle = self.accept()
        handle.finish(6, False)
        self.hand.tick()
        self.assertIn('move_failed', self.hand.error)
        self.hand.tick()
        self.assertEqual(self.transport.stops, 1)
        self.assertEqual(len(self.transport.moves), 1)

    def test_cancel_rejected_never_allows_overlap_and_times_out(self):
        self.hand.request_move(0.)
        self.hand.tick()
        handle = self.accept()
        self.hand.request_move(.08)
        self.hand.tick()
        handle.cancel.set_result(NS(return_code=1, goals_canceling=[]))
        self.hand.tick()
        self.time = 4.
        self.hand.tick()
        self.assertIn('timeout', self.hand.error)
        self.assertEqual(len(self.transport.moves), 1)
        self.assertEqual(self.transport.stops, 1)

    def test_stop_during_goal_acceptance_cancels_late_goal(self):
        self.hand.request_move(0.)
        self.hand.tick()
        self.hand.stop()
        self.hand.tick()
        self.assertFalse(self.hand.settled)
        handle = self.accept()
        self.assertEqual(handle.cancels, 1)
        handle.cancel.set_result(NS(return_code=0, goals_canceling=[1]))
        handle.finish(5, False)
        self.hand.tick()
        self.assertTrue(self.hand.settled)
        self.assertEqual(self.transport.stops, 1)

    def test_stop_failure_is_recorded(self):
        self.transport.stop_response = NS(success=False, message='failed')
        self.hand.stop()
        self.hand.tick()
        self.assertIn('stop_failed', self.hand.error)

    def test_completion_cancel_race_accepts_success_after_rejected_cancel(self):
        self.hand.request_move(0.)
        self.hand.tick()
        handle = self.accept()
        self.hand.request_move(.08)
        self.hand.tick()
        handle.cancel.set_result(NS(return_code=3, goals_canceling=[]))
        handle.finish()
        self.hand.tick()
        self.assertEqual(len(self.transport.moves), 2)
        self.assertIsNone(self.hand.error)

    def test_transport_exception_latches_and_stops(self):
        def broken(width, speed):
            raise RuntimeError('connection lost')
        self.transport.move = broken
        self.hand.request_move(.08)
        self.hand.tick()
        with self.assertRaisesRegex(RuntimeError, 'connection lost'):
            self.hand.check()
        self.hand.tick()
        self.assertEqual(self.transport.stops, 1)
        self.assertFalse(self.hand.settled)

    def test_cancel_acknowledgement_is_audited_before_terminal_result(self):
        self.hand.request_move(0.)
        self.hand.tick()
        handle = self.accept()
        self.hand.request_move(.08)
        self.hand.tick()
        handle.cancel.set_result(NS(return_code=0, goals_canceling=[1]))
        self.hand.tick()
        events = self.hand.evidence()['events']
        self.assertTrue(any(e['kind'] == 'cancel_response' for e in events))
        self.assertEqual(len(self.transport.moves), 1)

    def test_invalid_inputs(self):
        for value in (-1., .09, float('nan'), float('inf')):
            with self.subTest(move_width=value):
                with self.assertRaises(ValueError):
                    self.hand.request_move(value)

        for force in (0., -1., float('nan'), float('inf')):
            with self.subTest(force=force):
                with self.assertRaises(ValueError):
                    self.hand.request_grasp(
                        0.,
                        force=force,
                        epsilon_inner=0.,
                        epsilon_outer=.08,
                    )

        for epsilon_inner, epsilon_outer in (
            (-.001, .08),
            (0., -.001),
            (float('nan'), .08),
            (0., float('inf')),
        ):
            with self.subTest(
                epsilon_inner=epsilon_inner,
                epsilon_outer=epsilon_outer,
            ):
                with self.assertRaises(ValueError):
                    self.hand.request_grasp(
                        0.,
                        force=20.,
                        epsilon_inner=epsilon_inner,
                        epsilon_outer=epsilon_outer,
                    )

    def test_grasp_dispatches_explicit_parameters(self):
        self.hand.request_grasp(
            0.,
            force=20.,
            epsilon_inner=0.,
            epsilon_outer=.08,
        )
        self.assertEqual(self.transport.grasps, [])

        self.hand.tick()

        self.assertEqual(
            self.transport.grasps,
            [(0., .1, 20., 0., .08)],
        )

        handle = self.accept()
        handle.finish()
        self.hand.tick()

        request = self.hand.evidence()['requests'][0]
        self.assertEqual(request['command_type'], 'grasp')
        self.assertEqual(request['disposition'], 'completed')

    def test_move_and_grasp_same_width_are_not_duplicates(self):
        self.hand.request_move(0.)
        self.hand.tick()
        handle = self.accept()
        handle.finish()
        self.hand.tick()

        request = self.hand.request_grasp(
            0.,
            force=20.,
            epsilon_inner=0.,
            epsilon_outer=.08,
        )

        self.assertEqual(request['disposition'], 'queued')

    def test_successful_grasp_is_stopped_before_open_move(self):
        self.hand.request_grasp(
            0.,
            force=20.,
            epsilon_inner=.001,
            epsilon_outer=.08,
        )
        self.hand.tick()

        handle = self.accept()
        handle.finish()
        self.hand.tick()

        self.assertTrue(self.hand.holding_grasp)
        self.assertEqual(self.transport.stops, 0)

        self.hand.request_move(.08)
        self.hand.tick()

        # Stop requested, but open Move must not overlap it.
        self.assertEqual(self.transport.stops, 1)
        self.assertEqual(self.transport.moves, [])

        self.hand.tick()

        self.assertFalse(self.hand.holding_grasp)
        self.assertEqual(self.transport.moves, [(.08, .1)])

        events = self.hand.evidence()['events']
        kinds = [event['kind'] for event in events]

        self.assertLess(
            kinds.index('release_stop_requested'),
            kinds.index('release_stop_result'),
        )
        self.assertLess(
            kinds.index('release_stop_result'),
            kinds.index('move_requested'),
        )

    def test_failed_release_stop_blocks_open_move(self):
        self.hand.request_grasp(
            0.,
            force=20.,
            epsilon_inner=.001,
            epsilon_outer=.08,
        )
        self.hand.tick()

        handle = self.accept()
        handle.finish()
        self.hand.tick()

        self.transport.stop_response = NS(success=False, message='release failed')

        self.hand.request_move(.08)
        self.hand.tick()
        self.hand.tick()

        self.assertIn('release_stop_failed', self.hand.error)
        self.assertEqual(self.transport.moves, [])


if __name__ == '__main__':
    unittest.main()
