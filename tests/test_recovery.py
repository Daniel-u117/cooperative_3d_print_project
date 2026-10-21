import contextlib
import io
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import requests

from util.command_script import command_script
from util.execution import execution_worker, recover_printers, send_gcode


class Controller:
    """Small firmware buffer, delayed motion, and a single-use completion reply."""

    def __init__(self):
        self.now = 0.0
        self.stopped = False
        self.ip = {1: 'robot-a', 2: 'robot-b'}
        self.positions = {1: (100.0, 100.0), 2: (200.0, 100.0)}
        self.ready = {p: 0.0 for p in self.ip}
        self.moves = {}
        self.targets = {}
        self.marker_counts = {p: 0 for p in self.ip}
        self.drop_first_marker = False
        self.reply_override = None
        self.markers = {}
        self.commands = []
        self.omit_marker = False
        self.offset = 0.0
        self.full = False
        self.lose_move_response = False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        self.now += seconds

    def get(self, url, **kwargs):
        host, endpoint = url.removeprefix('http://').split('/')
        p = next(p for p, value in self.ip.items() if value == host)
        response = Mock(text='')
        if endpoint == 'rr_gcode':
            command = kwargs['params']['gcode']
            if not command:
                response.json.return_value = {'buff': 0 if self.full or self.now < self.ready[p] else 64}
                return response
            # The old multi-line recovery request cannot fit in this controller.
            assert len(command.encode('utf-8')) + 1 <= 64
            assert self.now >= self.ready[p]
            self.commands.append((p, command.strip(), self.now))
            self.ready[p] = self.now + 0.1
            if command.startswith('G53 G1 '):
                self.moves[p] = self.now + 1.0
                words = command.split()
                self.targets[p] = tuple(float(next(w[1:] for w in words if w.startswith(axis)))
                                        for axis in ('X', 'Y'))
                if self.lose_move_response:
                    raise requests.Timeout('Motion may already have been accepted')
            if command.startswith('echo '):
                self.marker_counts[p] += 1
                self.markers[p] = command.split('"')[1]
                if self.drop_first_marker and self.marker_counts[p] == 1:
                    self.markers.pop(p)
            response.json.return_value = {'buff': 0}
        elif endpoint == 'rr_reply':
            if p in self.moves and self.now >= self.moves[p] and not self.omit_marker:
                response.text = self.markers.pop(p, '')
                if self.reply_override is not None:
                    response.text = self.reply_override
        else:
            raise AssertionError(f'Unexpected request: {url}')
        return response

    def status(self, connection, **kwargs):
        statuses = []
        for p in self.ip:
            # Firmware can report busy briefly after emitting the M400 marker.
            busy = p in self.moves and self.now < self.moves[p] + 0.2
            statuses.append('B' if busy else 'I')
            if p in self.moves and self.now >= self.moves[p]:
                self.positions[p] = (self.targets[p][0] + self.offset, self.targets[p][1])
        return statuses, [self.positions[p] for p in self.ip]


@contextlib.contextmanager
def controller_io(controller):
    with patch('util.execution.time.monotonic', side_effect=lambda: controller.now), \
         patch('util.execution.read_status', side_effect=controller.status), \
         patch('util.execution.requests.get', side_effect=controller.get), \
         patch('util.execution.RECOVERY_TIMEOUT', 20.0), \
         contextlib.redirect_stdout(io.StringIO()):
        yield


class RecoveryTests(unittest.TestCase):
    def test_small_buffer_and_delayed_idle_finish_both_routes_once(self):
        controller = Controller()
        with controller_io(controller):
            observed = recover_printers(controller.ip, controller)
        self.assertEqual(observed, {1: (275.0, 25.0), 2: (275.0, 25.0)})
        moves = [(p, when) for p, command, when in controller.commands if command.startswith('G53 G1 ')]
        self.assertEqual([p for p, _ in moves], [1, 2])
        self.assertGreaterEqual(moves[1][1], controller.moves[1] + 0.2)
        self.assertEqual(len(controller.commands), 8)

    def test_idle_without_marker_does_not_start_second_robot(self):
        controller = Controller()
        controller.omit_marker = True
        with controller_io(controller), self.assertRaisesRegex(TimeoutError, 'marker=False'):
            recover_printers(controller.ip, controller)
        self.assertEqual(set(controller.moves), {1})
        self.assertEqual(controller.marker_counts[1], 2)
        self.assertLess(controller.now, 8.0)

    def test_lost_initial_markers_retry_only_barrier_and_echo(self):
        controller = Controller()
        controller.drop_first_marker = True
        output = io.StringIO()
        with controller_io(controller), contextlib.redirect_stdout(output):
            observed = recover_printers(controller.ip, controller)
        self.assertEqual(observed, {1: (275.0, 25.0), 2: (275.0, 25.0)})
        for p in controller.ip:
            commands = [c for q, c, _ in controller.commands if q == p]
            self.assertEqual(len([c for c in commands if c.startswith('G53 G1 ')]), 1)
            self.assertEqual(commands[-2], 'M400')
            self.assertTrue(commands[-1].startswith('echo '))
            self.assertNotEqual(commands[3], commands[-1])
        self.assertEqual(controller.marker_counts, {1: 2, 2: 2})
        self.assertIn('RECOVERY_ACK_RETRY p=1', output.getvalue())
        self.assertIn('RECOVERY_REPLY p=1', output.getvalue())
        self.assertIn('RECOVERY_DONE p=2', output.getvalue())

    def test_late_old_marker_cannot_confirm_new_handshake(self):
        controller = Controller()
        controller.drop_first_marker = True
        original_get = controller.get

        def get(url, **kwargs):
            response = original_get(url, **kwargs)
            if url.endswith('/rr_reply') and controller.marker_counts[1] >= 2:
                first_echo = next(c for p, c, _ in controller.commands if c.startswith('echo '))
                response.text = first_echo.split('"')[1]
            return response

        with controller_io(controller), patch('util.execution.requests.get', side_effect=get):
            with self.assertRaisesRegex(TimeoutError, 'acknowledgement missing after retry'):
                recover_printers(controller.ip, controller)
        self.assertEqual(set(controller.moves), {1})

    def test_missing_marker_at_wrong_position_does_not_retry_handshake(self):
        controller = Controller()
        controller.omit_marker = True
        controller.offset = 1.0
        with controller_io(controller), self.assertRaisesRegex(TimeoutError, 'marker=False'):
            recover_printers(controller.ip, controller)
        self.assertEqual(controller.marker_counts[1], 1)
        self.assertEqual(set(controller.moves), {1})

    def test_other_robot_busy_prevents_handshake_retry(self):
        controller = Controller()
        controller.omit_marker = True
        original_status = controller.status

        def status(*args, **kwargs):
            statuses, coordinates = original_status(*args, **kwargs)
            if controller.moves:
                statuses[1] = 'B'
            return statuses, coordinates

        with controller_io(controller), patch('util.execution.read_status', side_effect=status):
            with self.assertRaisesRegex(TimeoutError, "2: 'B'"):
                recover_printers(controller.ip, controller)
        self.assertEqual(controller.marker_counts[1], 1)
        self.assertEqual(set(controller.moves), {1})

    def test_http_timeout_preserves_last_recovery_state(self):
        controller = Controller()
        controller.omit_marker = True
        original_status = controller.status

        def status(*args, **kwargs):
            if controller.now >= 2.0:
                raise requests.ReadTimeout('simulated status timeout')
            return original_status(*args, **kwargs)

        with controller_io(controller), patch('util.execution.read_status', side_effect=status):
            with self.assertRaises(RuntimeError) as raised:
                recover_printers(controller.ip, controller)
        message = str(raised.exception)
        for detail in ('status request failed', 'marker=False', "1: 'I'", 'observed=(275.0, 25.0)',
                       'target=(275.0, 25.0)', 'last_reply='):
            self.assertIn(detail, message)
        self.assertEqual(set(controller.moves), {1})

    def test_worker_forwards_recovery_error_to_scheduler(self):
        controller = Controller()
        pending, results, controls, errors = (queue.Queue() for _ in range(4))
        pending.put(('__recover__', None))
        detail = 'completion acknowledgement missing after retry: marker=False'
        with patch('util.execution.recover_printers', side_effect=TimeoutError(detail)):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                with self.assertRaisesRegex(TimeoutError, 'acknowledgement missing'):
                    execution_worker(controller.ip, pending, results, controls, controller, errors)
        self.assertEqual(results.get_nowait(), ('failed', detail))
        self.assertTrue(controller.is_set())
        self.assertIn('RECOVERY_FAILED', output.getvalue())
        self.assertIn(detail, errors.get_nowait()[1])

    def test_wrong_position_does_not_release_second_robot(self):
        controller = Controller()
        controller.offset = 1.0
        with controller_io(controller), self.assertRaisesRegex(RuntimeError, 'did not reach'):
            recover_printers(controller.ip, controller)
        self.assertEqual(set(controller.moves), {1})

    def test_uncertain_move_is_not_resent(self):
        controller = Controller()
        controller.lose_move_response = True
        with controller_io(controller), self.assertRaises(requests.Timeout):
            recover_printers(controller.ip, controller)
        self.assertEqual([command for _, command, _ in controller.commands if command.startswith('G53 G1 ')],
                         ['G53 G1 X275.000000 Y25.000000 F1500.000000'])

    def test_full_buffer_times_out_without_motion(self):
        controller = Controller()
        controller.full = True
        with controller_io(controller), self.assertRaisesRegex(TimeoutError, 'buffer space'):
            recover_printers(controller.ip, controller)
        self.assertEqual(controller.commands, [])

    def test_cancelled_submission_never_sends(self):
        controller = Controller()
        controller.set()
        with controller_io(controller), self.assertRaisesRegex(RuntimeError, 'cancelled'):
            send_gcode(controller.ip, 1, 'G90', controller, 5.0)
        self.assertEqual(controller.commands, [])

    def test_invalid_buffer_reply_is_rejected(self):
        for data in ({'err': 1}, {'buff': -1}, {'buff': '64'}, {'buff': True}, []):
            with self.subTest(data=data):
                controller = Controller()
                with controller_io(controller), \
                     patch('util.execution.requests.get', return_value=Mock(json=lambda: data)), \
                     self.assertRaisesRegex(RuntimeError, 'invalid rr_gcode'):
                    send_gcode(controller.ip, 1, 'G90', controller, 5.0)


class CallbackQueue(queue.Queue):
    def __init__(self, callback):
        super().__init__()
        self.callback = callback

    def put(self, item, *args, **kwargs):
        self.callback(item)


class SchedulerRecoveryTests(unittest.TestCase):
    def test_final_cell_assigned_to_printer_two_after_recovery(self):
        controller = Controller()
        printer = {
            1: SimpleNamespace(codes={}, prox_join=(0, 0)),
            2: SimpleNamespace(codes={'4.gcode': ['A', [(0, 0)]],
                                     '1.gcode': ['A', [(100, 100)]]}, prox_join=(0, 0)),
        }
        coords = controller.positions.copy()
        footprints = {p: ('initial', p) for p in printer}
        events, results, printing, errors = (queue.Queue(maxsize=2) for _ in range(4))
        events.put((1, 'free'))
        events.put((2, 'free'))
        finished = Mock()
        recovered = False
        submissions = []

        def execute(item):
            nonlocal recovered
            filename, p = item
            submissions.append(item)
            if filename == '__recover__':
                observed = recover_printers(controller.ip, controller)
                recovered = True
                results.put(('recovered', observed))
            else:
                results.put(('sent', p))

        def control(item):
            action, p = item
            if action == 'retry':
                self.assertTrue(recovered)
                self.assertEqual(coords, {1: (275.0, 25.0), 2: (275.0, 25.0)})
                self.assertEqual(footprints, {p: (275.0, 25.0) for p in printer})
                events.put((p, 'free'))
            elif action == 'print':
                printing.put(('Is printing', p))
                events.put((p, 'free'))

        # A stalled scheduler fails this test promptly instead of hanging the suite.
        original_get = events.get

        def next_event(*args, **kwargs):
            try:
                return original_get(timeout=0.05)
            except queue.Empty:
                raise AssertionError('Scheduler stranded a cell with no retry event')

        events.get = next_event
        with tempfile.TemporaryDirectory() as temp, controller_io(controller), \
             patch('util.command_script.sim_px'), \
             patch('util.command_script.simulate_candidate', side_effect=lambda **kw: (kw['candidate'], coords.copy())), \
             patch('util.command_script.polygon_at_position', side_effect=lambda p, printers, position: position), \
             patch('util.command_script.collides_with_stored',
                   side_effect=lambda polygon, p, stored: {1: 5.5} if polygon == '1.gcode' and not recovered else {}):
            command_script(printer, SimpleNamespace(log_dir=Path(temp)), coords, events,
                           CallbackQueue(execute), results, CallbackQueue(control), printing,
                           controller, finished, errors, store_polygons=footprints)
            log = (Path(temp) / 'order.txt').read_text()
        self.assertEqual(submissions, [('4_b.gcode', 2), ('__recover__', None), ('1_m.gcode', 2)])
        self.assertIn('recovered: observed positions=', log)
        self.assertTrue(errors.empty())
        self.assertTrue(controller.is_set())
        finished.set.assert_called_once()


if __name__ == '__main__':
    unittest.main()
