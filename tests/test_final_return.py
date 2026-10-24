import contextlib
import io
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from util.command_script import command_script
from test_recovery import CallbackQueue, Controller


class FinalReturnTests(unittest.TestCase):
    def test_waits_for_both_job_completions_before_return_and_shutdown(self):
        controller = Controller()
        printer = {p: SimpleNamespace(codes={f'{p}.gcode': ['A', [(p, p)]]},
                                      prox_join=(0, 0)) for p in (1, 2)}
        coords = controller.positions.copy()
        footprints = {p: None for p in printer}
        events, results, printing, errors = (queue.Queue() for _ in range(4))
        events.put((1, 'free'))
        events.put((2, 'free'))
        active, completed, submissions = set(), set(), []
        finished = Mock()
        parked = {1: (275.0, 25.0), 2: (270.0, 30.0)}

        def execute(item):
            filename, p = item
            submissions.append(item)
            if filename == '__park__':
                self.assertEqual(completed, {1, 2})
                self.assertFalse(controller.is_set())
                finished.set.assert_not_called()
                results.put(('parked', parked))
            else:
                active.add(p)
                results.put(('sent', p))

        def control(item):
            action, p = item
            if action == 'print':
                printing.put(('Is printing', p))
                events.put((p, 'free'))

        original_get = events.get

        def event(*args, **kwargs):
            item = original_get(timeout=0.05)
            if item[0] in active:
                completed.add(item[0])
            return item

        events.get = event
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()), \
             patch('util.command_script.sim_px'), \
             patch('util.command_script.simulate_candidate', return_value=(None, coords.copy())), \
             patch('util.command_script.collides_with_stored', return_value={}), \
             patch('util.command_script.polygon_at_position', side_effect=lambda p, printers, xy: xy):
            command_script(printer, SimpleNamespace(log_dir=Path(temp)), coords, events,
                           CallbackQueue(execute), results, CallbackQueue(control), printing,
                           controller, finished, errors, store_polygons=footprints)
            log = (Path(temp) / 'order.txt').read_text()
        self.assertEqual(submissions, [('1_b.gcode', 1), ('2_b.gcode', 2), ('__park__', None)])
        self.assertEqual(coords, parked)
        self.assertEqual(footprints, parked)
        self.assertIn('final return complete:', log)
        self.assertTrue(controller.is_set())
        self.assertTrue(errors.empty())
        finished.set.assert_called_once()

    def test_failed_incomplete_or_timed_out_return_is_not_logged_as_success(self):
        for reply in [('failed', 'target not reached'), ('parked', {1: (275, 25)}), None]:
            with self.subTest(reply=reply):
                controller = Controller()
                printer = {p: SimpleNamespace(codes={}) for p in (1, 2)}
                coords = controller.positions.copy()
                errors = queue.Queue()
                results = Mock()

                def receive(**kwargs):
                    if reply is not None:
                        return reply
                    controller.now += 1000
                    raise queue.Empty

                results.get.side_effect = receive
                with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()), \
                     patch('util.command_script.time.monotonic', side_effect=lambda: controller.now):
                    with self.assertRaises((RuntimeError, TimeoutError)):
                        command_script(printer, SimpleNamespace(log_dir=Path(temp)), coords,
                                       queue.Queue(), Mock(), results, queue.Queue(), queue.Queue(),
                                       controller, Mock(), errors, store_polygons={1: None, 2: None})
                    log = (Path(temp) / 'order.txt').read_text()
                self.assertNotIn('final return complete:', log)
                self.assertIn('scheduler failed:', log)
                self.assertFalse(errors.empty())
                self.assertEqual(coords, controller.positions)
                self.assertTrue(controller.is_set())


if __name__ == '__main__':
    unittest.main()
