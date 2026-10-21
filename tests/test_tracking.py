import contextlib
import io
import queue
import unittest
from unittest.mock import Mock, patch

from util.tracking import track


class ClockStop:
    def __init__(self):
        self.now = 0.0
        self.stopped = False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, seconds):
        self.now += seconds


class TrackingTests(unittest.TestCase):
    def run_tracker(self, moving):
        stop = ClockStop()
        errors = queue.Queue()
        controls = queue.Queue()
        controls.put(('hold', 1))
        controls.put(('print', 1))
        printing = Mock()
        def acknowledge(item):
            self.assertEqual(coords[1], (103.0, 100.0))
            self.assertEqual(footprints[1], coords[1])
            stop.set()
        printing.put.side_effect = acknowledge
        samples = 0

        def model(*args, **kwargs):
            nonlocal samples
            samples += 1
            return Mock(json=lambda: {'result': {
                'state': {'status': 'processing'},
                'move': {'axes': [{'machinePosition': 100 + samples if moving else 100},
                                  {'machinePosition': 100}],
                         'extruders': [{'position': samples}]},
            }})

        coords, footprints = {}, {}
        with patch('util.tracking.get_status', return_value=(['I', 'I'], [(300, 0), (300, 0)])), \
             patch('util.tracking.requests.get', side_effect=model), \
             patch('util.tracking.time.monotonic', side_effect=lambda: stop.now), \
             patch('util.tracking.time.sleep'), patch('util.tracking.WATCH_TIMEOUT', 0.5), \
             contextlib.redirect_stdout(io.StringIO()):
            args = ({1: 'robot-a', 2: 'robot-b'}, coords, queue.Queue(maxsize=1), controls,
                    printing, stop, errors, queue.Queue(maxsize=1))
            kwargs = dict(printer={1: None, 2: None}, store_polygons=footprints,
                          polygon_at_position=lambda p, printers, position: position)
            if moving:
                track(*args, **kwargs)
            else:
                with self.assertRaisesRegex(TimeoutError, 'no moving extrusion confirmed'):
                    track(*args, **kwargs)
        return printing, errors, coords, footprints

    def test_stationary_priming_times_out_without_print_acknowledgement(self):
        printing, errors, _, _ = self.run_tracker(moving=False)
        printing.put.assert_not_called()
        worker, details = errors.get_nowait()
        self.assertEqual(worker, 'track')
        self.assertIn('Job may have finished or failed', details)

    def test_moving_extrusion_publishes_observed_pose_before_acknowledgement(self):
        printing, errors, coords, footprints = self.run_tracker(moving=True)
        printing.put.assert_called_once_with(('Is printing', 1))
        self.assertEqual(coords[1], (103.0, 100.0))
        self.assertEqual(footprints[1], coords[1])
        self.assertTrue(errors.empty())


if __name__ == '__main__':
    unittest.main()
