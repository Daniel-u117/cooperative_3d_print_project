import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import requests

from simulation_main import main
from util.homing import home_robots
from util.network_utils import RobotConnection


class Firmware:
    """Fake clock/controller: G-code acceptance precedes motion completion."""
    def __init__(self, durations=(60, 90)):
        self.now = 0.0
        self.durations = dict(zip(('robot-a', 'robot-b'), durations))
        self.started = {}
        self.markers = {}
        self.submissions = []
        self.status_reads = []
        self.omit_marker = False
        self.always_idle = False
        self.early_marker = False
        self.firmware_error = False
        self.reject = False
        self.disconnect = False

    def sleep(self, seconds):
        self.now += seconds

    def get(self, url, **kwargs):
        host, endpoint = url.removeprefix('http://').split('/')
        response = Mock()
        response.text = ''
        if endpoint == 'rr_gcode':
            command = kwargs['params']['gcode']
            assert command.startswith('G28\nM400\necho "HOMING_')
            self.started[host] = self.now
            self.markers[host] = command.split('"')[1]
            self.submissions.append((host, self.now))
            response.json.return_value = {'err': 1} if self.reject else {'buff': 100}
            return response
        elapsed = self.now - self.started.get(host, self.now)
        completed = host in self.started and elapsed >= self.durations[host]
        if endpoint == 'rr_reply':
            if self.disconnect:
                raise requests.Timeout('Simulated connection failure')
            if self.firmware_error:
                response.text = 'Error: homing failed'
            elif (completed or self.early_marker) and not self.omit_marker:
                response.text = self.markers[host] + '\n'
            else:
                # A marker from a previous run must never count as completion.
                response.text = 'HOMING_old_run\n'
            return response
        assert endpoint == 'rr_status', endpoint
        self.status_reads.append(host)
        # Model a brief initial idle period while the command is still queued.
        idle = self.always_idle or host not in self.started or elapsed < 2 or completed
        response.json.return_value = {
            'status': 'I' if idle else 'B', 'coords': {'machine': [300, 0, 10]},
        }
        return response


class HomingTests(unittest.TestCase):
    def run_firmware(self, firmware, timeout=180):
        with patch('util.homing.time.monotonic', side_effect=lambda: firmware.now), \
             patch('util.homing.time.sleep', side_effect=firmware.sleep), \
             patch('requests.get', side_effect=firmware.get), \
             contextlib.redirect_stdout(io.StringIO()):
            home_robots(RobotConnection('robot-a', 'robot-b'), timeout=timeout, poll_interval=1)

    def test_long_homing_waits_for_each_robot_and_keeps_both_sessions_active(self):
        firmware = Firmware((60, 90))
        self.run_firmware(firmware, timeout=100)
        self.assertEqual(firmware.submissions, [('robot-a', 0), ('robot-b', 60)])
        self.assertEqual(firmware.now, 150)
        self.assertEqual(firmware.status_reads.count('robot-a'), firmware.status_reads.count('robot-b'))

    def test_marker_alone_does_not_finish_a_busy_robot(self):
        firmware = Firmware((5, 7))
        firmware.early_marker = True
        # Skip the simulated initial idle period in this scenario.
        original_get = firmware.get

        def get(url, **kwargs):
            result = original_get(url, **kwargs)
            host = url.removeprefix('http://').split('/')[0]
            if url.endswith('/rr_status') and host in firmware.started:
                elapsed = firmware.now - firmware.started[host]
                if elapsed < firmware.durations[host]:
                    result.json.return_value['status'] = 'B'
            return result

        firmware.get = get
        self.run_firmware(firmware)
        self.assertEqual(firmware.submissions, [('robot-a', 0), ('robot-b', 5)])
        self.assertEqual(firmware.now, 12)

    def test_idle_without_this_runs_marker_times_out_without_starting_second_robot(self):
        firmware = Firmware()
        firmware.always_idle = True
        firmware.omit_marker = True
        with self.assertRaisesRegex(TimeoutError, 'robot 1.*completion marker received=False'):
            self.run_firmware(firmware, timeout=5)
        self.assertEqual(firmware.submissions, [('robot-a', 0)])

    def test_second_robot_gets_its_own_timeout(self):
        firmware = Firmware((3, 60))
        with self.assertRaisesRegex(TimeoutError, 'robot 2.*last status=B'):
            self.run_firmware(firmware, timeout=5)
        self.assertEqual(firmware.submissions, [('robot-a', 0), ('robot-b', 3)])
        self.assertEqual(firmware.now, 8)

    def test_errors_abort_without_resubmission(self):
        for option, exception in [('firmware_error', RuntimeError), ('reject', RuntimeError),
                                  ('disconnect', ConnectionError)]:
            with self.subTest(option=option):
                firmware = Firmware()
                setattr(firmware, option, True)
                with self.assertRaises(exception):
                    self.run_firmware(firmware)
                self.assertEqual(firmware.submissions, [('robot-a', 0)])

    def test_invalid_timeouts_fail_before_network_access(self):
        with patch('requests.get') as get:
            for timeout in (0, -1, float('nan'), float('inf')):
                with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                    home_robots(RobotConnection(), timeout=timeout)
            get.assert_not_called()

    def test_live_startup_stops_before_workers_on_homing_timeout(self):
        response = Mock()
        response.json.return_value = {'err': 0}
        with tempfile.TemporaryDirectory() as temp, \
             patch('simulation_main.check_robot_connection', return_value=(['I', 'I'], [[300, 0], [300, 0]])), \
             patch('requests.post', return_value=response), \
             patch('requests.get', side_effect=AssertionError('Unexpected network request')), \
             patch('simulation_main.home_robots', side_effect=TimeoutError('Homing timeout')) as home, \
             patch('simulation_main.mp.get_context') as workers, \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(TimeoutError, 'Homing timeout'):
                main(['--mode', 'printers', '--homing-timeout', '300',
                      '--output-dir', str(Path(temp) / 'output')])
            self.assertEqual(home.call_args.kwargs['timeout'], 300)
            workers.assert_not_called()


if __name__ == '__main__':
    unittest.main()
