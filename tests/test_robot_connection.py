import contextlib
import io
import os
from pathlib import Path
import pickle
import queue
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import requests

from simulation_main import main
from util.execution import execution_worker
from util.network_utils import RobotConnection, read_status, reconnect_to_ambots


def response(data):
    result = Mock()
    result.json.return_value = data
    return result


class RobotConnectionTests(unittest.TestCase):
    def test_connect_mode_uses_custom_hosts_without_input_upload_or_motion(self):
        calls = []

        def get(url, **kwargs):
            calls.append((url, kwargs))
            if url.endswith('/rr_connect'):
                return response({'err': 0})
            if url.endswith('/rr_status'):
                return response({'status': 'I', 'coords': {'machine': [300, 0, 10]}})
            self.fail(f'Unexpected robot operation: {url}')

        with patch('requests.get', side_effect=get), patch('requests.post') as post, \
             patch('subprocess.run') as subprocess_run, \
             patch.dict(os.environ, {'AMBOTS_PASSWORD': 'test-password'}), \
             contextlib.redirect_stdout(io.StringIO()):
            main(['--mode', 'connect', '--printer-1', 'robot-a', '--printer-2', 'robot-b',
                  '--input-dir', 'does-not-exist'])
        self.assertEqual([url for url, _ in calls], [
            'http://robot-a/rr_connect', 'http://robot-b/rr_connect',
            'http://robot-a/rr_status', 'http://robot-b/rr_status'])
        self.assertEqual(calls[0][1]['params']['password'], 'test-password')
        post.assert_not_called()
        subprocess_run.assert_not_called()

    def test_connection_failure_does_not_change_wifi_without_profile(self):
        with patch('requests.get', side_effect=requests.ConnectionError), \
             patch('subprocess.run') as run:
            with self.assertRaises(ConnectionError):
                reconnect_to_ambots(connection=RobotConnection())
        run.assert_not_called()

    def test_explicit_wifi_profile_used_on_failed_connection(self):
        config = RobotConnection(wifi_profile='Lab robots', wifi_interface='Wireless')
        with patch('util.network_utils.connect_robots', side_effect=[ConnectionError(), None]), \
             patch('util.network_utils.run_netsh', return_value=Mock(returncode=0)) as netsh:
            self.assertTrue(reconnect_to_ambots(connection=config))
        netsh.assert_called_once_with('connect', 'name=Lab robots', 'interface=Wireless')

    def test_status_requires_native_machine_coordinates(self):
        with patch('requests.get', return_value=response({'status': 'I', 'coords': {}})):
            with self.assertRaisesRegex(ValueError, 'coords.machine'):
                read_status(RobotConnection())

    def test_invalid_or_duplicate_addresses_rejected(self):
        for hosts in [('same-host', 'same-host'), ('http://robot-a', 'robot-b')]:
            with self.assertRaises(ValueError):
                RobotConnection(*hosts)

    def test_live_mode_stops_before_upload_on_connection_rejection(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch('requests.get', return_value=response({'err': 1})) as get, \
             patch('requests.post') as post, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ConnectionError, 'rejected'):
                main(['--mode', 'printers', '--output-dir', str(Path(temp) / 'output')])
        post.assert_not_called()
        self.assertTrue(all(call.args[0].endswith('/rr_connect') for call in get.call_args_list))

    def test_failed_upload_prevents_homing(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch('simulation_main.check_robot_connection', return_value=(['I', 'I'], [[300, 0], [300, 0]])), \
             patch('requests.get') as get, patch('requests.post', return_value=response({'err': 1})), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, 'Upload failed'):
                main(['--mode', 'printers', '--output-dir', str(Path(temp) / 'output')])
        get.assert_not_called()

    def test_worker_receives_serializable_connection_settings(self):
        connection = pickle.loads(pickle.dumps(RobotConnection('robot-a', 'robot-b', 'test-secret')))
        self.assertNotIn('test-secret', repr(connection))
        stop = threading.Event()
        requests_queue, result_queue, control_queue, errors = (queue.Queue() for _ in range(4))
        requests_queue.put(('1_b.gcode', 2))

        def send(url, **kwargs):
            if kwargs['params']['gcode']:
                stop.set()
            return response({'buff': 100})

        with patch('util.execution.get_status', return_value=(['I', 'I'], [[300, 0], [300, 0]])) as status, \
             patch('requests.get', side_effect=send) as get, contextlib.redirect_stdout(io.StringIO()):
            execution_worker(connection.ip, requests_queue, result_queue, control_queue,
                             stop, errors, connection=connection)
        status.assert_called_once_with(connection=connection)
        self.assertEqual(get.call_args.args[0], 'http://robot-b/rr_gcode')
        self.assertEqual(result_queue.get_nowait(), ('sent', 2))
        self.assertTrue(errors.empty())


if __name__ == '__main__':
    unittest.main()
