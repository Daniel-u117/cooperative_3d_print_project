import contextlib
import io
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from simulation_main import main
from util.command_script import command_script, polygon_at_position
from util.execution import recover_printers
from util.gcode_input import read_print_points
from util.initial_assignments import validate_initial_assignments
from test_recovery import CallbackQueue, Controller, controller_io


class InitialAssignmentTests(unittest.TestCase):
    def test_seeded_preview_assigns_four_and_seven_then_checks_one(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch('requests.sessions.Session.request', side_effect=AssertionError('Network access')), \
             contextlib.redirect_stdout(io.StringIO()):
            report = main(['--headless', '--cells', '1', '4', '7', '--initial-cells', '4', '7',
                           '--output-dir', temp])
        self.assertEqual(report['accepted'], [{'printer': 1, 'gcode': '4.gcode'},
                                               {'printer': 2, 'gcode': '7.gcode'}])
        self.assertEqual(report['remaining_reachable_regions'], ['1.gcode'])
        self.assertEqual([(a['printer'], a['gcode'], a['accepted']) for a in report['attempts'][2:]],
                         [(1, '1.gcode', False), (2, '1.gcode', False)])

    def test_duplicate_or_unselected_initial_cells_fail_before_output(self):
        for initial in (['4', '4'], ['4', 'missing'], ['4', '2']):
            with self.subTest(initial=initial), tempfile.TemporaryDirectory() as temp, \
                 patch('requests.sessions.Session.request', side_effect=AssertionError('Network access')), \
                 contextlib.redirect_stderr(io.StringIO()):
                output = Path(temp) / 'output'
                with self.assertRaises(SystemExit):
                    main(['--mode', 'printers', '--cells', '1', '4', '7', '--initial-cells', *initial,
                          '--output-dir', str(output)])
                self.assertFalse(output.exists())

    def test_unreachable_initial_assignment_rejected(self):
        with self.assertRaisesRegex(ValueError, 'not reachable'):
            validate_initial_assignments({1: SimpleNamespace(codes={})}, {1: '4.gcode'})

    def test_initial_collision_stops_instead_of_substituting_another_cell(self):
        printer = {1: SimpleNamespace(codes={'4.gcode': ['A', [(0, 0)]],
                                             '1.gcode': ['A', [(1, 1)]]})}
        events, results, controls, printing, errors = (queue.Queue() for _ in range(5))
        events.put((1, 'free'))
        controller = Controller()
        execution = Mock()
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()), \
             patch('util.command_script.sim_px'), \
             patch('util.command_script.polygon_at_position'), \
             patch('util.command_script.simulate_candidate', return_value=(None, {1: (300, 0)})), \
             patch('util.command_script.collides_with_stored', return_value={2: 1.0}):
            with self.assertRaisesRegex(RuntimeError, 'Initial assignment p1:4.gcode failed collision'):
                command_script(printer, SimpleNamespace(log_dir=Path(temp)), {1: (300, 0)}, events,
                               execution, results, controls, printing, controller, Mock(), errors,
                               store_polygons={1: None}, initial_assignments={1: '4.gcode'})
        execution.put.assert_not_called()
        self.assertEqual(printer[1].codes['4.gcode'][0], 'A')

    def test_real_geometry_final_cell_after_both_initial_jobs_and_recovery(self):
        for drop_marker, completed_early in ((False, False), (True, False), (True, True)):
            with self.subTest(drop_marker=drop_marker, completed_early=completed_early), tempfile.TemporaryDirectory() as temp:
                self.exercise_recovery(Path(temp), drop_marker=drop_marker, completed_early=completed_early)

    def exercise_recovery(self, output, *, drop_marker=False, completed_early=False):
        captured = {}

        def capture(printer, config, **kwargs):
            captured.update(printer=printer, config=config, initial=kwargs['initial_assignments'])

        with patch('util.offline_simulation.run_offline', side_effect=capture), \
             contextlib.redirect_stdout(io.StringIO()):
            main(['--headless', '--cells', '1', '4', '7', '--initial-cells', '4', '7',
                  '--output-dir', str(output)])
        printer, config = captured['printer'], captured['config']
        job_tokens = {p: {path.name: path.read_text().splitlines()[-1].split('"')[1]
                          for path in (output / 'processed' / f'p{p}').glob('*.gcode')}
                      for p in printer}
        controller = Controller()
        controller.drop_first_marker = drop_marker
        controller.positions = {p: (300.0, 0.0) for p in printer}
        coords = controller.positions.copy()
        footprints = {p: polygon_at_position(p, printer, coords[p]) for p in printer}
        events, results, printing, errors = (queue.Queue() for _ in range(4))
        events.put((1, 'free'))
        events.put((2, 'free'))
        submissions, completed = [], set()
        active = {}
        finished = Mock()

        def execute(item):
            filename, p = item
            submissions.append(item)
            if filename == '__recover__':
                self.assertEqual(completed, {'4', '7'})
                results.put(('recovered', recover_printers(controller.ip, controller)))
            elif filename == '__park__':
                self.assertEqual(completed, {'1', '4', '7'})
                self.assertFalse(controller.is_set())
                finished.set.assert_not_called()
                results.put(('parked', recover_printers(controller.ip, controller)))
            else:
                stem = filename.split('_')[0]
                if stem == '1':
                    self.assertEqual(completed, {'4', '7'})
                    self.assertIn(('__recover__', None), submissions)
                active[p] = stem
                results.put(('sent', p))

        def control(item):
            action, p, *details = item
            if action == 'retry':
                events.put((p, 'free'))
            elif action == 'print':
                self.assertEqual(details[0]['token'], job_tokens[p][details[0]['file']])
                stem = active[p]
                # Model final telemetry from the G-code endpoint, not the sampled boundary.
                coords[p] = read_print_points(output / 'transformed' / f'p{p}' / f'{stem}.gcode')[-1]
                controller.positions[p] = coords[p]
                footprints[p] = polygon_at_position(p, printer, coords[p])
                printing.put(('Completed' if completed_early else 'Is printing', p))
                events.put((p, 'free'))

        original_get = events.get

        def next_event(*args, **kwargs):
            try:
                item = original_get(timeout=0.05)
                if item[0] in active:
                    completed.add(active[item[0]])
                return item
            except queue.Empty:
                raise AssertionError('Scheduler stalled without a pending event')

        events.get = next_event
        with controller_io(controller):
            command_script(printer, config, coords, events, CallbackQueue(execute), results,
                           CallbackQueue(control), printing, controller, finished, errors,
                           store_polygons=footprints, initial_assignments=captured['initial'], job_tokens=job_tokens)
        log = (output / 'order.txt').read_text()
        self.assertEqual(submissions, [('4_b.gcode', 1), ('7_b.gcode', 2),
                                       ('__recover__', None), ('1_m.gcode', 1), ('__park__', None)])
        self.assertLess(log.index('initial assignments complete'), log.index('candidate p1:1.gcode'))
        self.assertIn('reject p1:1.gcode', log)
        self.assertIn('reject p2:1.gcode', log)
        self.assertIn('recovered: observed positions=', log)
        self.assertIn('final return complete: observed positions=', log)
        self.assertTrue(errors.empty())
        self.assertTrue(controller.is_set())
        finished.set.assert_called_once()
        return log


if __name__ == '__main__':
    unittest.main()
