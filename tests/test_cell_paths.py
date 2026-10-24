import contextlib
from dataclasses import replace
import io
import math
import os
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch

from project_paths import CELL_PATH_DIR, INPUT_DIR, CELL_PRINT_CONFIG
from simulation_main import main
from util.cell_paths import CellPrintSettings, convert_cell_path, read_cell_points, load_cell_settings
from util.gcode_input import cell_input_files, read_print_points


class CellPathTests(unittest.TestCase):
    def test_configuration_is_loaded_from_project_and_reread_after_edits(self):
        previous = Path.cwd()
        with tempfile.TemporaryDirectory() as temp:
            try:
                os.chdir(temp)
                self.assertEqual(load_cell_settings(), CellPrintSettings())
                config = Path(temp) / 'profile.toml'
                config.write_text(CELL_PRINT_CONFIG.read_text().replace('240.0', '215.0'))
                with patch('util.cell_paths.CELL_PRINT_CONFIG', config):
                    source, destination = Path(temp) / '01.txt', Path(temp) / '01.gcode'
                    source.write_text('0 0\n3 0\n3 4\n')
                    convert_cell_path(source, destination)
                    self.assertIn('M109 S215', destination.read_text())
                    config.write_text(config.read_text().replace('215.0', '225.0'))
                    convert_cell_path(source, destination)
                    self.assertIn('M109 S225', destination.read_text())
            finally:
                os.chdir(previous)

    def test_invalid_configurations_are_rejected(self):
        original = CELL_PRINT_CONFIG.read_text()
        variants = [original.replace('20.0', '"fast"'),
                    original.replace('20.0', 'true'),
                    original.replace('20.0', '0'),
                    original.replace('20.0', 'nan'),
                    original.replace('print_speed', 'print_speeed'),
                    original.replace('print_speed = 20.0', ''),
                    original + '\nextra_setting = 1\n',
                    original + '\n[extra]\nvalue = 1\n',
                    '[cell_print', 'cell_print = 5']
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / 'invalid.toml'
            with self.assertRaisesRegex(ValueError, 'Cannot read cell-print configuration'):
                load_cell_settings(config)
            for text in variants:
                with self.subTest(text=text):
                    config.write_text(text)
                    with self.assertRaises(ValueError) as raised:
                        load_cell_settings(config)
                    self.assertIn(str(config), str(raised.exception))

    def test_profile_applies_to_preparation_and_cli_override_takes_precedence(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / 'profile.toml'
            config.write_text(CELL_PRINT_CONFIG.read_text().replace('240.0', '215.0')
                              .replace('20.0', '12.0').replace('0.44', '0.5'))
            output = Path(temp) / 'out'
            with patch('requests.sessions.Session.request', side_effect=AssertionError('Network')), \
                 contextlib.redirect_stdout(io.StringIO()):
                main(['--mode', 'prepare', '--input-dir', str(CELL_PATH_DIR), '--cells', '01',
                      '--cell-config', str(config), '--output-dir', str(output),
                      '--txt-nozzle-temperature', '220'])
            text = (output / 'processed' / 'p1' / '01_m.gcode').read_text()
            self.assertIn('M109 S220', text)
            self.assertIn('F720', text)
            self.assertIn('line width: 0.5', (output / 'converted' / '01.gcode').read_text())
            self.assertIn('215.0', config.read_text())
            bad_output = Path(temp) / 'bad-output'
            config.write_text('[cell_print]\nprint_speed = "fast"')
            with patch('requests.sessions.Session.request', side_effect=AssertionError('Network')):
                with self.assertRaises(ValueError):
                    main(['--mode', 'printers', '--input-dir', str(CELL_PATH_DIR), '--cells', '01',
                          '--cell-config', str(config), '--output-dir', str(bad_output)])
            self.assertFalse(bad_output.exists())

    def test_order_extrusion_and_no_implicit_closure(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / '01.txt'
            original = '\ufeff# ordered path\n0 0\n3 0 ; next corner\n\n3 4\n1 2\n'
            source.write_text(original, encoding='utf-8')
            target = Path(temp) / 'out' / '01.gcode'
            settings = CellPrintSettings(filament_diameter=1.75, line_width=0.4,
                                         layer_height=0.2, flow_multiplier=1.1,
                                         nozzle_temperature=215, print_speed=15)
            convert_cell_path(source, target, settings)
            self.assertEqual(source.read_text(encoding='utf-8'), original)
            self.assertEqual(read_print_points(target), [(0, 0), (3, 0), (3, 4), (1, 2)])
            text = target.read_text()
            moves = [line for line in text.splitlines() if line.startswith('G1 X')]
            self.assertEqual(len(moves), 3)
            expected = (3 + 4 + math.sqrt(8)) * .4 * .2 * 1.1 / (math.pi * (.875 ** 2))
            actual = float(re.search(r'E([\d.]+)', moves[-1]).group(1))
            self.assertAlmostEqual(actual, expected, places=7)
            self.assertIn('M109 S215', text)
            self.assertTrue(all('F900' in line for line in moves))
            self.assertNotIn('G28', text)
            self.assertNotIn('M84', text)
            self.assertLess(text.index('G0 Z10'), text.index('G0 X0.00000 Y0.00000'))
            self.assertLess(text.index('G0 X0.00000 Y0.00000'), text.index('G0 Z0.2'))

    def test_invalid_paths_have_source_and_line_diagnostics(self):
        cases = [('0 0\n1 2 3\n2 0', ':2: expected two finite numbers'),
                 ('0 0\nnan 2\n2 0', ':2: expected two finite numbers'),
                 ('0 0\ninf 2\n2 0', ':2: expected two finite numbers'),
                 ('0,0\n1,1\n2,0', ':1: expected two finite numbers'),
                 ('0 0\n0 0', 'three distinct'),
                 ('0 0\n1 1\n2 2', 'nonzero area')]
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'bad.txt'
            for content, error in cases:
                with self.subTest(content=content):
                    source.write_text(content)
                    with self.assertRaisesRegex(ValueError, error):
                        read_cell_points(source)

    def test_invalid_settings(self):
        for field in ('filament_diameter', 'line_width', 'layer_height', 'nozzle_temperature',
                      'print_speed', 'travel_speed', 'flow_multiplier'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    replace(CellPrintSettings(), **{field: value})
        with self.assertRaisesRegex(ValueError, 'below travel_height'):
            CellPrintSettings(layer_height=10)

    def test_duplicate_stems_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            for name in ('01.txt', '01.gcode'):
                (Path(temp) / name).touch()
            with self.assertRaisesRegex(ValueError, 'Duplicate cell stem'):
                cell_input_files(temp)

    def test_bad_later_source_leaves_no_output_and_never_connects(self):
        with tempfile.TemporaryDirectory() as temp:
            folder, output = Path(temp) / 'input', Path(temp) / 'out'
            folder.mkdir()
            (folder / '01.txt').write_text('0 0\n3 0\n3 4\n')
            (folder / '02.txt').write_text('invalid')
            with patch('requests.sessions.Session.request', side_effect=AssertionError('Network')):
                with self.assertRaisesRegex(ValueError, '02.txt:1'):
                    main(['--mode', 'printers', '--input-dir', str(folder), '--output-dir', str(output)])
            self.assertFalse(output.exists())

    def test_all_supplied_paths_prepare_into_independent_printer_jobs(self):
        before = {p.name: p.read_bytes() for p in CELL_PATH_DIR.glob('*.txt')}
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)
            with patch('requests.sessions.Session.request', side_effect=AssertionError('Network')), \
                 contextlib.redirect_stdout(io.StringIO()):
                main(['--mode', 'prepare', '--input-dir', str(CELL_PATH_DIR), '--output-dir', temp])
            self.assertEqual(len(list((output / 'converted').glob('*.gcode'))), len(before))
            self.assertEqual(len(list((output / 'processed').glob('*/*.gcode'))), 6 * len(before))
            source_points = read_cell_points(CELL_PATH_DIR / '01.txt')
            self.assertEqual(read_print_points(output / 'converted' / '01.gcode'), source_points)
            for p in (1, 2):
                transformed = read_print_points(output / 'transformed' / f'p{p}' / '01.gcode')
                # Existing layout-to-machine transforms: p1=(y,300-x); p2=(300-y,x).
                for (x, y), actual in zip(source_points, transformed):
                    expected = (y, 300 - x) if p == 1 else (300 - y, x)
                    self.assertAlmostEqual(actual[0], expected[0])
                    self.assertAlmostEqual(actual[1], expected[1])
                for stage in ('b', 'm', 'e'):
                    job = output / 'processed' / f'p{p}' / f'01_{stage}.gcode'
                    text = job.read_text()
                    for command in ('G21', 'G90', 'M82', 'G92 E0', 'M109 S240', 'M107'):
                        self.assertIn(command, text)
                    self.assertEqual(read_print_points(job), transformed)
                    if stage in ('b', 'm'):
                        self.assertIn('G1 Z10\n\nM400\nset global.ambotJobDone', text)
                        self.assertNotIn('M104 S0', text)
            self.assertEqual(before, {p.name: p.read_bytes() for p in CELL_PATH_DIR.glob('*.txt')})

    def test_mixed_inputs_and_selected_initial_cell_names(self):
        with tempfile.TemporaryDirectory() as temp:
            folder, output = Path(temp) / 'input', Path(temp) / 'out'
            folder.mkdir()
            (folder / '01.txt').write_bytes((CELL_PATH_DIR / '01.txt').read_bytes())
            (folder / '4.gcode').write_bytes((INPUT_DIR / '4.gcode').read_bytes())
            (folder / 'unused.txt').write_text('invalid unselected input')
            with patch('requests.sessions.Session.request', side_effect=AssertionError('Network')), \
                 patch('util.offline_simulation.run_offline') as run, \
                 contextlib.redirect_stdout(io.StringIO()):
                main(['--input-dir', str(folder), '--output-dir', str(output), '--headless',
                      '--cells', '01', '4', '--initial-cells', '01', '4',
                      '--txt-nozzle-temperature', '210', '--txt-print-speed', '12'])
            self.assertEqual(run.call_args.kwargs['initial_assignments'], {1: '01.gcode', 2: '4.gcode'})
            self.assertEqual({p.name for p in (output / 'converted').iterdir()}, {'01.gcode'})
            self.assertIn('M109 S210', (output / 'processed' / 'p1' / '01_m.gcode').read_text())
            self.assertEqual({p.name for p in (output / 'transformed' / 'p1').iterdir()},
                             {'01.gcode', '4.gcode'})


if __name__ == '__main__':
    unittest.main()
