import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from project_paths import INPUT_DIR
from simulation_main import main
from util.gcode_input import gcode_files, read_print_points


class LocalRunTests(unittest.TestCase):
    def test_missing_marker_fails_instead_of_hanging(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "broken.gcode"
            source.write_text("G1 X0 Y0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing M107"):
                read_print_points(source)

    def test_blank_lines_comments_and_modal_coordinates(self):
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "sample.gcode"
            source.write_text("M107 ; fan off\nG0 X1 Y2\n\nG1 X3 ; Y999\nG1 Y4\n", encoding="utf-8")
            self.assertEqual(read_print_points(source), [(1, 2), (3, 2), (3, 4)])

    def test_missing_and_empty_input(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(ValueError, "No .gcode"):
                gcode_files(temp)
            with self.assertRaisesRegex(ValueError, "does not exist"):
                gcode_files(Path(temp) / "missing")

    def test_offline_pipeline_from_another_directory_without_network(self):
        with tempfile.TemporaryDirectory() as temp:
            previous = Path.cwd()
            source_bytes = {p.name: p.read_bytes() for p in gcode_files(INPUT_DIR)}
            output = Path(temp) / "generated output"
            # A stale file must not be interpreted as a current input region.
            stale = output / "boundaries_store" / "p1" / "stale.gcode"
            stale.parent.mkdir(parents=True)
            stale.write_text("invalid", encoding="utf-8")
            try:
                os.chdir(temp)
                with patch("requests.sessions.Session.request", side_effect=AssertionError("Network access")), \
                     patch("subprocess.run", side_effect=AssertionError("External command")), \
                     contextlib.redirect_stdout(io.StringIO()):
                    report = main(["--headless", "--output-dir", str(output)])
            finally:
                os.chdir(previous)
            accepted = [item["gcode"] for item in report["accepted"]]
            remaining = report["remaining_reachable_regions"]
            self.assertTrue(accepted)
            self.assertEqual(len(accepted), len(set(accepted)))
            self.assertFalse(set(accepted) & set(remaining))
            self.assertEqual(set(accepted) | set(remaining), set(source_bytes))
            self.assertEqual(len(list((output / "processed").glob("*/*.gcode"))), 6 * len(source_bytes))
            self.assertEqual(json.loads((output / "simulation_report.json").read_text())["mode"], "offline")
            self.assertEqual(source_bytes, {p.name: p.read_bytes() for p in gcode_files(INPUT_DIR)})


if __name__ == "__main__":
    unittest.main()
