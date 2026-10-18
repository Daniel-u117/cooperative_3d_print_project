"""Read the Cura input used by the preparation pipeline."""
import re
from pathlib import Path

COORDINATE = {axis: re.compile(rf"\b{axis}\s*(-?(?:\d+(?:\.\d*)?|\.\d+))")
              for axis in ("X", "Y")}


def gcode_files(folder):
    folder = Path(folder)
    if not folder.is_dir():
        raise ValueError(f"Input directory does not exist: {folder}")
    files = sorted((p for p in folder.iterdir()
                    if p.is_file() and p.suffix.lower() == ".gcode"),
                   key=lambda p: (int(p.stem) if p.stem.isdigit() else float("inf"), p.name))
    if not files:
        raise ValueError(f"No .gcode files found in {folder}")
    return files


def read_print_points(file_path):
    """Read modal XY motion after M107; reject unsupported input without hanging."""
    started = False
    current = {"X": None, "Y": None}
    points = []
    with open(file_path, encoding="utf-8-sig") as source:
        for raw in source:
            line = raw.split(";", 1)[0].strip()
            if not line:
                continue
            command = line.split()[0]
            if command == "M107":
                started = True
                continue
            if not started or command not in {"G0", "G00", "G1", "G01"}:
                continue
            changed = False
            for axis, pattern in COORDINATE.items():
                match = pattern.search(line)
                if match:
                    current[axis] = float(match.group(1))
                    changed = True
            if changed and all(value is not None for value in current.values()):
                points.append((current["X"], current["Y"]))
    if not started:
        raise ValueError(f"{file_path}: missing M107 marker; expected Cura G-code")
    if len(set(points)) < 3:
        raise ValueError(f"{file_path}: at least three distinct XY points are required")
    return points
