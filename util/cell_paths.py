"""Convert ordered XY cell paths (millimetres) to single-layer G-code."""
from dataclasses import dataclass, fields
import math
import tomllib
from pathlib import Path

from shapely.geometry import MultiPoint

from project_paths import CELL_PRINT_CONFIG


@dataclass(frozen=True)
class CellPrintSettings:
    filament_diameter: float = 2.85
    line_width: float = 0.44
    layer_height: float = 0.2
    nozzle_temperature: float = 240.0
    print_speed: float = 20.0  # mm/s; G-code feed rates are mm/min
    travel_speed: float = 60.0
    flow_multiplier: float = 1.0
    travel_height: float = 10.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"Text conversion {field.name} must be finite and positive")
        if self.layer_height >= self.travel_height:
            raise ValueError("Text conversion layer_height must be below travel_height")
        # Existing beginning/middle variants finish by lifting to Z10.
        if self.travel_height != 10.0:
            raise ValueError("Text conversion travel_height must be 10 mm for the current pipeline")


def load_cell_settings(config_path=None):
    """Read a complete, strictly validated profile on every invocation."""
    path = Path(config_path) if config_path is not None else CELL_PRINT_CONFIG
    try:
        with path.open("rb") as source:
            document = tomllib.load(source)
    except (OSError, ValueError) as exc:
        raise ValueError(f"Cannot read cell-print configuration {path}: {exc}") from exc
    if set(document) != {"cell_print"} or not isinstance(document["cell_print"], dict):
        raise ValueError(f"{path}: expected only a [cell_print] table")
    values = document["cell_print"]
    expected = {field.name for field in fields(CellPrintSettings)} - {"travel_height"}
    missing, unknown = expected - values.keys(), values.keys() - expected
    if missing or unknown:
        raise ValueError(f"{path}: invalid cell_print keys; missing={sorted(missing)}, unknown={sorted(unknown)}")
    try:
        return CellPrintSettings(**values)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def read_cell_points(file_path):
    """Read exactly two finite numbers per row; blanks and #/; comments are allowed."""
    file_path = Path(file_path)
    points = []
    for number, raw in enumerate(file_path.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = raw.split("#", 1)[0].split(";", 1)[0].strip()
        if not line:
            continue
        try:
            values = line.split()
            if len(values) != 2:
                raise ValueError
            point = tuple(float(value) for value in values)
            if not all(math.isfinite(value) for value in point):
                raise ValueError
        except ValueError:
            raise ValueError(f"{file_path}:{number}: expected two finite numbers: X Y") from None
        points.append(point)
    if len(set(points)) < 3:
        raise ValueError(f"{file_path}: at least three distinct XY points are required")
    if MultiPoint(points).convex_hull.area <= 0:
        raise ValueError(f"{file_path}: XY points must span a nonzero area")
    return points


def cell_gcode(points, settings, *, source_name):
    """Emit explicit XY and absolute E; never close or reorder the supplied path.

    Extrusion uses a rectangular bead: length * width * height * flow / filament area.
    Setup is inside LAYER:0 so every b/m/e variant resets its own extrusion state.
    """
    area = math.pi * (settings.filament_diameter / 2) ** 2
    extrusion_per_mm = settings.line_width * settings.layer_height * settings.flow_multiplier / area
    print_feed = settings.print_speed * 60
    travel_feed = settings.travel_speed * 60
    lines = [
        f";Converted cell path: {source_name}",
        ";XY units: mm; one continuous layer; original point order",
        f";Filament diameter: {settings.filament_diameter:g}; line width: {settings.line_width:g}; "
        f"layer height: {settings.layer_height:g}; flow: {settings.flow_multiplier:g}",
        ";LAYER_COUNT:1", ";LAYER:0",
        "G21", "G90", "M82", "G92 E0",
        f"M109 S{settings.nozzle_temperature:g}",
        "M107",
        f"G0 Z{settings.travel_height:g} F{travel_feed:g}",
        f"G0 X{points[0][0]:.5f} Y{points[0][1]:.5f} F{travel_feed:g}",
        f"G0 Z{settings.layer_height:g} F{print_feed:g}",
        ";TYPE:FILL",
    ]
    extrusion, elapsed = 0.0, 0.0
    for previous, point in zip(points, points[1:]):
        distance = math.dist(previous, point)
        extrusion += distance * extrusion_per_mm
        elapsed += distance / settings.print_speed
        if not math.isfinite(extrusion) or not math.isfinite(elapsed):
            raise ValueError(f"{source_name}: path or extrusion exceeds the supported numeric range")
        lines.append(f"G1 X{point[0]:.5f} Y{point[1]:.5f} E{extrusion:.8f} F{print_feed:g}")
    lines.extend([
        f";TIME_ELAPSED:{elapsed:.3f}",  # Path-only estimate; excludes heating/travel.
        f"G1 Z{settings.travel_height:g} F{print_feed:g}",
        "M400", "M104 S0", ";End of converted cell path",
    ])
    return "\n".join(lines) + "\n"


def convert_cell_path(source, destination, settings=None):
    """Convert one file without changing the source; return the generated path."""
    source, destination = Path(source), Path(destination)
    if source.resolve() == destination.resolve():
        raise ValueError("Converted output must not overwrite the source path")
    text = cell_gcode(read_cell_points(source), settings if settings is not None else load_cell_settings(), source_name=source.name)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination
