"""Finite, sequential candidate preview with no printer or Wi-Fi calls."""
import json
import queue
import threading

from shapely.geometry import mapping

from conflict_detect.collision_utils import sim_px
from kinematics.animation_simulation import animation_simulation
from util.command_script import (
    PHYSICAL_BOUNDS, collides_with_stored, mark_gcode_unavailable,
    pick_command_px, polygon_at_position, simulate_candidate,
)


def run_offline(printer, config, headless=False):
    positions = {p: (300.0, 0.0) for p in printer}
    frames = None if headless else queue.Queue()
    accepted = []
    rejected = 0
    counts = {p: 0 for p in printer}
    with (config.log_dir / "order.txt").open("w", encoding="utf-8") as log:
        while True:
            progressed = False
            for p in sorted(printer):
                sim_px(p, printer[p].codes, positions, config)
                excluded = set()
                while True:
                    candidate = pick_command_px(p, printer, positions,
                                                b=counts[p] == 0, excluded=excluded)
                    if candidate is None:
                        break
                    stored = {q: polygon_at_position(q, printer, pos)
                              for q, pos in positions.items()}
                    sweep, final = simulate_candidate(p, candidate, printer, positions,
                                                      config, simulation_queue=frames)
                    collisions = collides_with_stored(sweep, p, stored)
                    if frames is not None:
                        frames.put({
                            "type": "result", "printer": p, "gcode": candidate,
                            "positions": final, "accepted": not bool(collisions),
                            "overlap": sum(collisions.values()), "collisions": collisions,
                            "physical_bounds": PHYSICAL_BOUNDS,
                            "candidate_polygon": mapping(sweep),
                            "stored_polygons": {q: mapping(poly) for q, poly in stored.items() if q != p},
                            "overlap_polygons": {q: mapping(sweep.intersection(stored[q])) for q in collisions},
                        })
                    if collisions:
                        rejected += 1
                        excluded.add(candidate)
                        log.write(f"reject p{p}:{candidate}; collision={collisions}\n")
                        continue
                    mark_gcode_unavailable(candidate, printer)
                    positions = final
                    counts[p] += 1
                    accepted.append({"printer": p, "gcode": candidate})
                    log.write(f"simulate p{p}:{candidate}\n")
                    progressed = True
                    break
            if not progressed:
                break
    remaining = sorted({name for px in printer.values() for name, value in px.codes.items()
                        if value[0] == "A"})
    report = {"mode": "offline", "accepted": accepted, "rejected_attempts": rejected,
              "remaining_reachable_regions": remaining, "final_positions": positions}
    report_path = config.log_dir / "simulation_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Local simulation: {len(accepted)} regions accepted; {len(remaining)} blocked.")
    print(f"Report: {report_path}")
    if frames is not None:
        frames.put({"type": "stop"})
        animation_simulation(frames, threading.Event(), threading.Event(), printer)
    return report
