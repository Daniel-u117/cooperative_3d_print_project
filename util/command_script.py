from pathlib import Path
import numpy as np
import time

from conflict_detect.collision_utils import (
    segments_intersect, segment_intersects_circle, point_in_quad,
    rectangles_intersect, extract_xy, xy_points_after_m107,
    write_sim_file, sim_one_px, sim_px,
)
from .execution import execution_worker, RECOVERY_TIMEOUT
import requests
import random
import queue
import multiprocessing as mp

from shapely import Polygon
from shapely.ops import unary_union
import ast


from shapely.geometry import Polygon, mapping

from shapely.validation import explain_validity




import queue, time, uuid, traceback
from pathlib import Path

from collections import deque

















PHYSICAL_BOUNDS = {
    1: dict(
        elbow_joint_f=26,
        elbow_joint_b=-26,
        distal_arm_f=246,
        distal_arm_b=-26,
        prox_joint_f=8,
        prox_joint_b=-8,
        proximal_arm_f=230,
        proximal_arm_b=-8,
    ),
    2: dict(
        elbow_joint_f=-25,
        elbow_joint_b=25,
        distal_arm_f=253,
        distal_arm_b=-25,
        prox_joint_f=8,
        prox_joint_b=-8,
        proximal_arm_f=234,
        proximal_arm_b=-8,
    ),
    3: dict(
            elbow_joint_f=-25,
            elbow_joint_b=25,
            distal_arm_f=253,
            distal_arm_b=-25,
            prox_joint_f=8,
            prox_joint_b=-8,
            proximal_arm_f=234,
            proximal_arm_b=-8,
    )
}













# def pick_command_px(p, printer, excluded=None):
#     """
#     PROPOSAL ONLY.

#     Does not:
#       - change A -> N
#       - write order.txt

#     Those things happen only after simulation accepts the candidate.
#     """
#     excluded = set() if excluded is None else set(excluded)


#     #no mutatoin of printer[p].codes because
#     available = [
#         code
#         for code, value in printer[p].codes.items()
#         if value[0] == "A" and code not in excluded
#     ]

#     if not available:
#         return None



#     return random.choice(available)


# def pick_command_px(p, printer, excluded=None):
#     """
#     PROPOSAL ONLY.

#     Does not:
#       - change A -> N
#       - write order.txt

#     Those things happen only after simulation accepts the candidate.
#     """
#     excluded = set() if excluded is None else set(excluded)


#     #no mutatoin of printer[p].codes because
#     available = [
#         code
#         for code, value in printer[p].codes.items()
#         if value[0] == "A" and code not in excluded
#     ]

#     if not available:
#         return None



#     return random.choice(available)


def pick_command_px(p, printer, store_coords, b, excluded=None):
    print(b)
    """
    Propose the nearest available command that is not excluded.
    Does not modify availability, coordinates, or exclusions.
    """
    excluded = set() if excluded is None else set(excluded)
    if b:
        cur_x, cur_y = printer[p].prox_join[0], printer[p].prox_join[1]
    else:
        cur_x, cur_y = store_coords[p]
    nearest_code = None
    nearest_distance_squared = float("inf")

    for code, (availability, points) in printer[p].codes.items():
        if availability != "A" or code in excluded:
            continue

        if len(points) == 0:
            raise ValueError(f"p{p}: {code} has no stored coordinates")

        # Distance to this command's closest listed point.
        # Squared distances preserve nearest-first ordering.
        distance_squared = min(
            (x - cur_x) ** 2 + (y - cur_y) ** 2
            for x, y in points
        )

        if distance_squared < nearest_distance_squared:
            nearest_code = code
            nearest_distance_squared = distance_squared

    return nearest_code



def _try_send_sim_frame(simulation_queue, message):
    if simulation_queue is None:
        return

    try:
        simulation_queue.put_nowait(message)
        return True
    except queue.Full:
        # The optional visualization is allowed to drop frames.
        # Collision computation must never depend on GUI speed.
        return False


def _validated_arm_bounds(px, x, y, physical_bounds, context):
    """Validate the returned corners before using them in collision geometry.

    boundary_points must already return perimeter-ordered corners in the same
    common coordinate frame for every printer. Do not guess a transform or
    repair a crossing polygon, because that could hide a geometry error.
    """
    boundary = px.boundary_points(x, y, **physical_bounds)
    corners = {}
    polygons = []
    for arm in ("distal", "proximal"):
        points = np.asarray(boundary[arm], dtype=float)
        if points.shape != (4, 2) or not np.isfinite(points).all():
            raise ValueError(
                f"{context}: invalid {arm} corners at raw position {(x, y)}: "
                f"{points.tolist()}"
            )
        polygon = Polygon(points)
        if not polygon.is_valid or polygon.area <= 0:
            raise ValueError(
                f"{context}: invalid {arm} polygon at raw position {(x, y)}; "
                f"{explain_validity(polygon)}; corners={points.tolist()}. "
                "Check boundary_points() corner order."
            )
        corners[arm] = tuple(map(tuple, points.tolist()))
        polygons.append(polygon)
    return corners, polygons



def polygon_at_position(p, printer, raw_position):
    x, y = map(float, raw_position)

    _, arm_polygons = _validated_arm_bounds(
        printer[p], x, y, PHYSICAL_BOUNDS[p],
        f"p{p}: observed position",
    )

    return unary_union(arm_polygons)


def simulate_candidate(
    p,
    candidate,
    printer,
    store_coords,
    config,
    simulation_queue=None,
    visualization_delay=0.0,
):
    """
    Speculatively replay one candidate without changing real store_coords.

    Returns:
        candidate_polygon
        candidate_final_coords

    candidate_polygon is the swept occupied area for printer p during
    the candidate simulation.

    candidate_final_coords is a COPY of store_coords containing the final
    simulated coordinate for p.
    """
    local_coords = {q: tuple(position) for q, position in store_coords.items()}

    command = config.out_px[p] / candidate

    arm_bounds, initial_polygons = _validated_arm_bounds(
        printer[p], *local_coords[p], PHYSICAL_BOUNDS[p],
        f"p{p}:{candidate}, initial pose",
    )
    rectangles = list(initial_polygons)
    frame_count = 0


    _try_send_sim_frame(
        simulation_queue,
        {
            "type": "start",
            "printer": p,
            "gcode": candidate,
            "positions": dict(local_coords),
            "arm_bounds": arm_bounds,
            "physical_bounds": PHYSICAL_BOUNDS,
        },
    )

    with open(command, "r") as r:
        print(command)
        for line_number, line in enumerate(r, start=1):
            if not line.strip():
                continue
            frame = ast.literal_eval(line.strip())

            if not isinstance(frame, list) or len(frame) < 2:
                raise ValueError(f"Invalid simulation frame: {frame}")

            *positions, moved_printer = frame
            moved_printer = int(moved_printer)

            if len(positions) != len(printer):
                raise ValueError(
                    f"Expected {len(printer)} positions, got {len(positions)}"
                )

            # For this candidate file, p should be the moving printer.
            # If not, reject the malformed simulation file immediately.
            if moved_printer != p:
                raise ValueError(
                    f"{command} says p{moved_printer} moved while "
                    f"simulating p{p}"
                )

            # The file's other-printer positions are generation-time snapshots.
            # Keep their supplied telemetry positions; only this candidate moves.
            if set(printer) != set(range(1, len(positions) + 1)):
                raise ValueError("Simulation files require printer IDs 1..N")
            x, y = positions[p - 1]
            local_coords[p] = (float(x), float(y))
            print(f'local_coords: {local_coords}')

            cur_x_raw, cur_y_raw = local_coords[p]

            arm_bounds, arm_polygons = _validated_arm_bounds(
                printer[p], cur_x_raw, cur_y_raw, PHYSICAL_BOUNDS[p],
                f"p{p}:{candidate}, line {line_number}",
            )
            rectangles.extend(arm_polygons)
            frame_count += 1

            _try_send_sim_frame(
                simulation_queue,
                {
                    "type": "frame",
                    "printer": p,
                    "gcode": candidate,
                    "positions": dict(local_coords),
                    "arm_bounds": arm_bounds,
                    "physical_bounds": PHYSICAL_BOUNDS,
                },
            )
            
            if visualization_delay > 0.0:
                time.sleep(visualization_delay)

    if frame_count == 0:
        raise ValueError(
            f"No simulated occupied geometry generated for {command}"
        )

    
    return unary_union(rectangles), local_coords



def collides_with_stored(candidate_polygon, p, store_polygons):
    """
    Skip printers whose polygon is still None, as requested.
    """
    collisions = {}

    for other_p, other_polygon in store_polygons.items():
        if other_p == p or other_polygon is None:
            continue

        overlap = candidate_polygon.intersection(other_polygon)
        if not overlap.is_empty:
            collisions[other_p] = overlap.area

    return collisions


def mark_gcode_unavailable(candidate, printer):
    """
    Once accepted, the same physical region becomes unavailable to every
    printer that has that code.
    """
    for px in printer.values():
        if candidate in px.codes:
            px.codes[candidate][0] = "N"

def command_script(printer, config, store_coords, tracker_event_queue,
                   execution_request_queue, execution_result_queue,
                   tracker_control_queue, printing_queue, stop_requested,
                   execution_finished, error_queue, simulation_queue=None,
                   stage="m", store_polygons = None):
    
    path = Path(r"C:\Users\nural\python2")
    if store_polygons is None:
        raise ValueError("Pass the shared store_polygons dictionary")

    count = {
        1: 1,
        2: 1,
        3: 1
    }

    try:
        ip = {1:'192.168.0.15',
              2: '192.168.0.16'}


        if stage not in {"b", "m", "e"}:
            raise ValueError("stage must be b, m, or e")

        
        with open(path / "order.txt", "w") as out:
            out.write("Begin experiment\n")

        loop_check = set()
        in_flight = set()
        blocked = set()
        recovery_since_progress = False



        while not stop_requested.is_set():

            #any time 'continue' is used it means restart the loop

            printers_with_work = {
                q for q in ip
                if any(value[0] == "A" for value in printer[q].codes.values())
            }
            print(f'printers with work {printers_with_work}')

            if not printers_with_work and not in_flight:
                stop_requested.set()
                break
            ####this if branch is supposed to deal with loop check
            if (
                printers_with_work
                and printers_with_work <= loop_check
                and not in_flight
                and set(ip) <= blocked
                ):
                if recovery_since_progress:
                    raise RuntimeError(
                        "All remaining candidates still fail after recovery; "
                        "check geometry and the configured recovery route"
                    )
                recovery_since_progress = True

                with open(path / "order.txt", "a") as out:
                    out.write("deadlock: requesting recovery for both printers\n")

                #tell the execution script to start recovery
                execution_request_queue.put(("__recover__", None))


                deadline = time.monotonic() + RECOVERY_TIMEOUT + 10.0
                while not stop_requested.is_set():
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Execution worker did not complete recovery") 
                    try:
                        result, observed = execution_result_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if result != "recovered":
                        raise RuntimeError(f"Recovery failed: {(result, observed)}")
                    if not isinstance(observed, dict) or set(observed) != set(printer):
                        raise RuntimeError("Recovery did not return both printer positions")

                    positions = {
                        q: tuple(map(float, observed[q])) for q in printer
                    }
                    # Build every polygon before publishing any replacement.
                    footprints = {
                        q: polygon_at_position(q, printer, positions[q])
                        for q in printer
                    }

                    print(
                        f"POSE_WRITE t={time.monotonic():.6f} reason=recovery "
                        f"old={store_coords.copy()} new={positions}", flush=True,
                    )


                    store_coords.update(positions)
                    store_polygons.update(footprints)
                    # No normal event is released until BOTH dictionaries are updated.
                    loop_check.clear()
                    for q in sorted(blocked):
                        if any(v[0] == "A" for v in printer[q].codes.values()):
                            tracker_control_queue.put(("retry", q))
                            blocked.remove(q)
                    with open(path / "order.txt", "a") as out:
                        out.write(f"recovered: observed positions={positions}\n")
                    break
                if stop_requested.is_set():
                    return
                continue

                

            # get() removes ONLY the oldest message. Everything else waits.
            try:
                p, status = tracker_event_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if status != "free":
                raise RuntimeError(f"Unexpected tracker event: {(p, status)}")

            if p not in printer:
                raise RuntimeError(f"Unknown printer in tracker event: {p}")

            if p in in_flight:
                in_flight.remove(p)
                loop_check.clear()
                recovery_since_progress = False
                for q in sorted(blocked):
                    if any(v[0] == "A" for v in printer[q].codes.values()):
                        tracker_control_queue.put(("retry", q))
                        blocked.remove(q)

            # Required before no-print or print controls can be applied.


            print(
                f"FREE_CONSUMED t={time.monotonic():.6f} p={p} "
                f"stored_now={store_coords.copy()}", flush=True,
            )



            tracker_control_queue.put(("hold", p))

            #past this is wehre simulation starts




            # if status is not free, just continue to probe the queue
            #if status says Finsihed, that means all gcode commands have been executed
            #in that case, send a command ot all printers to raise by Z = +10 and break from the loop

            # if status != "free":
            #     raise ValueError(f"Unexpected idle message: {(p, status)}")












            #how is this different from coordinates = store_coordinates.copy()
            coordinates = {
                q: tuple(position) 
                for q, position in store_coords.copy().items()
            }


            #hwo is this different from comparison_polygons = store_polygons.copy()
            comparison_polygons = {
                q: polygon_at_position(q, printer, coordinates[q])
                for q in printer
            }

            # Existing simulation files embed coordinates. Regenerate using
            # telemetry, without changing sim_px or simulate_candidate.

            print(
                f"POSE_SNAPSHOT t={time.monotonic():.6f} "
                f"candidate_printer={p} coordinates={coordinates}", flush=True,
            )

            sim_px(p, printer[p].codes, coordinates, config)







            rejected = set()
            eligible_count = sum(
            value[0] == "A" for value in printer[p].codes.values()
                )





            
            if eligible_count == 0:
                tracker_control_queue.put(("no print", p))
                blocked.add(p)
                continue



            while not stop_requested.is_set():
                if count[p] == 1:
                    candidate = pick_command_px(p, printer, coordinates, b = True, excluded=rejected)
                else:
                    candidate = pick_command_px(p, printer, coordinates, b = False, excluded=rejected)






                if candidate is None:
                    break
                candidate_polygon, candidate_final_coords = simulate_candidate(
                    p=p, candidate=candidate, printer=printer,
                    store_coords=coordinates, config=config,
                    simulation_queue=simulation_queue,
                    visualization_delay=0.03 if simulation_queue is not None else 0,
                )




                collisions = collides_with_stored(
                    candidate_polygon, p, comparison_polygons,
                )

                simulation_result = {
                    "type": "result", "printer": p, "gcode": candidate,
                    "accepted": not bool(collisions),
                    "overlap": sum(collisions.values()),
                }
                if simulation_queue is not None:
                    simulation_result.update({
                        "collisions": dict(collisions),
                        "positions": candidate_final_coords,
                        "physical_bounds": PHYSICAL_BOUNDS,
                        "candidate_polygon": mapping(candidate_polygon),
                        "stored_polygons": {
                            other_p: mapping(polygon)
                            for other_p, polygon in comparison_polygons.items()
                            if other_p != p and polygon is not None
                        },

                        "overlap_polygons": {
                            other_p: mapping(candidate_polygon.intersection(comparison_polygons[other_p]))
                            for other_p in collisions
                        },
                    })
                _try_send_sim_frame(simulation_queue, simulation_result)
                if collisions:
                    rejected.add(candidate)
                    with open(path / "order.txt", "a") as out:
                        out.write(f"reject p{p}:{candidate}; collision={collisions}\n")
                    continue





                mark_gcode_unavailable(candidate, printer)





                #there is a stage there shoudl says only middle is used
                if count[p] == 1:
                    filename = f"{Path(candidate).stem}_b.gcode"
                else:
                    filename = f"{Path(candidate).stem}_{stage}.gcode"


                with open(path / "order.txt", "a") as out:
                    out.write(f"execute p{p}:{filename}\n")







                in_flight.add(p)
                execution_request_queue.put((filename, p))

                # First wait: execution confirms submission.
                while not stop_requested.is_set():
                    try:
                        result, response_p = execution_result_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if (result, response_p) != ("sent", p):
                        raise RuntimeError(f"Execution failed: {(result, response_p)}")
                    break








                # Second wait: tracking confirms extrusion for this printer.
                # Do NOT read tracker_event_queue in either waiting loop.



                if stop_requested.is_set():
                    return


                #now the command script is responibel for telling the tracker to watch after having
                # sent the execution command to the printer, not sure how correct this is
                #but we will see, shouldn't be much of a difference because either way
                #the tracker watches only after the execution command is sent

                tracker_control_queue.put(("print", p))

                while not stop_requested.is_set():
                    try:
                        result, response_p = printing_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if (result, response_p) != ("Is printing", p):
                        raise RuntimeError(f"Unexpected printing message: {(result, response_p)} instead of {("Is printing", p)}")


                    print(
                        f"PRINT_ACK t={time.monotonic():.6f} p={p} file={filename} "
                        f"stored_now={store_coords.copy()}", flush=True,
                    )
                    
                    
                    count[p] += 1
                    print(count)
                    break
                break  # Return to the outer FIFO get() only now.

            #write a script that checks the length of the rejected and the amount of codes that p1 was eligible to take 'A' status
            #if they are equal, then px in this turn was not eligible to take anything and has to pass his turn
            # then you run tracker_control_queue.put(('no print', p))
            if not stop_requested.is_set() and len(rejected) == eligible_count and eligible_count > 0:
                tracker_control_queue.put(("no print", p))
                blocked.add(p)
                loop_check.add(p) 


            

    except Exception:
        error_queue.put(("command_script", traceback.format_exc()))
        stop_requested.set()
        raise
    finally:
        _try_send_sim_frame(simulation_queue, {"type": "stop"})
        execution_finished.set()





