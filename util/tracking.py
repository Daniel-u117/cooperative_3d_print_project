"""Poll both robots without blocking on one job's printing acknowledgement."""
import math
import queue
import time
import traceback

import requests

from util.job_tracking import START_VARIABLE, DONE_VARIABLE
from util.network_utils import RobotConnection, read_status

TRACK_POSITION_FIELD = "machinePosition"
WATCH_XY_EPSILON = 0.01
WATCH_TIMEOUT = 300.0
COMPLETION_TIMEOUT = 10.0
POLL_INTERVAL = 0.1


def estimate_extrusion_rate(position, now, previous):
    if position is None:
        return None, None
    position, now = float(position), float(now)
    if not (math.isfinite(position) and math.isfinite(now)):
        raise ValueError("Extruder position and sample time must be finite")
    current = (position, now)
    if previous is None:
        return None, current
    old_position, old_time = previous
    elapsed = now - old_time
    if elapsed <= 0:
        return None, current
    rate = (position - old_position) / elapsed
    if not math.isfinite(rate):
        raise ValueError("Invalid estimated extrusion rate")
    return rate, current


def track(ip, store_coords, tracker_event_queue, tracker_control_queue,
          printing_queue, stop_requested, error_queue, position_queue1,
          printer=None, store_polygons=None, polygon_at_position=None, connection=None):
    connection = connection or RobotConnection(ip[1], ip[2])
    state = {p: "free" for p in ip}
    jobs = {}

    def publish(p, position, reason):
        position = tuple(map(float, position[:2]))
        if len(position) != 2 or not all(math.isfinite(v) for v in position):
            raise ValueError(f"p{p}: invalid machine position {position}")
        footprint = polygon_at_position(p, printer, position)
        previous = store_coords.get(p)
        store_coords[p] = position
        store_polygons[p] = footprint
        print(f"POSE_WRITE t={time.monotonic():.6f} p={p} reason={reason} "
              f"old={previous} new={position}", flush=True)

    def model(p, **params):
        response = requests.get(f"http://{ip[p]}/rr_model", params=params, timeout=5)
        response.raise_for_status()
        return response.json()["result"]

    try:
        while not stop_requested.is_set():
            # One snapshot and at most one job sample per printer per pass.
            statuses, machine_coords = read_status(connection, timeout=5)
            for p in ip:
                if stop_requested.is_set():
                    return
                if state[p] in {"watch", "printing"}:
                    job = jobs[p]
                    # Read persistent completion first, then fresh machine status.
                    # Do not use flags=f for globals: they are not live-only fields.
                    globals_ = model(p, key="global", flags="d99n")
                    if not isinstance(globals_, dict):
                        raise ValueError(f"p{p}: firmware did not expose global job markers")
                    job["started"] |= globals_.get(START_VARIABLE) == job["token"]
                    job["done"] |= globals_.get(DONE_VARIABLE) == job["token"]
                    sample = model(p, flags="d99fn")
                    now = time.monotonic()
                    status = sample["state"]["status"]
                    axes = sample["move"]["axes"]
                    position = tuple(float(axis[TRACK_POSITION_FIELD]) for axis in axes[:2])
                    if len(position) != 2 or not all(math.isfinite(v) for v in position):
                        raise ValueError(f"p{p}: invalid job position {position}")
                    machine_coords[p - 1] = position
                    extruders = sample["move"].get("extruders", [])
                    extrusion = extruders[0].get("position") if extruders else None
                    rate = None
                    if status == "processing" and job["started"]:
                        rate, job["previous_e"] = estimate_extrusion_rate(extrusion, now, job["previous_e"])
                    else:
                        job["previous_e"] = None
                    signature = (status, job["started"], job["done"])
                    if signature != job["signature"] or now >= job["next_report"]:
                        print(f"JOB_SAMPLE p={p} file={job['file']} phase={state[p]} status={status} "
                              f"started={job['started']} completed={job['done']} "
                              f"E={extrusion} estimated_rate={rate} xy={position}", flush=True)
                        job["signature"], job["next_report"] = signature, now + 5.0

                    if job["started"] and job["done"] and status == "idle":
                        missed_start = state[p] == "watch"
                        publish(p, position, "job_complete")
                        print(f"JOB_COMPLETE p={p} file={job['file']} token={job['token']} "
                              f"printing_sample_missed={missed_start}", flush=True)
                        if missed_start:
                            # Release the scheduler's acknowledgement wait FIRST.
                            printing_queue.put(("Completed", p))
                        state[p] = "free"
                        statuses[p - 1] = "I"
                        del jobs[p]
                    else:
                        if status == "idle" and job["started"]:
                            if job["idle_since"] is None:
                                job["idle_since"] = now
                            if now - job["idle_since"] >= COMPLETION_TIMEOUT:
                                raise RuntimeError(f"p{p}: {job['file']} idle without its completion token; "
                                                   "job may have failed or been cancelled; not marked complete")
                        else:
                            job["idle_since"] = None
                        if state[p] == "watch":
                            previous = job["previous_xy"]
                            extruding = rate is not None and rate > 0
                            moving = extruding and previous is not None and max(
                                abs(a - b) for a, b in zip(position, previous)) > WATCH_XY_EPSILON
                            job["previous_xy"] = position if extruding else None
                            if moving:
                                publish(p, position, "extrusion")
                                state[p] = "printing"
                                printing_queue.put(("Is printing", p))
                            elif now >= job["deadline"]:
                                raise TimeoutError(f"p{p}: no moving extrusion or verified completion within "
                                                   f"{WATCH_TIMEOUT:g}s for {job['file']}; status={status}, "
                                                   f"started={job['started']}, completed={job['done']}, "
                                                   f"position={position}. Job may have finished or failed; not resent.")
                        # A job remains active until its own completion token and idle.
                        # Paused jobs remain paused; polling never sends M24.
                        continue

                if state[p] == "free" and statuses[p - 1] == "I":
                    publish(p, machine_coords[p - 1], "idle")
                    try:
                        tracker_event_queue.put_nowait((p, "free"))
                    except queue.Full:
                        pass  # Retry next pass; never lose a completion event.
                    else:
                        state[p] = "queued"

            for p in ip:
                frame = [tuple(map(float, machine_coords[q - 1][:2])) for q in sorted(ip)] + [p]
                try:
                    position_queue1.put_nowait(frame)
                except queue.Full:
                    pass

            # Process a bounded batch of controls so neither queue traffic nor a
            # single job can starve telemetry for the other robot.
            for _ in range(2 * len(ip)):
                try:
                    message = tracker_control_queue.get_nowait()
                except queue.Empty:
                    break
                action, p, *details = message
                if p not in state:
                    raise ValueError(f"Unknown printer: {p}")
                if action == "hold" and state[p] == "queued":
                    state[p] = "hold"
                elif action == "print" and state[p] == "hold":
                    if len(details) != 1 or not isinstance(details[0], dict):
                        raise ValueError(f"p{p}: job identity required for tracking; regenerate prepared jobs")
                    identity = details[0]
                    if not identity.get("file") or not identity.get("token"):
                        raise ValueError(f"p{p}: incomplete job identity")
                    jobs[p] = dict(identity, started=False, done=False, previous_e=None,
                                   previous_xy=None, deadline=time.monotonic() + WATCH_TIMEOUT,
                                   idle_since=None, signature=None, next_report=0.0)
                    state[p] = "watch"
                    print(f"JOB_WATCH p={p} file={identity['file']} token={identity['token']}", flush=True)
                elif action == "no print" and state[p] == "hold":
                    state[p] = "blocked"
                elif action == "retry" and state[p] == "blocked":
                    state[p] = "free"
                else:
                    raise RuntimeError(f"Cannot apply {(action, p)} while p{p} is {state[p]}")
            stop_requested.wait(POLL_INTERVAL)
    except Exception:
        error_queue.put(("track", traceback.format_exc()))
        stop_requested.set()
        raise


############################################ do not touch this part, it is only for the further extension of collision management#########################################
            # if pre_switch.is_set():
            #     if p == 1:


            #         cur_x_raw1, cur_y_raw1 = store_pos[1]
            #         px1 = printer[1]
            #         prox1, elbow1, cur1 = px1.coords_to_plot(cur_x_raw1, cur_y_raw1)

            #         cur_x_raw2, cur_y_raw2 = store_pos[2]
            #         px2 = printer[2]
            #         prox2, elbow2, cur2 = px2.coords_to_plot(cur_x_raw2, cur_y_raw2)



            #         close = closest_point_on_segment([elbow2[1], elbow2[0]], [cur2[1], cur2[0]], [cur1[1], cur1[0]])

            #         min_d = min([np.linalg.norm(cur1 - cur2), close[2], np.linalg.norm(cur1 - elbow2)])
            #         #since we have post pause and pre pause, we can have differnet min_ds for different siutaitons
                    
            #         if min_d < 0:
            #             #collision_procedure() 
            #             #because this is inside execute, we need to return collision and have teh higher-level scripts decide what to execute
            #             #execute should have a 'saftey' so that once it is told to just do the pause or something, it doesn't just do an error again
            #             #this will be the equivalent of going to home and doing something else (having printer 2 do another print or just stay at home)
            #             collision_pre_pause.set()


            #     if p == 2:
            #         cur_x_raw1, cur_y_raw1 = store_pos[1]
            #         px1 = printer[1]
            #         prox1, elbow1, cur1 = px1.coords_to_plot(cur_x_raw1, cur_y_raw1)

            #         cur_x_raw2, cur_y_raw2 = store_pos[2]
            #         px2 = printer[2]
            #         prox2, elbow2, cur2 = px2.coords_to_plot(cur_x_raw2, cur_y_raw2)


            #         #the order variables and the variables themeslves change in close and min_d depending on 'p'
            #         close = closest_point_on_segment([elbow1[1], elbow1[0]], [cur1[1], cur1[0]], [cur2[1], cur2[0]])
            #         min_d = min([np.linalg.norm(cur2 - cur1), close[2], np.linalg.norm(cur2 - elbow1)])


            #         if min_d < 0:
            #             # for now i am sepearting pre pause and post pause tracking, but honestly one can just have the pauses be set or not set outisde
            #             #collision_procedure() 
            #             #because this is inside execute, we need to return collision and have teh higher-level scripts decide what to execute
            #             #execute should have a 'saftey' so that once it is told to just do the pause or something, it doesn't just do an error again
            #             #this will be the equivalent of going to home and doing something else (having printer 2 do another print or just stay at home)
            #             collision_pre_pause.set()


            # elif post_switch.is_set():
            #     if p == 1:
            #         cur_x_raw1, cur_y_raw1 = store_pos[1]
            #         px1 = printer[1]
            #         prox1, elbow1, cur1 = px1.coords_to_plot(cur_x_raw1, cur_y_raw1)

            #         cur_x_raw2, cur_y_raw2 = store_pos[2]
            #         px2 = printer[2]
            #         prox2, elbow2, cur2 = px2.coords_to_plot(cur_x_raw2, cur_y_raw2)



            #         close = closest_point_on_segment([elbow2[1], elbow2[0]], [cur2[1], cur2[0]], [cur1[1], cur1[0]])

            #         min_d = min([np.linalg.norm(cur1 - cur2), close[2], np.linalg.norm(cur1 - elbow2)])
            #         #since we have post pause and pre pause, we can have differnet min_ds for different siutaitons
                    
            #         if min_d < 0:
            #             #collision_procedure() 
            #             #because this is inside execute, we need to return collision and have teh higher-level scripts decide what to execute
            #             #execute should have a 'saftey' so that once it is told to just do the pause or something, it doesn't just do an error again
            #             #this will be the equivalent of going to home and doing something else (having printer 2 do another print or just stay at home)
            #             collision_post_pause.set()


            #     if p == 2:
            #         cur_x_raw1, cur_y_raw1 = store_pos[1]
            #         px1 = printer[1]
            #         prox1, elbow1, cur1 = px1.coords_to_plot(cur_x_raw1, cur_y_raw1)

            #         cur_x_raw2, cur_y_raw2 = store_pos[2]
            #         px2 = printer[2]
            #         prox2, elbow2, cur2 = px2.coords_to_plot(cur_x_raw2, cur_y_raw2)


            #         #the order variables and the variables themeslves change in close and min_d depending on 'p'
            #         close = closest_point_on_segment([elbow1[1], elbow1[0]], [cur1[1], cur1[0]], [cur2[1], cur2[0]])
            #         min_d = min([np.linalg.norm(cur2 - cur1), close[2], np.linalg.norm(cur2 - elbow1)])


            #         if min_d < 0:
            #             # for now i am sepearting pre pause and post pause tracking, but honestly one can just have the pauses be set or not set outisde
            #             #collision_procedure() 
            #             #because this is inside execute, we need to return collision and have teh higher-level scripts decide what to execute
            #             #execute should have a 'saftey' so that once it is told to just do the pause or something, it doesn't just do an error again
            #             #this will be the equivalent of going to home and doing something else (having printer 2 do another print or just stay at home)
            #             collision_post_pause.set()
                      
#######################################################


