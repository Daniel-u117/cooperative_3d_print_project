#used for collision detection and other things can be incorporated like which nodes are available to be transported for later 

#won't be used much for now, is here just for additional functionality but is not required to work

import numpy as np
import queue
from kinematics.animation import closest_point_on_segment
import requests
from util.network_utils import RobotConnection, run_netsh, current_wifi, tcp_reachable, reconnect_to_ambots, get_status
import traceback

import time
# run_netsh("disconnect", f'interface=Wi-Fi') --> this will disrupt the connection on purpose



#| # get_status reads rr_status coords.machine without a coordinate transform.
#| # Keep the corresponding machine-position field in the same-response watch.


TRACK_POSITION_FIELD = "machinePosition"
WATCH_XY_EPSILON = 0.01  # mm: minimum reported X/Y progress between samples
WATCH_TIMEOUT = 300.0  # seconds allowed for heating, travel, and extrusion confirmation



def estimate_extrusion_rate(position, now, previous):
    """Return (signed average mm/s or None, next (position, time) baseline)."""
    if position is None:
        return None, None
    position, now = float(position), float(now)
    if not (np.isfinite(position) and np.isfinite(now)):
        raise ValueError("Extruder position and sample time must be finite")
    current = (position, now)
    if previous is None:
        return None, current
    old_position, old_time = previous
    elapsed = now - old_time
    if elapsed <= 0.0:
        return None, current
    rate = (position - old_position) / elapsed
    if not np.isfinite(rate):
        raise ValueError(f"Invalid estimated extrusion rate: {rate}")
    return rate, current




def track(ip, store_coords, tracker_event_queue, tracker_control_queue,
          printing_queue, stop_requested, error_queue, position_queue1, printer = None, store_polygons = None, polygon_at_position = None, connection=None):
    connection = connection or RobotConnection(ip[1], ip[2])
    # One small state dictionary prevents duplicate free messages:
    # free -> queued (includes being analyzed) -> watch -> printing -> queued.
    # "no print" parks a printer; it cannot be silently requeued.
    state = {p: "free" for p in ip}
    try:
        while not stop_requested.is_set():
            statuses, machine_coords = get_status(connection=connection)

            for p in ip:


                frame = [
                    (float(machine_coords[q - 1][0]),
                        float(machine_coords[q - 1][1]))
                    for q in sorted(ip)
                ] + [p]
                try:
                    position_queue1.put_nowait(frame)
                except queue.Full:

                    pass








                # if state[p] == "watch":

                #     previous_extruding_sample = None

                #     while not stop_requested.is_set():

                        

                #         extrusion = requests.get(f'http://{ip[p]}/rr_model?flags=d99fn').json()
                #         # Check current processing status in the SAME response,
                #         # so an idle machine's old/nonzero value cannot trigger it.
                #         print(f'extrusion check1 {extrusion['result']["state"]["status"], extrusion['result']['job'], extrusion['result']["state"]["status"]}')

                #         extrusion.raise_for_status()
                #         sample = extrusion.json()['result']
                #         move = sample['move']
                #         axes = {axis["letter"]: axis for axis in move["axes"]}

                #         statuses, machine_coords = get_status(connection=connection)
                        
                #         frame = [
                #         (float(machine_coords[q - 1][0]),
                #             float(machine_coords[q - 1][1]))
                #         for q in sorted(ip)
                #         ] + [p]

                #         try:
                #             position_queue1.put_nowait(frame)
                #         except queue.Full:

                #             pass


                #         if (extrusion['result']["state"]["status"] == "processing" 
                #                and extrusion['result']['job']['rawExtrusion'] is not None and extrusion['result']['job']['rawExtrusion'] > 0):
                #             print(f' extrusion check2 {extrusion['result']['job'], extrusion['result']["state"]["status"]}')
                #             break





                #     if stop_requested.is_set():
                #         return

                     
                #     # Refresh only this printer when its extrusion begins.
                    



                #     raw_position = (
                #         float(machine_coords[p - 1][0]),
                #         float(machine_coords[p - 1][1]),
                #     )

                #     store_coords[p] = raw_position
                #     store_polygons[p] = polygon_at_position(
                #         p, printer, raw_position
                #     )






                #     state[p] = "printing"
                #     printing_queue.put(("Is printing", p))

###############################################################################################################
                if state[p] == "watch":
                    watch_deadline = time.monotonic() + WATCH_TIMEOUT
                    sample_status, raw_position = "unknown", None
                    previous_extruding_sample = None
                    previous_job_file = None
                    previous_extruder_sample = None
                    while not stop_requested.is_set():
                        if time.monotonic() >= watch_deadline:
                            raise TimeoutError(
                                f"p{p}: no moving extrusion confirmed within {WATCH_TIMEOUT:g}s; "
                                f"last status={sample_status}, position={raw_position}. "
                                "Job may have finished or failed before confirmation; it was not resent."
                            )
                        response = requests.get(f"http://{ip[p]}/rr_model?flags=d99fn", timeout=5)
                        response.raise_for_status()
                        sample = response.json()["result"]
                        now = time.monotonic()
                        move = sample["move"]
                        axes = move['axes']



                        # Same machine-coordinate convention as get_status.

                        raw_position = (
                            float(axes[0]["machinePosition"]),
                            float(axes[1]["machinePosition"]),
                        )


                        if not all(np.isfinite(value) for value in raw_position):
                            raise ValueError(f"Invalid observed position for p{p}: {raw_position}")

                        


                        sample_status = sample['state']['status']
                        extruder_position = move["extruders"][0]["position"]



                        if sample_status == "processing":
                            rate, previous_extruder_sample = estimate_extrusion_rate(
                                extruder_position, now, previous_extruder_sample,
                            )
                        else:
                            rate = None
                            previous_extruder_sample = None
                        print(
                            f"WATCH_SAMPLE t={now:.6f} p={p} "
                            f"status={sample_status} E={extruder_position} "
                            f"estimated_rate={rate} xy={raw_position}",
                            flush=True,
                        )





                        
                        # A positive rate alone also accepts G1 F200 E3 at
                        # the old X/Y position, before region travel in 13_b.
                        extruding = (
                            sample_status == "processing"
                            and rate is not None
                            and rate > 0.0
                        )

                        # Require two consecutive positive estimates and XY progress.
                        # Stationary priming alone must not release the scheduler.
                        xy_progress = False
                        if extruding and previous_extruding_sample is not None:
                            previous_x, previous_y = previous_extruding_sample
                            xy_progress = max(
                                abs(raw_position[0] - previous_x),
                                abs(raw_position[1] - previous_y),
                            ) > WATCH_XY_EPSILON

                        if xy_progress:
                            break

                        # Zero, negative, or unavailable rates reset this sequence.
                        previous_extruding_sample = raw_position if extruding else None
                        stop_requested.wait(0.1)

####################################################################################################



                    if stop_requested.is_set():
                        return

                    # Publish both stores before allowing the scheduler to proceed.
                    footprint = polygon_at_position(p, printer, raw_position)
                    previous = store_coords.get(p)

                    store_coords[p] = raw_position
                    store_polygons[p] = footprint

                    print(
                        f"POSE_WRITE t={time.monotonic():.6f} p={p} "
                        f"reason=extrusion old={previous} new={raw_position} "
                        f"estimated_rate={rate}",
                        flush=True,
                    )

                    state[p] = "printing"
                    printing_queue.put(("Is printing", p))

                        # Leave the outer for-p loop. Its status sample predates this watch.
                        # The next outer iteration will obtain fresh status.
                    break
                    

                #     # I = idle
                #     # B = busy / live movement / macro
                #     # P = printing a file
                #     # D = pausing
                #     # S = paused/stopped
                #     # R = resumin




                while (
                    not stop_requested.is_set()
                    and state[p] == "printing"
                    and statuses[p - 1] in {"D", "S"}
                ):




                    requests.get(f"http://{ip[p]}/rr_gcode", params={"gcode": 'M24'}, timeout=5)
                    statuses, machine_coords = get_status(connection=connection)
                    time.sleep(0.2)








                if statuses[p - 1] == "I" and state[p] in {"free", 'printing'}:
                    # Both the initial idle position and the final position
                    # must be available before command_script takes this item.
                    raw_position = (
                        float(machine_coords[p - 1][0]),
                        float(machine_coords[p - 1][1]),
                    )

                    footprint = polygon_at_position(p, printer, raw_position)
                    previous = store_coords.get(p)

                    store_coords[p] = raw_position
                    store_polygons[p] = footprint

                    print(
                        f"POSE_WRITE t={time.monotonic():.6f} p={p} "
                        f"reason=idle prior_state={state[p]} old={previous} "
                        f"new={raw_position} status={statuses[p - 1]}",
                        flush=True,
                    )











                    if state[p] == "printing":
                        state[p] = "free"
                    try:
                        tracker_event_queue.put_nowait((p, "free"))
                        time.sleep(0.1)
                    except queue.Full:
                        # Keep p eligible and try again on the next poll.
                        continue
                    state[p] = "queued"  # exactly one queued entry per printer

            # One control message per pass. On timeout, poll again and retry
            # any free-printer notification that could not enter the FIFO.
            try:
                action, p = tracker_control_queue.get(timeout=0.2)
                print(action, p)
            except queue.Empty:
                continue
            if p not in state:
                raise ValueError(f"Unknown printer: {p}")

            if action == "hold" and state[p] == "queued":
                state[p] = "hold"
            elif action == "print" and state[p] == "hold":
                state[p] = "watch"
            elif action == "no print" and state[p] == "hold":
                state[p] = "blocked"
            #where will action = 'retry' be assigned
            elif action == "retry" and state[p] == "blocked":
                state[p] = "free"
            else:
                raise RuntimeError(f"Cannot apply {(action, p)} while p{p} is {state[p]}")



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


