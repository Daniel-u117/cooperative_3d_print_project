import queue, time, uuid, traceback
from pathlib import Path
import subprocess
import requests
import re
import socket
import time

from .network_utils import get_status, reconnect_to_ambots, tcp_reachable, current_wifi, run_netsh




# REVIEW E1: INSERT
# INSERT this configuration and ONE essential recovery helper before execution_worker. Keep the
# current process/queue wiring. RECOVERY_PLAN must be configured; empty configuration fails before
# movement. Routes execute one printer and one leg at a time. No movement is resent after an uncertain
# response. Firmware wait/echo behavior: https://docs.duet3d.com/User_manual/Reference/Gcodes and
# https://github.com/Duet3D/RepRapFirmware/wiki/HTTP-requests . get_status is not supplied here;
# verify its coordinate convention and bounded network calls. Check that no other code consumes
# rr_reply for this HTTP client.
# Proposed code: remove only the first "#| " prefix from each line.
#| # Configure native per-printer X/Y waypoints and their execution order.
#| # Each entry is (printer_id, ((x1, y1), (x2, y2), ...)).
#| # Include each printer exactly once. A leg is a straight G1 move from the
#| # observed starting point / previous waypoint. Supply validated routes;
#| # this helper executes them and does not plan or certify collision clearance.
#| # Empty by design: no target or ordering has been supplied by the user yet.
RECOVERY_PLAN = (
    (1, ((300.0, 0.0),)),  
    (2, ((300.0, 0.0),)),  
)
RECOVERY_TIMEOUT = 180.0
RECOVERY_FEEDRATE = 1500.0
RECOVERY_TOLERANCE = 0.25
 
def recover_printers(ip, stop_requested):
    """Run configured routes serially; return observed final raw X/Y positions."""
    import math

    routes = []
    seen = set()
    for printer_id, points in RECOVERY_PLAN:
        if printer_id not in ip or printer_id in seen:
            raise ValueError("Recovery plan must name each printer exactly once")
        route = tuple(tuple(map(float, point)) for point in points)
        if not route or any(
            len(point) != 2 or not all(math.isfinite(v) for v in point)
            for point in route
        ):
            raise ValueError(f"Invalid recovery route for p{printer_id}")
        seen.add(printer_id)
        routes.append((printer_id, route))
    if seen != set(ip):
        raise ValueError("Configure RECOVERY_PLAN for both printers before recovery")
    if not (math.isfinite(RECOVERY_FEEDRATE) and RECOVERY_FEEDRATE > 0):
        raise ValueError("RECOVERY_FEEDRATE must be finite and positive")
    if not (math.isfinite(RECOVERY_TIMEOUT) and RECOVERY_TIMEOUT > 0):
        raise ValueError("RECOVERY_TIMEOUT must be finite and positive")
    if not (math.isfinite(RECOVERY_TOLERANCE) and RECOVERY_TOLERANCE >= 0):
        raise ValueError("RECOVERY_TOLERANCE must be finite and nonnegative")

    deadline = time.monotonic() + RECOVERY_TIMEOUT
    targets = {q: route[-1] for q, route in routes}
    for printer_id, route in routes:
        for target_x, target_y in route:
            if stop_requested.is_set():
                raise RuntimeError("Recovery cancelled")
            if time.monotonic() >= deadline:
                raise TimeoutError("Recovery exceeded its deadline")
            statuses, _ = get_status()
            if any(statuses[q - 1] != "I" for q in ip):
                raise RuntimeError("Recovery requires both printers idle")

            marker = "RECOVERY_" + uuid.uuid4().hex
            commands = (
                "G90\n"
                f"G1 X{target_x:.6f} Y{target_y:.6f} F{RECOVERY_FEEDRATE:.6f}\n"
                "M400\n"
                f'echo "{marker}"'
            )
            response = requests.get(
                f"http://{ip[printer_id]}/rr_gcode",
                params={"gcode": commands}, timeout=5,
            )
            response.raise_for_status()
            reply = response.json()
            if reply.get("err", 0) != 0 or "buff" not in reply:
                raise RuntimeError(f"Recovery submission failed: {reply}")

            # HTTP acceptance is not motion completion. The unique marker
            # runs AFTER M400 in this same firmware input channel.
            # This worker must be the only rr_reply reader for its HTTP client;
            # another reader can consume the marker and cause a timeout.
            while not stop_requested.is_set():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"No recovery completion marker from p{printer_id}")
                response = requests.get(
                    f"http://{ip[printer_id]}/rr_reply", timeout=5,
                )
                response.raise_for_status()
                text = response.text
                if "error" in text.lower():
                    raise RuntimeError(f"Recovery firmware error for p{printer_id}: {text}")
                if marker in [line.strip() for line in text.splitlines()]:
                    break
                stop_requested.wait(0.05)
            if stop_requested.is_set():
                raise RuntimeError("Recovery cancelled")

            # Verify in the same native coordinate convention as get_status.
            # This read is AFTER the firmware barrier, not during movement.
            _, coordinates = get_status()
            actual = tuple(map(float, coordinates[printer_id - 1][:2]))
            target = (target_x, target_y)
            if len(actual) != 2 or any(
                not math.isfinite(a) or abs(a - b) > RECOVERY_TOLERANCE
                for a, b in zip(actual, target)
            ):
                raise RuntimeError(f"p{printer_id} did not reach {target}; observed {actual}")

    # Read both positions together only after every configured leg completed.
    while not stop_requested.is_set():
        if time.monotonic() >= deadline:
            raise TimeoutError("Printers did not settle idle after recovery")
        statuses, coordinates = get_status()
        if all(statuses[q - 1] == "I" for q in ip):
            observed = {
                q: tuple(map(float, coordinates[q - 1][:2])) for q in ip
            }
            for q, actual in observed.items():
                if len(actual) != 2 or any(
                    not math.isfinite(a) or abs(a - b) > RECOVERY_TOLERANCE
                    for a, b in zip(actual, targets[q])
                ):
                    raise RuntimeError(f"Final recovery position mismatch for p{q}")
            return observed
        stop_requested.wait(0.05)
    raise RuntimeError("Recovery cancelled")


















def execution_worker(ip, execution_request_queue, execution_result_queue,
                     tracker_control_queue, stop_requested, error_queue):
    try:
        while not stop_requested.is_set():
            try:
                filename, p = execution_request_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:

                if filename == "__recover__":
                    if p is not None:
                        raise ValueError("Recovery request must be (__recover__, None)")
                    observed = recover_printers(ip, stop_requested)
                    execution_result_queue.put(("recovered", observed))
                    continue

                



                # Check connectivity before sending. get_status already has
                # its own reconnect attempts; use the helper if it still fails.
                try:
                    statuses, _ = get_status()
                except (requests.RequestException, ConnectionError):
                    if reconnect_to_ambots() is not True:
                        raise ConnectionError("Could not reconnect to AMBOTS")
                    statuses, _ = get_status()

                if statuses[p - 1] != "I":
                    raise RuntimeError(f"p{p} is no longer idle")
                if (p not in ip or not filename.endswith(".gcode")
                        or any(c in filename for c in '/\\\r\n"')):
                    raise ValueError(f"Invalid execution request: {(filename, p)}")


                print(
                    f"SEND_JOB t={time.monotonic():.6f} p={p} ip={ip[p]} "
                    f"file=0:/gcodes/sub_directory/{filename}", flush=True,
                )



                response = requests.get(
                    f"http://{ip[p]}/rr_gcode",
                    params={"gcode": f'M32 "0:/gcodes/sub_directory/{filename}"'},
                    timeout=5,
                )
                response.raise_for_status()
                reply = response.json()
                # rr_gcode normally replies with {"buff": bufferSpace}.
                # Some responses include an error code. Do not require err=0
                # when the documented successful response has no err field.
                if reply.get("err", 0) != 0 or "buff" not in reply:
                    raise RuntimeError(f"Unexpected rr_gcode response: {reply}")

                execution_result_queue.put(("sent", p))



                #so the execution_wroker is not responsible anymore for telling the tracker to watch the rpint?

                #tracker_control_queue.put(("print", p))


            except Exception:
                # Never resend M32 automatically after an uncertain response.
                # The printer might already have received it.
                stop_requested.set()
                execution_result_queue.put(("failed", p))
                #tracker_control_queue.put(("no print", p))
                raise
    except Exception:
        error_queue.put(("execution_worker", traceback.format_exc()))
        stop_requested.set()
        raise
