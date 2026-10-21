import queue, time, uuid, traceback
from pathlib import Path
import subprocess
import requests
import re
import socket
import time

from .network_utils import RobotConnection, get_status, read_status, reconnect_to_ambots, tcp_reachable, current_wifi, run_netsh




# Native X/Y waypoints, executed one printer and one leg at a time.
# These configured straight-line routes must have physical clearance.
# This helper executes the routes; it does not plan collision-free recovery.
RECOVERY_PLAN = (
    (1, ((275.0, 25.0),)),
    (2, ((275.0, 25.0),)),
)
RECOVERY_TIMEOUT = 180.0
RECOVERY_FEEDRATE = 1500.0
RECOVERY_TOLERANCE = 0.25
RECOVERY_ACK_GRACE = 1.0
RECOVERY_ACK_TIMEOUT = 5.0


def send_gcode(ip, printer_id, commands, stop_requested, deadline, *, collect_reply=None):
    """Send complete lines within firmware capacity, never resend a motion.

    rr_gcode's buff field is remaining capacity, not a completion acknowledgement.
    Empty submissions query capacity without executing anything. This worker must
    be the sole G-code sender for the controller during submission.
    """
    def request(command):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"p{printer_id}: timed out waiting for G-code buffer space")
        if stop_requested.is_set():
            raise RuntimeError("G-code submission cancelled")
        response = requests.get(
            f"http://{ip[printer_id]}/rr_gcode",
            params={"gcode": command}, timeout=min(5.0, remaining),
        )
        response.raise_for_status()
        reply = response.json()
        if (not isinstance(reply, dict) or reply.get("err", 0) != 0
                or type(reply.get("buff")) is not int or reply["buff"] < 0):
            raise RuntimeError(f"p{printer_id}: invalid rr_gcode response: {reply}")
        if collect_reply is not None:
            collect_reply()
        return reply["buff"]

    for line in commands.splitlines():
        if not line.strip():
            continue
        command = line + "\n"
        # Include a byte for firmware's command terminator as well as the LF.
        required = len(command.encode("utf-8")) + 1
        available = request("")
        reported_wait = False
        while available < required:
            if not reported_wait:
                print(f"BUFFER_WAIT p={printer_id} required={required} available={available}", flush=True)
                reported_wait = True
            stop_requested.wait(0.05)
            available = request("")
        # An HTTP failure here is uncertain: do not retry this command.
        request(command)
 
def recover_printers(ip, stop_requested, connection=None):
    """Run configured routes serially; return observed final raw X/Y positions."""
    import math
    connection = connection or RobotConnection(ip[1], ip[2])

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
    printer_id, target = None, None
    marker, marker_seen, last_reply = None, False, ""
    statuses, actual, acknowledgement_attempt = [], None, 0
    acknowledgement_deadline = None
    phase = "initial status"

    def diagnostic(reason):
        return (f"Recovery p={printer_id} {reason}: phase={phase} "
                f"marker={marker_seen} expected={marker!r} "
                f"statuses={dict(zip(sorted(ip), statuses))} "
                f"observed={actual} target={target} "
                f"ack_attempt={acknowledgement_attempt} last_reply={last_reply!r}")

    def remaining_time():
        active_deadline = deadline
        if acknowledgement_deadline is not None and not marker_seen:
            active_deadline = min(active_deadline, acknowledgement_deadline)
        remaining = active_deadline - time.monotonic()
        if remaining <= 0:
            reason = ("completion acknowledgement missing after retry"
                      if acknowledgement_deadline is not None and not marker_seen
                      else "deadline expired")
            raise TimeoutError(diagnostic(reason))
        return remaining

    def collect_reply():
        nonlocal marker_seen, last_reply
        try:
            response = requests.get(
                f"http://{ip[printer_id]}/rr_reply", timeout=min(5.0, remaining_time()),
            )
            response.raise_for_status()
        except requests.RequestException as exc:
            raise RuntimeError(diagnostic(f"reply request failed ({exc})")) from exc
        text = response.text
        if text.strip():
            last_reply = text
            print(f"RECOVERY_REPLY p={printer_id} expected={marker!r} raw={text!r}", flush=True)
        if "error" in text.lower():
            raise RuntimeError(diagnostic("firmware error"))
        # Match the complete echoed line, never a substring of another marker
        # or a command echo. Latch it even if telemetry is still busy.
        marker_seen |= marker in [line.strip() for line in text.splitlines()]

    def snapshot():
        # Do not reconnect/retry beyond the recovery deadline.
        try:
            return read_status(connection, timeout=min(5.0, remaining_time() / len(ip)))
        except requests.RequestException as exc:
            raise RuntimeError(diagnostic(f"status request failed ({exc})")) from exc

    targets = {q: route[-1] for q, route in routes}
    for printer_id, route in routes:
        for target_x, target_y in route:
            target = (target_x, target_y)
            marker, marker_seen, last_reply = None, False, ""
            actual, acknowledgement_attempt = None, 0
            acknowledgement_deadline = None
            phase = "initial status"
            if stop_requested.is_set():
                raise RuntimeError("Recovery cancelled")
            if time.monotonic() >= deadline:
                raise TimeoutError("Recovery exceeded its deadline")
            statuses, _ = snapshot()
            if any(statuses[q - 1] != "I" for q in ip):
                raise RuntimeError("Recovery requires both printers idle")

            marker = "RECOVERY_" + uuid.uuid4().hex
            commands = (
                "G90\n"
                f"G53 G1 X{target_x:.6f} Y{target_y:.6f} F{RECOVERY_FEEDRATE:.6f}\n"
                "M400\n"
                f'echo "{marker}"'
            )
            print(f"RECOVERY_START p={printer_id} target={target} expected={marker!r}", flush=True)
            phase = "command submission"
            send_gcode(ip, printer_id, commands, stop_requested, deadline,
                       collect_reply=collect_reply)

            # HTTP acceptance is not motion completion. The unique marker
            # runs AFTER M400 in this same firmware input channel.
            # This worker must be the only rr_reply reader for its HTTP client;
            # another reader can consume the marker and cause a timeout.
            idle_at_target_since = None
            acknowledgement_deadline = None
            next_report = 0.0
            while not stop_requested.is_set():
                phase = "completion wait"
                remaining_time()
                collect_reply()
                statuses, coordinates = snapshot()
                actual = tuple(map(float, coordinates[printer_id - 1][:2]))
                all_idle = all(statuses[q - 1] == "I" for q in ip)
                at_target = all(math.isfinite(a) and abs(a - b) <= RECOVERY_TOLERANCE
                                for a, b in zip(actual, target))
                if marker_seen and all_idle:
                    break
                now = time.monotonic()
                if all_idle and at_target and not marker_seen:
                    if idle_at_target_since is None:
                        idle_at_target_since = now
                    if (acknowledgement_attempt == 0
                            and now - idle_at_target_since >= RECOVERY_ACK_GRACE):
                        # Only repeat the barrier and echo, never the motion.
                        # A fresh token prevents a late old reply confirming it.
                        acknowledgement_attempt = 1
                        marker = "RECOVERY_" + uuid.uuid4().hex
                        print(f"RECOVERY_ACK_RETRY p={printer_id} reason=idle_at_target_without_marker "
                              f"expected={marker!r} motion_resent=False", flush=True)
                        phase = "acknowledgement submission"
                        acknowledgement_deadline = min(deadline, now + RECOVERY_ACK_TIMEOUT)
                        try:
                            send_gcode(ip, printer_id, f'M400\necho "{marker}"', stop_requested,
                                       acknowledgement_deadline, collect_reply=collect_reply)
                        except (requests.RequestException, TimeoutError) as exc:
                            raise TimeoutError(diagnostic(f"acknowledgement submission failed ({exc})")) from exc
                        # Take a fresh status snapshot after the handshake.
                        continue
                else:
                    idle_at_target_since = None
                if (acknowledgement_deadline is not None and not marker_seen
                        and now >= acknowledgement_deadline):
                    raise TimeoutError(diagnostic("completion acknowledgement missing after retry"))
                if time.monotonic() >= next_report:
                    reason = "waiting_for_idle" if marker_seen else "waiting_for_marker"
                    print(f"RECOVERY_WAIT p={printer_id} marker={marker_seen} "
                          f"statuses={dict(zip(sorted(ip), statuses))} observed={actual} "
                          f"target={target} reason={reason} ack_attempt={acknowledgement_attempt}", flush=True)
                    next_report = time.monotonic() + 5.0
                stop_requested.wait(0.05)
            if stop_requested.is_set():
                raise RuntimeError("Recovery cancelled")

            # Verify in the same native coordinate convention as get_status.
            # This read is AFTER the firmware barrier, not during movement.
            target = (target_x, target_y)
            if len(actual) != 2 or any(
                not math.isfinite(a) or abs(a - b) > RECOVERY_TOLERANCE
                for a, b in zip(actual, target)
            ):
                raise RuntimeError(f"p{printer_id} did not reach {target}; observed {actual}")
            print(f"RECOVERY_DONE p={printer_id} observed={actual}", flush=True)

    # Read both positions together only after every configured leg completed.
    while not stop_requested.is_set():
        if time.monotonic() >= deadline:
            raise TimeoutError("Printers did not settle idle after recovery")
        statuses, coordinates = snapshot()
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
                     tracker_control_queue, stop_requested, error_queue, connection=None):
    connection = connection or RobotConnection(ip[1], ip[2])
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
                    observed = recover_printers(ip, stop_requested, connection=connection)
                    execution_result_queue.put(("recovered", observed))
                    continue

                



                # Check connectivity before sending. get_status already has
                # its own reconnect attempts; use the helper if it still fails.
                try:
                    statuses, _ = get_status(connection=connection)
                except (requests.RequestException, ConnectionError):
                    if reconnect_to_ambots(connection=connection) is not True:
                        raise ConnectionError("Could not reconnect to AMBOTS")
                    statuses, _ = get_status(connection=connection)

                if statuses[p - 1] != "I":
                    raise RuntimeError(f"p{p} is no longer idle")
                if (p not in ip or not filename.endswith(".gcode")
                        or any(c in filename for c in '/\\\r\n"')):
                    raise ValueError(f"Invalid execution request: {(filename, p)}")


                print(
                    f"SEND_JOB t={time.monotonic():.6f} p={p} ip={ip[p]} "
                    f"file=0:/gcodes/sub_directory/{filename}", flush=True,
                )



                send_gcode(
                    ip, p, f'M32 "0:/gcodes/sub_directory/{filename}"',
                    stop_requested, time.monotonic() + 30.0,
                )

                execution_result_queue.put(("sent", p))



                #so the execution_wroker is not responsible anymore for telling the tracker to watch the rpint?

                #tracker_control_queue.put(("print", p))


            except Exception as exc:
                # Never resend M32 automatically after an uncertain response.
                # The printer might already have received it.
                stop_requested.set()
                failure = p
                if filename == "__recover__":
                    failure = str(exc)
                    print(f"RECOVERY_FAILED {failure}", flush=True)
                execution_result_queue.put(("failed", failure))
                #tracker_control_queue.put(("no print", p))
                raise
    except Exception:
        error_queue.put(("execution_worker", traceback.format_exc()))
        stop_requested.set()
        raise
