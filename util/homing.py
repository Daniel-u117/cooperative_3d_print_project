"""Sequential homing with firmware completion acknowledgement."""
import math
import time
import uuid

import requests

from util.network_utils import read_status


def home_robots(connection, timeout=180.0, poll_interval=0.25):
    """Home each robot once; require its completion marker and idle status.

    Run before starting workers: this must be the only rr_reply consumer for
    this HTTP client. A failed/uncertain request is never retried with G28.
    """
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Homing timeout must be a finite positive number of seconds")
    if not math.isfinite(poll_interval) or poll_interval <= 0:
        raise ValueError("Homing poll interval must be finite and positive")

    for p, host in connection.ip.items():
        started = time.monotonic()
        deadline = started + timeout
        marker_seen = False
        last_status = "unknown"

        def remaining():
            seconds = deadline - time.monotonic()
            if seconds <= 0:
                raise TimeoutError(
                    f"Homing robot {p} ({host}) exceeded {timeout:g}s; "
                    f"last status={last_status}, completion marker received={marker_seen}. "
                    "Live execution was not started. The robot may still be moving; "
                    "G28 was not resent."
                )
            return seconds

        def status_snapshot():
            # Poll both robots to keep both HTTP sessions alive during a long home.
            result = read_status(connection, timeout=min(5.0, remaining() / 2))
            remaining()
            return result

        try:
            statuses, _ = status_snapshot()
            if any(status != "I" for status in statuses):
                raise RuntimeError(f"Both robots must be idle before homing robot {p}: {statuses}")

            marker = "HOMING_" + uuid.uuid4().hex
            # The marker follows G28 and the motion barrier on the same channel.
            # An HTTP acknowledgement or an initial idle poll is insufficient.
            commands = f'G28\nM400\necho "{marker}"'
            print(f"Homing robot {p} ({host}); timeout={timeout:g}s...", flush=True)
            response = requests.get(
                f"http://{host}/rr_gcode", params={"gcode": commands},
                timeout=min(5.0, remaining()),
            )
            response.raise_for_status()
            reply = response.json()
            if not isinstance(reply, dict) or reply.get("err", 0) != 0 or "buff" not in reply:
                raise RuntimeError(f"Robot {p} did not acknowledge G28")

            next_report = started
            while True:
                response = requests.get(f"http://{host}/rr_reply", timeout=min(5.0, remaining()))
                response.raise_for_status()
                remaining()
                reply_text = response.text
                if "error" in reply_text.lower():
                    raise RuntimeError(f"Homing firmware error from robot {p}: {reply_text.strip()}")
                marker_seen |= marker in [line.strip() for line in reply_text.splitlines()]

                statuses, coordinates = status_snapshot()
                last_status = statuses[p - 1]
                if marker_seen and last_status == "I":
                    if any(status != "I" for status in statuses):
                        raise RuntimeError(f"Another robot became busy during homing: {statuses}")
                    print(f"Robot {p} homing complete after {time.monotonic() - started:.1f}s; "
                          f"machine XY={coordinates[p - 1][:2]}", flush=True)
                    break
                if time.monotonic() >= next_report:
                    print(f"Waiting for robot {p}: status={last_status}, "
                          f"completion marker received={marker_seen}, "
                          f"elapsed={time.monotonic() - started:.1f}s", flush=True)
                    next_report = time.monotonic() + 5
                time.sleep(min(poll_interval, remaining()))
        except requests.RequestException as error:
            raise ConnectionError(
                f"Could not confirm homing for robot {p} ({host}); "
                "live execution was not started. The robot may still be moving; G28 was not resent."
            ) from error
