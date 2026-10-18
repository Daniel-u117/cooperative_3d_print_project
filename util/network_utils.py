"""Connection and telemetry helpers shared by the main process and workers."""
from dataclasses import dataclass, field
import math
import re
import socket
import subprocess
import time

import requests


@dataclass(frozen=True)
class RobotConnection:
    printer_1: str = "192.168.0.15"
    printer_2: str = "192.168.0.16"
    password: str = field(default="reprap", repr=False)
    wifi_profile: str | None = None
    wifi_interface: str = "Wi-Fi"

    def __post_init__(self):
        for host in self.ip.values():
            if not re.fullmatch(r"[A-Za-z0-9.-]+(?::[0-9]{1,5})?", host):
                raise ValueError("Robot addresses must be hostnames or IPv4 addresses, optionally with a port")
        if self.printer_1.lower() == self.printer_2.lower():
            raise ValueError("The two robots must have different addresses")

    @property
    def ip(self):
        return {1: self.printer_1, 2: self.printer_2}


def connect_robots(connection):
    """Authenticate both controllers; never upload files or issue movement."""
    for p, host in connection.ip.items():
        try:
            response = requests.get(f"http://{host}/rr_connect",
                                    params={"password": connection.password}, timeout=5)
            response.raise_for_status()
            reply = response.json()
        except requests.RequestException:
            raise ConnectionError(f"Cannot connect to robot {p} at {host}") from None
        if not isinstance(reply, dict) or reply.get("err") != 0:
            raise ConnectionError(f"Robot {p} at {host} rejected the connection; check its password/session availability")


def read_status(connection, *, timeout=5):
    statuses, coordinates = [], []
    for p, host in connection.ip.items():
        response = requests.get(f"http://{host}/rr_status", params={"type": 0}, timeout=timeout)
        response.raise_for_status()
        data = response.json()
        try:
            status = data["status"]
            coords = data["coords"]["machine"]
            valid = isinstance(status, str) and bool(status) and len(coords) >= 2
            valid = valid and all(math.isfinite(float(value)) for value in coords[:2])
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            raise ValueError(f"Robot {p} at {host} did not provide status and native coords.machine XY telemetry")
        statuses.append(status)
        coordinates.append(coords)
    return [statuses, coordinates]


def get_status(retries=5, connection=None):
    connection = connection or RobotConnection()
    for attempt in range(retries):
        try:
            return read_status(connection)
        except requests.RequestException:
            if attempt + 1 == retries:
                break
            reconnect_to_ambots(connection=connection)
    raise ConnectionError(f"Could not obtain status from both robots after {retries} attempts")


def run_netsh(*args, timeout=15):
    result = subprocess.run(
        ["netsh", "wlan", *args],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
        timeout=timeout
    )
    return result


def current_wifi():
    result = run_netsh("show", "interfaces")
    text = result.stdout

    state_match = re.search(r"^\s*State\s*:\s*(.+)$", text, re.MULTILINE)
    ssid_match = re.search(r"^\s*SSID\s*:\s*(.+)$", text, re.MULTILINE)

    state = state_match.group(1).strip() if state_match else None
    ssid = ssid_match.group(1).strip() if ssid_match else None

    return state, ssid, text





def tcp_reachable(ip, port, timeout=2):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False



def reconnect_to_ambots(max_wait=45, connection=None):
    connection = connection or RobotConnection()
    try:
        connect_robots(connection)
        return True
    except ConnectionError:
        if not connection.wifi_profile:
            raise

    # Change Wi-Fi only when the user explicitly supplies a saved profile.
    result = run_netsh("connect", f"name={connection.wifi_profile}",
                       f"interface={connection.wifi_interface}")
    if result.returncode:
        raise ConnectionError(f"Could not connect using Wi-Fi profile {connection.wifi_profile}: {result.stdout.strip()} {result.stderr.strip()}")
    deadline = time.monotonic() + max_wait
    while time.monotonic() < deadline:
        try:
            connect_robots(connection)
            return True
        except ConnectionError:
            time.sleep(min(2, max(0, deadline - time.monotonic())))
    raise ConnectionError(f"Could not reach both robots using Wi-Fi profile {connection.wifi_profile}")


def check_robot_connection(connection):
    reconnect_to_ambots(connection=connection)
    statuses, coordinates = get_status(retries=1, connection=connection)
    for p, host in connection.ip.items():
        print(f"Robot {p} ({host}): status={statuses[p - 1]}, machine XY={coordinates[p - 1][:2]}")
    return statuses, coordinates
