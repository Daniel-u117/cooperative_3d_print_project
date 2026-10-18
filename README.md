# Cooperative 3D printing

Run from PowerShell in this project folder using Python 3.12 or newer. This device
was checked with Python 3.14.8 and the package versions in [requirements.txt](requirements.txt).
Its current Python installation already has those packages and Tk for plotting.

```powershell
python simulation_main.py
```

This prepares the G-code in [ulti_maker](ulti_maker), runs a finite local candidate
simulation, and opens an animated preview. Close the plot window to exit. Default
mode makes no printer requests and does not change Wi-Fi. You do not need to run
[polygon_construction.py](polygon_construction.py) first.

To run without a window:

```powershell
python simulation_main.py --headless
```

To prepare transformed G-code and boundaries only:

```powershell
python simulation_main.py --mode prepare
```

[polygon_construction.py](polygon_construction.py) now calls this same preparation
pipeline, replacing the older duplicate implementation with different paths and
coordinate transforms.

## Setup on a fresh Python installation

Use Python 3.14 for the exact dependency versions tested here. The older Anaconda
Python 3.9 installation on this device is not supported by this project.

```powershell
py -3.14 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe simulation_main.py
```

No environment activation or PowerShell execution-policy change is needed.

## Inputs and results

Default paths are anchored to the project location, so the script also works when
launched from another working directory. Override them when needed:

```powershell
python simulation_main.py --input-dir "D:\My Gcode" --output-dir "D:\Simulation Results" --headless
```

Input files must be Cura-style `.gcode` files containing an `M107` marker followed
by XY motion forming a non-degenerate polygon. Files without that marker now fail
with a message instead of hanging. Original source files are preserved. Generated
data lives under `output/` by default:

| Location | Contents |
| --- | --- |
| `transformed/p1`, `transformed/p2` | G-code in each printer's coordinate frame |
| `boundaries_store/p1`, `boundaries_store/p2` | Sampled region boundaries |
| `processed/p1`, `processed/p2` | Beginning, middle, and end G-code variants |
| `sim_fold/p1`, `sim_fold/p2` | Candidate simulation frames |
| `order.txt` | Accepted and rejected candidate log |
| `simulation_report.json` | Local results and blocked reachable regions |

Repeated runs overwrite generated files with matching names. Files left over from
earlier input sets are not included in the current run. The output folder is ignored
by Git.

Local simulation starts both printers at native XY `(300, 0)`, considers nearest
available regions in printer order, and applies the existing sampled-boundary
collision calculation. It stops when no candidate can be accepted; blocked regions
are reported rather than retried forever. This is a sequential geometric preview,
not a reproduction of concurrent motion, extrusion timing, or live recovery.
The 13 supplied files produced 8 accepted regions and 5 blocked regions in testing.

## Connect and run on the robots

Join the robot network, then check both connections without uploading or moving:

```powershell
python simulation_main.py --mode connect
```

Once connected, run the live workflow:

```powershell
python simulation_main.py --mode printers
```

Live mode authenticates both robots, checks that they are idle, uploads the prepared
G-code, homes each robot with `G28`, and starts execution, tracking, and visualization.
It checks HTTP and controller errors before continuing. Each robot must finish homing
before the next robot starts. The program polls both robots, waits for a unique
firmware reply after `G28` and `M400`, and requires idle status before proceeding.
This uses the same command/reply approach as recovery; see the
[controller HTTP API](https://github.com/Duet3D/RepRapFirmware/wiki/HTTP-requests).
Keep other clients on this computer from consuming `rr_reply` during homing.

The default homing timeout is 180 seconds **per robot**. For slower homing:

```powershell
python simulation_main.py --mode printers --homing-timeout 300
```

Waiting progress is printed every five seconds. A timeout, controller error, or
connection failure stops startup without sending `G28` again or starting jobs.
It does not cancel a homing movement already running on a robot.

The default robot addresses are `192.168.0.15` and `192.168.0.16`. Set different
addresses using the same options for either mode:

```powershell
python simulation_main.py --mode connect --printer-1 192.168.0.15 --printer-2 192.168.0.16
python simulation_main.py --mode printers --printer-1 192.168.0.15 --printer-2 192.168.0.16
```

To let the program switch to an existing Windows Wi-Fi profile when the robots are
unreachable, add `--wifi-profile AMBOTS_5G`. Use `--wifi-interface "Wi-Fi"` if needed.
Without a supplied profile, the program uses your current network and never switches
Wi-Fi. The same connection settings are passed to the tracking, execution, and
recovery workers, including when Windows starts them in separate processes.

```powershell
python simulation_main.py --mode connect --wifi-profile AMBOTS_5G
python simulation_main.py --mode printers --wifi-profile AMBOTS_5G
```

The controller password defaults to `reprap`. For a different password, set it in
this terminal before running either command:

```powershell
$env:AMBOTS_PASSWORD = "your-controller-password"
```

These modes expect the project's existing RepRapFirmware HTTP API, including native
XY coordinates in `rr_status` under `coords.machine`. Missing telemetry or rejected
authentication produces an error. Connection-check mode does not require input files.
Hardware operation has not been tested against connected robots on this device.
Printer geometry and recovery routes remain the project's existing configuration in
[simulation_main.py](simulation_main.py), [command_script.py](util/command_script.py),
and [execution.py](util/execution.py). Local preview uses sampled boundaries and does
not validate physical collision clearance. Closing the plot stops the software workers;
it does not send an emergency stop to a robot already executing an uploaded file.

## Checks

```powershell
python -B -m unittest discover -s tests -v
```

The checks cover malformed input, blank lines and modal XY parsing, and the full
local pipeline from a different working directory with HTTP and subprocess calls
blocked. They also verify source preservation and that stale generated files are ignored.

Robot tests use mocked HTTP responses to check custom addresses, connection-only mode,
Wi-Fi opt-in, telemetry validation, upload failure handling, and worker settings.
