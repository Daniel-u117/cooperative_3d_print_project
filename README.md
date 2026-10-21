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
The operator confirmed a successful live final-cell recovery run on 2026-10-21
after the fixes documented below. Automated checks use simulated controller responses.
Printer geometry and recovery routes remain the project's existing configuration in
[simulation_main.py](simulation_main.py), [command_script.py](util/command_script.py),
and [execution.py](util/execution.py). Local preview uses sampled boundaries and does
not validate physical collision clearance. Closing the plot stops the software workers;
it does not send an emergency stop to a robot already executing an uploaded file.

### Reproduce the final-cell recovery with cells 1, 4, and 7

Select a small input set without removing files from [ulti_maker](ulti_maker):

```powershell
.\.venv\Scripts\python.exe simulation_main.py --cells 1 4 7 --initial-cells 4 7 --headless --output-dir output/recovery-preview
.\.venv\Scripts\python.exe simulation_main.py --mode printers --cells 1 4 7 --initial-cells 4 7 --output-dir output/recovery-live
```

The second command uploads, homes, and runs the robots. Only selected cells are
prepared, uploaded, and scheduled. Missing cell names fail before preparation.
`--initial-cells 4 7` reserves cell 4 as printer 1's first job and cell 7 as
printer 2's first job. Both must finish before other cells can be considered.
Initial jobs still pass the usual collision checks; a rejected initial assignment
stops the experiment instead of substituting another cell. Duplicate, unselected,
or unreachable initial cells are rejected. Omit this option for normal scheduling.

The offline preview checks geometry; it does not exercise live recovery. With these
initial assignments and the supplied geometry, it accepts cells 4 and 7 and rejects
cell 1 for both printers. Its JSON report includes each candidate's starting poses
and collision results.

A regression test runs the live scheduler with actual geometry and simulated robot
responses. It uses transformed G-code endpoints as completion positions, observes
both rejections of cell 1, completes both recovery routes, and then assigns
`1_m.gcode` to printer 1. In a live run, the first eligible free-printer event whose
candidate passes collision checks determines the assignment; cell 1 is not pinned
to either printer. Real telemetry and event order can change the outcome.

```powershell
python -B -m unittest discover -s tests -p test_initial_assignments.py -v
```

When all remaining candidates are blocked and neither printer has a job in flight,
the scheduler requests recovery. The configured routes move printer 1 and then
printer 2 to native XY `(275, 25)`. Returning printer 1 alone does not release a job:
both routes must complete before the scheduler updates both observed positions and
footprints, retries printers with remaining work, and checks collisions again.

Recovery now sends one complete G-code line at a time after checking available
firmware buffer bytes using an empty `rr_gcode` request. Job-start commands use the
same check. Increasing a Python queue size does not change this firmware buffer.
An uncertain motion or job-start response is never automatically resent.

Recovery sends `G90` and `G53 G1` to command machine coordinates. Both configured
recovery targets are currently `(275, 25)`; verify each robot's route and clearance.
Completion requires the unique reply after `M400`, both robots idle, and observed
XY within `RECOVERY_TOLERANCE` (0.25 mm per axis).

Replies are collected during command submission and completion polling, and a
received marker is retained while waiting for idle telemetry. If both robots are
idle and the moving robot stays at its target for one second without a marker,
recovery sends **one fresh `M400` + `echo` handshake**. It never repeats the movement.
The retry has a five-second acknowledgement budget within the existing 180-second
budget for both routes. A missing retry reply stops recovery; idle alone never
releases the next robot or job.

Watch the console for `RECOVERY_START`, `RECOVERY_WAIT`, `RECOVERY_ACK_RETRY`, and
`RECOVERY_DONE`. Nonempty replies appear as `RECOVERY_REPLY` with the expected marker
and raw response. Wait messages include both robot statuses and the reason for
waiting. Status/reply request failures include the last observed position, target,
marker state, retry count, and last nonempty reply. `BUFFER_WAIT` reports a full
command buffer. Keep other clients from consuming `rr_reply` or sending G-code
during recovery. Increasing position tolerance cannot repair a missing reply.

The run's `order.txt` records fixed initial assignments, the end of the initial phase,
each candidate's starting positions and collision result, execution choices, recovery completion, and
scheduler failures. If a submitted job never confirms extrusion while moving,
tracking now reports an error after 300 seconds (`WATCH_TIMEOUT` in
[tracking.py](util/tracking.py)) instead of waiting indefinitely. Error shutdown
stops the software workers; it does not cancel a command already on a robot.

### Confirmed recovery fix (2026-10-21)

The operator reported that the updated live recovery run succeeded after testing
cells 1, 4, and 7 with printer 1 assigned cell 4 and printer 2 assigned cell 7 first.
The changes address two distinct failures observed during that experiment:

| Observed failure | Change and resulting behavior |
| --- | --- |
| Printer 1 finished at `(298.635, 3.922)` when the target was `(300, 0)`, outside the 0.25 mm per-axis tolerance. | Both configured parking targets were changed to `(275, 25)`. Recovery retains `G90` and adds `G53` on each `G1` line so commands and verification use machine coordinates. The subsequent log confirmed printer 1 idle at exactly `(275, 25)`. |
| Printer 1 remained idle at the new target while `marker=False` repeated until the 180-second deadline. | Replies are now collected during submission as well as polling. A recognized marker is retained even while a robot is busy. After one second with both robots idle and the moving robot at its target, one fresh `M400` + unique `echo` handshake is allowed, with a five-second acknowledgement budget. The movement is not resent. |
| The final HTTP timeout obscured the stalled condition, and the scheduler reported only `('failed', None)`. | Failures include the current phase, expected marker, marker state, both statuses, last position, target, retry count, and last nonempty reply. The execution worker forwards the recovery error to the scheduler and prints `RECOVERY_FAILED`. |

The original missing reply's precise cause was not established by the logs; the
successful run confirms the updated workflow worked for this experiment. It does
not establish that changing coordinates alone repaired reply delivery. Recovery
still requires acknowledgement, both robots idle, and the configured position
tolerance before proceeding. A late marker from the original handshake cannot
confirm the fresh handshake. Both recovery routes finish before the scheduler
refreshes positions and footprints and checks the remaining cell again.

The regression tests in [test_recovery.py](tests/test_recovery.py) cover dropped
initial replies, an immediate retry reply, stale markers, permanently missing
markers, a busy peer, incorrect positions, HTTP failures, and uncertain motion
submissions. [test_initial_assignments.py](tests/test_initial_assignments.py) runs
the cells 4/7 then cell 1 sequence using actual project geometry and simulated
controllers, both with normal replies and with each initial recovery reply dropped.

## Changes from the original repository

This version builds on
[Nurali14941/cooperative_3d_print_project](https://github.com/Nurali14941/cooperative_3d_print_project).
The comparison below uses its `main` at
[`d648c4d` (New version, works on printers)](https://github.com/Nurali14941/cooperative_3d_print_project/commit/d648c4dc0dcc42da6a172dec42a0add5f5fab89f),
verified on 2026-10-21. Later upstream changes are outside this comparison.

| Area | Changes in this version |
| --- | --- |
| Portable paths and setup | Replaced machine-specific input/output paths with [project_paths.py](project_paths.py), project-local [ulti_maker](ulti_maker) inputs, and an overridable output directory. Added dependency pins, virtual-environment instructions, and generated-output ignore rules. |
| Local execution | Added explicit `simulate`, `prepare`, `connect`, and `printers` modes. The default is an offline, finite geometric preview with an optional headless run and a JSON report. Only explicit live mode uploads, homes, and runs the robots. |
| Preparation and input handling | Unified preparation through [simulation_main.py](simulation_main.py), with [polygon_construction.py](polygon_construction.py) as a compatibility entry point. Added clear failures for missing/invalid inputs and missing `M107`, handled blank lines and modal XY coordinates, preserved source inputs, and excluded stale generated files from current runs. |
| Robot connections | Added configurable robot addresses, an environment-variable password override, connection-only checks, validated native telemetry, upload error checks, and an optional saved Wi-Fi profile. The same connection settings are passed to spawned workers. |
| Homing completion | Replaced the original fixed 45-second sleeps with sequential `G28`, `M400`, and a unique completion reply, plus idle checks, bounded requests, progress output, and a configurable per-robot timeout. |
| Repeatable scheduling experiments | Added `--cells` and `--initial-cells`. Initial cells are reserved per printer and checked for validity, reachability, and collisions. Both seeded jobs must finish before the remaining cells are considered. Candidate poses and collision decisions are logged. |
| Recovery and command delivery | Added buffer-capacity checks and complete-line submission for recovery and job starts. Updated the parking targets and machine-coordinate commands, retained received markers, required both robots idle, added the bounded acknowledgement retry, and improved failure diagnostics as described above. |
| Tracking and validation | Added bounded HTTP requests and a 300-second limit for confirming moving extrusion. Added automated input, offline, connection, homing, tracking, scheduling, and recovery regressions. |

The original project already supplied the robot geometry, kinematics, collision
checking, live worker/queue structure, moving-extrusion detection, and blocked-work
recovery scheduling. This version retains those foundations. The local preview is
sequential and does not model concurrent hardware timing; configured recovery paths
still require physical clearance, and live cell assignment depends on telemetry,
event order, and collision checks.

## Checks

```powershell
python -B -m unittest discover -s tests -v
```

The checks cover malformed input, blank lines and modal XY parsing, and the full
local pipeline from a different working directory with HTTP and subprocess calls
blocked. They also verify source preservation and that stale generated files are ignored.

Robot tests use mocked HTTP responses to check custom addresses, connection-only mode,
Wi-Fi opt-in, telemetry validation, upload failure handling, and worker settings.
