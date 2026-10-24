# Cooperative 3D printing

Run from PowerShell in this project folder using Python 3.12 or newer. This device
was checked with Python 3.14.8 and the package versions in [requirements.txt](requirements.txt).
Its current Python installation already has those packages and Tk for plotting.

```powershell
python simulation_main.py
```

This prepares the G-code in [input/ulti_maker](input/ulti_maker), runs a finite local candidate
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

Input folders can contain Cura-style `.gcode` files, ordered XY `.txt` cell paths,
or both. Existing G-code must contain an `M107` marker followed by XY motion forming
a non-degenerate polygon. Text paths are converted automatically before the usual
preparation steps (see below). Files with the same cell stem, such as `01.txt` and
`01.gcode`, are rejected to avoid ambiguous assignments and output overwrites.
Original source files are preserved. Generated data lives under `output/` by default:

| Location | Contents |
| --- | --- |
| `converted/` | G-code converted from selected text paths, in the original layout frame |
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

## Convert text cell paths

The supplied [input/cells_spiral](input/cells_spiral) folder contains 20 ordered
XY paths. Each `.txt` file is one continuous, single-layer cell in millimetres,
in the same layout coordinate frame used by the original G-code inputs:

```text
147.93469 116.25000
140.57487 129.67491
128.14112 129.45573
```

Every data row must contain exactly two finite numbers separated by whitespace.
Blank lines and comments beginning with `#` or `;` are accepted. At least three
distinct points spanning a nonzero area are required for the existing geometry
pipeline. Point order is preserved: every consecutive segment is extruded, and no
closing segment, infill, skirt, or additional layer is invented. Blank lines do not
split the file into separate paths. Units and the coordinate frame are not inferred.

Prepare converted G-code, per-printer transforms, boundaries, and job variants:

```powershell
python simulation_main.py --mode prepare --input-dir input/cells_spiral --output-dir output/spiral
```

Run the local geometric preview with the same input:

```powershell
python simulation_main.py --input-dir input/cells_spiral --output-dir output/spiral-preview --headless
```

Select files by their exact stems, including leading zeros:

```powershell
python simulation_main.py --mode prepare --input-dir input/cells_spiral --cells 01 04 07 --output-dir output/spiral-selected
```

`--initial-cells` also accepts text-file stems and maps them to the converted job
names, for example `04` becomes `04.gcode`. Existing `.gcode` inputs continue through
the same pipeline without conversion. The default input directory is now
[input/ulti_maker](input/ulti_maker), matching the relocated source files.

The text files supply no printing parameters. Edit
[config/cell_print.toml](config/cell_print.toml) to set them; the program rereads this
file each time it converts text inputs. The default file is located relative to the
project, so it also works when launched from another working directory. Rerun
preparation or live mode after an edit to regenerate the G-code. Existing `.gcode`
inputs are not changed by these settings.

```toml
[cell_print]
filament_diameter = 2.85
line_width = 0.44
layer_height = 0.2
nozzle_temperature = 240.0
print_speed = 20.0
travel_speed = 60.0
flow_multiplier = 1.0
```

Distances are in mm, speeds in mm/s, and temperature in degrees Celsius.
`flow_multiplier = 1.0` means 100% extrusion. These are editable starting values;
match them to the installed filament, nozzle, and material before a live run.
All values must be finite positive numbers, and layer height must be below the
pipeline's fixed Z10 travel clearance. Missing keys, unknown keys, invalid types,
and malformed or missing configuration files stop text conversion before output
or robot access. The file is not needed for G-code-only runs or connection checks.

To keep several profiles, copy the file and select a different one:

```powershell
python simulation_main.py --mode prepare --input-dir input/cells_spiral --output-dir output/spiral --cell-config config/my_material.toml
```

For a temporary override, the existing `--txt-filament-diameter`, `--txt-line-width`,
`--txt-layer-height`, `--txt-nozzle-temperature`, `--txt-print-speed`,
`--txt-travel-speed`, and `--txt-flow-multiplier` options take precedence over values
in the selected file. They do not modify the file. For example:

```powershell
python simulation_main.py --mode prepare --input-dir input/cells_spiral --output-dir output/spiral --txt-nozzle-temperature 210
```

Extrusion is calculated using a rectangular bead approximation:
`E increment = segment length * line width * layer height * flow / (pi * (filament diameter / 2)^2)`.
Speeds are converted from mm/s to G-code feed rates in mm/min. Each cell establishes
millimetres, absolute XYZ and absolute extrusion, resets E to zero, and waits for
its nozzle temperature. It travels at Z10 to the first XY point, lowers to the
layer height, then extrudes along the remaining points. It uses the existing
`M107`, `;LAYER:0`, and `;TIME_ELAPSED` markers so the beginning/middle/end split
works. The time comment estimates extrusion-path time only. Setup remains in each
job variant, including middle jobs, to avoid inheriting another cell's E position.
Beginning and middle jobs finish at Z10; end jobs also wait for motion and turn off
the nozzle. No bed heating, automatic priming/retraction, or per-cell homing is added.

Converted files in `output/spiral/converted/` are in the common layout frame.
The live workflow uses the transformed and processed files under `processed/p1`
and `processed/p2`. To run after verifying the profile, use the same conversion
options with `--mode printers` instead of `--mode prepare`; that command uploads,
homes, and executes on the robots. Conversion and automated validation have been
tested on all 20 supplied paths; physical printing of these new paths has not been
tested by the assistant.

The reusable converter is [convert_cell_path](util/cell_paths.py), with settings
loaded from the same configuration file unless an explicit `CellPrintSettings`
object is supplied. Automated conversion tests are in
[test_cell_paths.py](tests/test_cell_paths.py).

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

Select a small input set without removing files from [input/ulti_maker](input/ulti_maker):

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
scheduler failures. Tracking allows 300 seconds to confirm moving extrusion or
verified completion (`WATCH_TIMEOUT` in [tracking.py](util/tracking.py)). Error
shutdown stops the software workers; it does not cancel a command already on a robot.

### Job tracking and short-job completion (2026-10-23)

The spiral experiment exposed a separate tracking stall: printer 1 was idle at
cell 12's final XY position, but the tracker was still waiting for consecutive
samples of moving extrusion. The scheduler was waiting for that acknowledgement
and still counted the job as in flight, so the recovery condition was never reached.

Tracking now polls each printer once per pass with independent job state and
timeouts. Waiting for printer 1's start or completion does not prevent polling
printer 2, consuming control messages, or updating visualization. HTTP requests
remain bounded; there is no inner loop waiting for one printer to start extruding.

Every prepared printer-specific beginning/middle/end job receives a fresh token.
The job writes `global.ambotJobStarted` at its start and writes the same token to
`global.ambotJobDone` only after its final `M400` motion barrier. The tracker reads
the global variables through `rr_model` and then reads fresh machine status.
These variables remain readable after completion and do not consume `rr_reply`,
which remains available to homing and recovery. This requires RepRapFirmware
support for [G-code global variables and meta commands](https://docs.duet3d.com/User_manual/Reference/Gcode_meta_commands)
and the `global` object-model query. Tokens differ between preparation runs,
printers, and file variants, so a previous job's token cannot complete the new job.

Normal moving-extrusion confirmation still releases the scheduler's start wait.
If that sampling window is missed, matching start/completion tokens plus idle
status instead produce a `Completed` acknowledgement. Final coordinates and
footprints are published before that acknowledgement and the subsequent free
notification. The scheduler can then remove the job from its in-flight set,
consider remaining cells, and invoke recovery if the normal collision-blocked
conditions hold. An idle robot alone or a stale completion token is insufficient.
A full event queue is retried without sending duplicate acknowledgements.

A started job that stays idle without its matching completion token for 10 seconds
stops with a diagnostic instead of being assumed successful; it may have failed or
been cancelled. A job with no moving-extrusion or completion confirmation stops
after 300 seconds. Polling does not restart jobs or automatically resume paused
robots. No completion is inferred from the final XY position or E value alone.

Watch for `JOB_WATCH`, `JOB_SAMPLE`, and `JOB_COMPLETE`. Samples print when status
or token state changes and otherwise at five-second intervals. `JOB_COMPLETE`
includes `printing_sample_missed=True` when the completion path released a missed
start acknowledgement. `PRINT_ACK` and the order log identify whether the scheduler
received `Is printing` or `Completed`.

**Regenerate and upload the jobs through the updated live workflow.** Old uploaded
files do not contain completion tokens. The normal command performs preparation
and upload automatically before homing and execution:

```powershell
python simulation_main.py --mode printers --input-dir input/cells_spiral --output-dir output/spiral
```

The fix is validated with simulated controller responses, including missed printing
samples, stale/missing tokens, a busy peer, queue backpressure, and the final-cell
recovery sequence. It has not yet been validated on physical robots.

### Automatic return after all jobs finish

In live printer mode, once no available jobs remain and tracking has confirmed
completion of every dispatched job, both printers automatically follow
`RECOVERY_PLAN` in [execution.py](util/execution.py). This uses the same configured
waypoints, order, feedrate, timeout, and arrival checks as deadlock recovery.
The current plan returns printer 1 and then printer 2 to machine XY `(275, 25)`.
Editing that plan changes both recovery and the final return destinations.

The scheduler keeps the workers running until both routes have completed,
refreshes the stored positions and footprints, and then ends the run. The console
shows `FINAL_RETURN_START`, the existing per-printer `RECOVERY_*` progress messages,
and `FINAL_RETURN_DONE` only after verified arrival. A route failure produces
`FINAL_RETURN_FAILED` and stops the run with an error. [order.txt](output/order.txt)
in the selected output directory records the final return request and result.
Cancellation or an earlier job error does not initiate this return. Offline
simulation and prepare mode do not move the printers.

Automated tests cover completion ordering, changed waypoints, verified arrival,
and failure/timeout handling. The final return still needs a physical-printer run.

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
| Portable paths and setup | Replaced machine-specific input/output paths with [project_paths.py](project_paths.py), project-local [input/ulti_maker](input/ulti_maker) inputs, and an overridable output directory. Added dependency pins, virtual-environment instructions, and generated-output ignore rules. |
| Local execution | Added explicit `simulate`, `prepare`, `connect`, and `printers` modes. The default is an offline, finite geometric preview with an optional headless run and a JSON report. Only explicit live mode uploads, homes, and runs the robots. |
| Preparation and input handling | Unified preparation through [simulation_main.py](simulation_main.py), with [polygon_construction.py](polygon_construction.py) as a compatibility entry point. Added clear failures for missing/invalid inputs and missing `M107`, handled blank lines and modal XY coordinates, preserved source inputs, and excluded stale generated files from current runs. |
| Robot connections | Added configurable robot addresses, an environment-variable password override, connection-only checks, validated native telemetry, upload error checks, and an optional saved Wi-Fi profile. The same connection settings are passed to spawned workers. |
| Homing completion | Replaced the original fixed 45-second sleeps with sequential `G28`, `M400`, and a unique completion reply, plus idle checks, bounded requests, progress output, and a configurable per-robot timeout. |
| Repeatable scheduling experiments | Added `--cells` and `--initial-cells`. Initial cells are reserved per printer and checked for validity, reachability, and collisions. Both seeded jobs must finish before the remaining cells are considered. Candidate poses and collision decisions are logged. |
| Recovery and command delivery | Added buffer-capacity checks and complete-line submission for recovery and job starts. Updated the parking targets and machine-coordinate commands, retained received markers, required both robots idle, added the bounded acknowledgement retry, and improved failure diagnostics as described above. |
| Tracking and validation | Added bounded HTTP requests, independent per-printer polling, persistent job-completion tokens, and a 300-second limit for confirming moving extrusion or completion. Added automated input, offline, connection, homing, tracking, scheduling, and recovery regressions. |

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
