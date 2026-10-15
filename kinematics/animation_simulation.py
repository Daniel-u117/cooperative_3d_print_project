import queue

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon


# Fallback for callers that do not send physical_bounds in queue messages.
# Messages from command_script override these with the collision-check values.
BOUNDS = {
    1: dict(
        elbow_joint_f=26, elbow_joint_b=-26,
        distal_arm_f=246, distal_arm_b=-26,
        prox_joint_f=8, prox_joint_b=-8,
        proximal_arm_f=230, proximal_arm_b=-8,
    ),
    2: dict(
        elbow_joint_f=-25, elbow_joint_b=25,
        distal_arm_f=253, distal_arm_b=-25,
        prox_joint_f=8, prox_joint_b=-8,
        proximal_arm_f=234, proximal_arm_b=-8,
    ),
    3: dict(
        elbow_joint_f=-25, elbow_joint_b=25,
        distal_arm_f=253, distal_arm_b=-25,
        prox_joint_f=8, prox_joint_b=-8,
        proximal_arm_f=234, proximal_arm_b=-8,
    ),
}


def arm_boundary_points(px, cur_x_raw, cur_y_raw, bounds, p):
    """Return the supplied physical bounds and arm joints in the common frame."""
    boundary = px.boundary_points(cur_x_raw, cur_y_raw, **bounds)
    prox, elbow, cur = px.coords_to_plot(cur_x_raw, cur_y_raw)
    return (
        np.asarray(boundary["distal"], dtype=float),
        np.asarray(boundary["proximal"], dtype=float),
        prox, elbow, cur,
    )


def _polygon_rings(geometry):
    """Read Shapely mapping() output without importing Shapely in the GUI."""
    kind = geometry.get("type")
    if kind == "Polygon":
        yield from geometry["coordinates"]
    elif kind == "MultiPolygon":
        for polygon in geometry["coordinates"]:
            yield from polygon
    elif kind == "GeometryCollection":
        for part in geometry["geometries"]:
            yield from _polygon_rings(part)
    elif kind in ("LineString", "LinearRing"):
        yield geometry["coordinates"]
    elif kind == "MultiLineString":
        yield from geometry["coordinates"]
    elif kind == "Point":
        yield [geometry["coordinates"]]
    elif kind == "MultiPoint":
        for point in geometry["coordinates"]:
            yield [point]


def animation_simulation(simulation_queue, simulation_finished, stop_requested, printer):
    """Play one queue message per tick, preserving start/frame/result ordering.

    start/frame messages may supply arm_bounds and physical_bounds. result
    messages may supply positions, candidate_polygon, stored_polygons, and
    overlap_polygons. Geometry mappings use the collision frame; only drawing
    swaps the two coordinate columns, as in animation.py.
    """
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-100, 400)
    ax.set_ylim(-400, 400)
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_title("Candidate simulation")
    for xs, ys in (
        ([0, 300], [300, 300]), ([0, 0], [300, 0]),
        ([300, 300], [300, 0]), ([0, 300], [0, 0]),
        ([300, 300], [-300, 0]), ([0, 0], [-300, 0]),
        ([0, 300], [-300, -300]),
    ):
        ax.plot(xs, ys, color="black")

    status_text = ax.text(
        0.02, 0.98, "Simulation idle", transform=ax.transAxes,
        verticalalignment="top",
    )
    arm_lines = {}
    for index, printer_id in enumerate(printer):
        color = f"C{index % 10}"
        proximal, = ax.plot([], [], "o-", color=color, linewidth=3)
        distal, = ax.plot([], [], "o-", color=color, linewidth=5)
        arm_lines[printer_id] = proximal, distal

    nozzle_path, = ax.plot([], [], "k:", linewidth=1, label="nozzle path")
    ax.plot([], [], "b--", label="candidate sweep")
    ax.plot([], [], color="darkorange", linestyle="--", label="stored sweep")
    ax.plot([], [], "r-", linewidth=3, label="intersection")
    ax.legend(loc="lower left", fontsize=8)

    patches = []
    sweep_lines = []
    path = []
    last_positions = {}
    physical_bounds = {p: dict(values) for p, values in BOUNDS.items()}

    def artists():
        return tuple(
            [artist for pair in arm_lines.values() for artist in pair]
            + [status_text, nozzle_path] + patches + sweep_lines
        )

    def remove_patches():
        for patch in patches:
            patch.remove()
        patches.clear()

    def remove_sweeps():
        for line in sweep_lines:
            line.remove()
        sweep_lines.clear()

    def draw_geometry(geometry, color, linestyle="--", linewidth=1):
        for ring in _polygon_rings(geometry):
            points = np.asarray(ring, dtype=float)
            if points.size:
                line, = ax.plot(
                    points[:, 1], points[:, 0], color=color,
                    linestyle=linestyle, linewidth=linewidth,
                    marker="o" if len(points) == 1 else None,
                )
                sweep_lines.append(line)

    def update_positions(message, record_path=False):
        remove_patches()
        candidate_p = int(message["printer"])
        for p, values in message.get("physical_bounds", {}).items():
            physical_bounds[int(p)] = dict(values)
        for p, position in message.get("positions", {}).items():
            last_positions[int(p)] = tuple(position)

        for p, (x, y) in last_positions.items():
            px = printer[p]
            supplied_bounds = message.get("arm_bounds") if p == candidate_p else None
            if supplied_bounds is None:
                distal, proximal, prox, elbow, cur = arm_boundary_points(
                    px, x, y, physical_bounds[p], p,
                )
            else:
                # Draw the very same corner arrays used to construct the polygons.
                distal = np.asarray(supplied_bounds["distal"], dtype=float)
                proximal = np.asarray(supplied_bounds["proximal"], dtype=float)
                prox, elbow, cur = px.coords_to_plot(x, y)

            proximal_line, distal_line = arm_lines[p]
            proximal_line.set_data([prox[1], elbow[1]], [prox[0], elbow[0]])
            distal_line.set_data([elbow[1], cur[1]], [elbow[0], cur[0]])
            for quad in (distal, proximal):
                patch = MplPolygon(
                    quad[:, [1, 0]], closed=True, fill=False, linestyle="--",
                    edgecolor=distal_line.get_color(),
                )
                ax.add_patch(patch)
                patches.append(patch)
            if p == candidate_p and record_path:
                path.append((cur[1], cur[0]))
        if path:
            points = np.asarray(path)
            nozzle_path.set_data(points[:, 0], points[:, 1])

    def animate(_):
        try:
            message = simulation_queue.get_nowait()
        except queue.Empty:
            # multiprocessing.Queue's feeder can briefly lag behind put_nowait.
            # Wait for another timer tick; Empty alone is not completion.
            return artists()

        kind = message.get("type")
        if kind == "stop":
            simulation_finished.set()
            ani.event_source.stop()
            # Leave the last result visible for inspection.
        elif kind == "clear":
            remove_patches()
            remove_sweeps()
            path.clear()
            nozzle_path.set_data([], [])
            status_text.set_text("Simulation idle")
        elif kind in ("start", "frame"):
            if kind == "start":
                path.clear()
                nozzle_path.set_data([], [])
                remove_sweeps()
            update_positions(message, record_path=True)
            status_text.set_text(
                f"Simulating p{message['printer']}: {message.get('gcode', '?')}"
            )
        elif kind == "result":
            # Results carry final positions so the arms can still be drawn
            # when a bounded producer queue dropped some start/frame messages.
            if message.get("positions"):
                update_positions(message)
            remove_sweeps()
            if message.get("candidate_polygon"):
                draw_geometry(message["candidate_polygon"], "blue")
            for geometry in message.get("stored_polygons", {}).values():
                draw_geometry(geometry, "darkorange")
            for geometry in message.get("overlap_polygons", {}).values():
                draw_geometry(geometry, "red", "-", 3)
            accepted = bool(message.get("accepted"))
            status_text.set_text(
                f"p{message['printer']}: {message.get('gcode', '?')}\n"
                f"{'ACCEPTED' if accepted else 'REJECTED'}\n"
                f"overlap={float(message.get('overlap', 0.0)):.6f}\n"
                f"collisions={message.get('collisions', {})}"
            )
        return artists()

    def close_application(_):
        stop_requested.set()

    fig.canvas.mpl_connect("close_event", close_application)
    ani = FuncAnimation(
        fig, animate, init_func=artists, interval=30, blit=False,
        repeat=False, cache_frame_data=False,
    )
    plt.show()
