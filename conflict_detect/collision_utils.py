import numpy as np
import random
from pathlib import Path





def extract_xy(config, line: str):
    """Return X/Y values from a stripped G-code line if present."""
    x_match = config.coord_pattern["X"].search(line)
    y_match = config.coord_pattern["Y"].search(line)

    x = float(x_match.group(1)) if x_match else None
    y = float(y_match.group(1)) if y_match else None

    return x, y


def xy_points_after_m107(config, file_path: Path, start_x=None, start_y=None):
    """
    Yield printer XY positions after M107.

    If a G0/G1 line gives only X or only Y, the missing coordinate keeps
    its previous value, because G-code coordinates are modal.
    """
    cur_x = start_x
    cur_y = start_y
    #reached_m107 = False


    with open(file_path, "r") as r:
        for raw_line in r:
            line = raw_line.strip()   

            # if not reached_m107:
            #     if line == "M107":
            #         reached_m107 = True
            #     continue

            if not line or line.startswith(";"):
                continue

            command = line.split(maxsplit=1)[0]

            # if command not in {"G0", "G00", "G1", "G01"}:
            #     continue

            new_x, new_y = extract_xy(config, line)

            if new_x is None and new_y is None:
                continue

            if new_x is not None:
                cur_x = new_x

            if new_y is not None:
                cur_y = new_y

            if cur_x is None or cur_y is None:
                continue

            yield cur_x, cur_y


def write_sim_file(out_path: Path, frames):
    """
    Writes one frame per line, like:
    [(x1, y1), (x2, y2), 1]
    """
    with open(out_path, "w") as w:
        for frame in frames:
            w.write(repr(frame) + "\n")





def sim_px(p, codes, store_coords, config):

        
    for i in codes:
        targ = Path(rf'C:\Users\nural\python2\group-of-code2\boundaries_store\p{p}\{i}')
        with open(targ, 'r') as r:
            line = r.readline().strip()


            # while line != 'M107':
            #    line = r.readline().strip()


            #line = r.readline().strip()
            print(line)
            cur_x_sim, cur_y_sim = extract_xy(config, line)
            print(cur_x_sim, cur_y_sim)

        
       
        sim_one_px(p, i, cur_x_sim, cur_y_sim, store_coords, config)



###might be problematic, fix later####
def sim_one_px(p, file_name, cur_x_sim, cur_y_sim, store_coords, config):

    targ = config.px_fold[p] / file_name
    #remember that config is
            #     px_fold = {
            #     1: Path(r"C:\Users\nural\python2\group-of-code2\sim\p1"),
            #     2: Path(r"C:\Users\nural\python2\group-of-code2\sim\p2"),
            # }

            # out_px = {
            #     1: sim_fold / "p1",
            #     2: sim_fold / "p2",
            # }

            # for path in out_px.values():
            #     path.mkdir(parents=True, exist_ok=True)

            # config = SimConfig(
            #     sim_fold=sim_fold,
            #     px_fold=px_fold,
            #     out_px=out_px,
            #     coord_pattern=coord_pattern,
            # )

    frames = []



    for x, y in xy_points_after_m107(
        config,
        targ,
        start_x=cur_x_sim,
        start_y=cur_y_sim
    ):

        frame_coords = []

        for printer_id in sorted(store_coords):

            if printer_id == p:
                frame_coords.append((x, y))
            else:
                frame_coords.append(store_coords[printer_id])

        frame = frame_coords + [p]

        frames.append(frame)


    write_sim_file(
        config.out_px[p] / file_name,
        frames
    )



# def sim_one_p2(file_name, cur_x1_raw, cur_y1_raw, cur_x2_raw, cur_y2_raw, config):
#     """
#     Simulate one printer-1 G-code file.

#     Printer 1 moves.
#     Printer 2 stays frozen at its last known position.
#     """
#     targ = config.p2_fold / file_name
#     frames = []

#     last_x2 = cur_x2_raw
#     last_y2 = cur_y2_raw

#     for x2, y2 in xy_points_after_m107(config, targ, start_x=cur_x2_raw, start_y=cur_y2_raw):
#         last_x2 = x2
#         last_y2 = y2

#         frames.append([
#             (cur_x1_raw, cur_y1_raw),
#             (x2, y2),
#             2
#         ])

#     write_sim_file(config.out_p2 / file_name, frames)



# def sim_p2(codes, cur_x1_raw, cur_y1_raw, config):
#     for i in codes:
#         print(i)
#         targ = Path(rf'C:\Users\nural\python2\group-of-code2\p2\{i}')
#         with open(targ, 'r') as r:
#             line = r.readline().strip()

#             # while line != 'M107':
#             #     line = r.readline().strip()


#             # line = r.readline().strip()
#             print(line)
#             cur_x2_sim, cur_y2_sim = extract_xy(config, line)
#             print(cur_x2_sim, cur_y2_sim)

        
       
#         sim_one_p2(i, cur_x1_raw, cur_y1_raw,  cur_x2_sim, cur_y2_sim, config)








        

        


#     return None




#do i even use these?
def segment_intersects_circle(a, b, elbow_plot_x, elbow_plot_y, r):
    """
    Returns True if line segment AB intersects or touches
    the circle centered at (elbow_plot_x, elbow_plot_y)
    with radius r.
    """

    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    center = np.array(
        [elbow_plot_x, elbow_plot_y],
        dtype=float
    )

    # Direction of line segment
    ab = b - a

    # Handle case where a == b
    ab_squared = np.dot(ab, ab)

    if ab_squared == 0:
        return np.linalg.norm(a - center) <= r

    # Projection of circle center onto infinite line
    t = np.dot(center - a, ab) / ab_squared

    # Restrict projection to the finite segment
    t = np.clip(t, 0.0, 1.0)

    # Closest point on segment to circle center
    closest = a + t * ab

    # Distance from closest point to circle center
    distance = np.linalg.norm(closest - center)

    return distance <= r


def point_in_quad(point, quad):
    """
    Returns True if point is inside or on the boundary
    of a convex quadrilateral.
    """
    p = np.asarray(point, dtype=float)
    q = [np.asarray(x, dtype=float) for x in quad]

    signs = []

    for i in range(4):
        a = q[i]
        b = q[(i + 1) % 4]

        edge = b - a
        rel = p - a

        cross = edge[0] * rel[1] - edge[1] * rel[0]
        signs.append(cross)

    # inside if all cross products have the same sign
    return (
        all(s >= 0 for s in signs)
        or
        all(s <= 0 for s in signs)
    )


def segments_intersect(a, b, c, d):
    """
    Checks whether line segment AB intersects line segment CD.
    """

    def orientation(p, q, r):
        return (
            (q[0] - p[0]) * (r[1] - p[1])
            - (q[1] - p[1]) * (r[0] - p[0])
        )

    o1 = orientation(a, b, c)
    o2 = orientation(a, b, d)
    o3 = orientation(c, d, a)
    o4 = orientation(c, d, b)

    return (
        (o1 == 0 or o2 == 0 or o1 * o2 < 0)
        and
        (o3 == 0 or o4 == 0 or o3 * o4 < 0)
    )


def rectangles_intersect(rect1, rect2):
    """
    rect1 and rect2 must contain their four vertices
    in perimeter order.
    """

    # 1. Is any point of rectangle 2 inside rectangle 1?
    for p in rect2:
        if point_in_quad(p, rect1):
            return True

    # 2. Is any point of rectangle 1 inside rectangle 2?
    for p in rect1:
        if point_in_quad(p, rect2):
            return True

    # 3. Do any edges cross?
    for i in range(4):
        a = rect1[i]
        b = rect1[(i + 1) % 4]

        for j in range(4):
            c = rect2[j]
            d = rect2[(j + 1) % 4]

            if segments_intersect(a, b, c, d):
                return True

    return False




