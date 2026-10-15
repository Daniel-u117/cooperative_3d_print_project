import multiprocessing as mp
import queue
import time
import traceback

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


from shapely.geometry import MultiPoint, Polygon
from shapely import concave_hull


import re
import socket
import subprocess
import requests
import os
import numpy as np
from pathlib import Path
import random
from matplotlib.ticker import FuncFormatter
from matplotlib.patches import Circle
from dataclasses import dataclass

from util.network_utils import run_netsh, current_wifi, tcp_reachable, reconnect_to_ambots
from kinematics.animation import inverse_kinematic, animation_script, machine1_to_plot, machine2_to_plot, closest_point_on_segment
from util.tracking import track
from util.command_script import command_script, polygon_at_position
from util.execution import execution_worker
from conflict_detect.collision_utils import segments_intersect, segment_intersects_circle, point_in_quad, rectangles_intersect, extract_xy, xy_points_after_m107, write_sim_file, sim_one_px, sim_px
from discretization.file_utils import parse_beg, parse_end, parse_mid

from dataclasses import dataclass, field
import numpy as np
from kinematics.animation_simulation import animation_simulation








@dataclass(frozen=True)
class SimConfig:
    sim_fold: Path
    px_fold: dict[int, Path]
    out_px: dict[int, Path]
    coord_pattern: dict



@dataclass
class Printer:
    px_ref: np.ndarray
    theta: float
    prox_join: np.ndarray #raw
    length_a: float
    length_b: float
    codes: dict = field(default_factory=dict)

    line1: object = field(default=None, init=False)
    line2: object = field(default=None, init=False)

    p1_ref: np.ndarray = field(
        default_factory=lambda: np.array([300.0, 0.0])
    )



    


    def px_to_p1(self, point_px):

        R = np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta),  np.cos(self.theta)]
        ])

        return R @ (point_px - self.p1_ref) + self.px_ref

    

    def elbow_pos_raw(self, cur_x_raw, cur_y_raw):
        _, elbow_x1, elbow_y1 = inverse_kinematic(
            cur_x_raw,
            cur_y_raw,
            self.prox_join,
            self.length_a,
            self.length_b
        )

        return np.array([elbow_x1, elbow_y1])

    def coords_to_plot(self, cur_x_raw, cur_y_raw):
        return self.px_to_p1(self.prox_join), self.px_to_p1(self.elbow_pos_raw(cur_x_raw, cur_y_raw)), self.px_to_p1(np.array([cur_x_raw, cur_y_raw]))
    
    def plot(self, ax, cur_x_raw, cur_y_raw, proximal_width, distal_width):

        prox_plot = self.px_to_p1(self.prox_join)
        elbow_plot = self.px_to_p1(self.elbow_pos_raw(cur_x_raw, cur_y_raw))
        cur_plot = self.px_to_p1(np.array([cur_x_raw, cur_y_raw]))

        ax.plot(
            prox_plot[1],
            prox_plot[0],
            'o',
            color = 'cyan'
        )

        # ax.annotate(f'{self.prox_join[0]}, {self.prox_join[1]}',
        #     xy = (prox_plot[1], prox_plot[0])
        # )   

        ax.plot(
            elbow_plot[1],
            elbow_plot[0],
            'o',
            color='green'
        )

        # ax.annotate(f'{self.elbow_pos_raw(cur_x_raw, cur_y_raw)[0]}, {self.elbow_pos_raw(cur_x_raw, cur_y_raw)[1]}',
        #             xy = (elbow_plot[1], elbow_plot[0])

        # )


        ax.plot(
            cur_plot[1],
            cur_plot[0],
            'o',
            color='purple'
        )

        # ax.annotate(f'{cur_x_raw}, {cur_y_raw}',
        #             xy = (cur_plot[1], cur_plot[0])

        # )


        line1, = ax.plot(
            [prox_plot[1], elbow_plot[1]],
            [prox_plot[0], elbow_plot[0]],
            "bo-",
            linewidth=proximal_width,
            label="proximal arm1",
        )

        line2, = ax.plot(
            [elbow_plot[1], cur_plot[1]],
            [elbow_plot[0], cur_plot[0]],
            "ro-",
            linewidth=distal_width,
            label="distal arm1",
        )
        self.line1 = line1
        self.line2 = line2

    def boundary_points(
        self,
        cur_x_raw,
        cur_y_raw,
        elbow_joint_f,
        elbow_joint_b,
        distal_arm_f,
        distal_arm_b,
        prox_joint_f,
        prox_joint_b,
        proximal_arm_f,
        proximal_arm_b,
    ):

        prox_plot, elbow_plot, cur_plot = self.coords_to_plot(
            cur_x_raw,
            cur_y_raw
        )

        # -----------------------
        # distal arm
        # -----------------------

        v = cur_plot - elbow_plot

        perp = np.array([
            -v[1],
            v[0]
        ])

        perp_norm = perp / np.linalg.norm(perp)
        v_norm = v / np.linalg.norm(v)

        point1 = (
            elbow_plot
            + elbow_joint_f * perp_norm
            + distal_arm_b * v_norm
        )

        point2 = (
            elbow_plot
            + elbow_joint_f * perp_norm
            + distal_arm_f * v_norm
        )

        point3 = (
            elbow_plot
            + elbow_joint_b * perp_norm
            + distal_arm_b * v_norm
        )

        point4 = (
            elbow_plot
            + elbow_joint_b * perp_norm
            + distal_arm_f * v_norm
        )


        # -----------------------
        # proximal arm
        # -----------------------

        v2 = elbow_plot - prox_plot

        perp2 = np.array([
            -v2[1],
            v2[0]
        ])

        perp2_norm = perp2 / np.linalg.norm(perp2)
        v_norm2 = v2 / np.linalg.norm(v2)

        point11 = (
            prox_plot
            + prox_joint_f * perp2_norm
            + proximal_arm_f * v_norm2
        )

        point22 = (
            prox_plot
            + prox_joint_f * perp2_norm
            + proximal_arm_b * v_norm2
        )

        point33 = (
            prox_plot
            + prox_joint_b * perp2_norm
            + proximal_arm_f * v_norm2
        )

        point44 = (
            prox_plot
            + prox_joint_b * perp2_norm
            + proximal_arm_b * v_norm2
        )


        return {
            "distal": [
                point1,
                point2,
                point4,
                point3
            ],

            "proximal": [
                point11,
                point22,
                point44,
                point33
            ]
        }


    def plot_bounds(
        self,
        ax,
        cur_x_raw,
        cur_y_raw,
        elbow_joint_f,
        elbow_joint_b,
        distal_arm_f,
        distal_arm_b,
        prox_joint_f,
        prox_joint_b,
        proximal_arm_f,
        proximal_arm_b,
    ):

        bounds = self.boundary_points(
            cur_x_raw,
            cur_y_raw,
            elbow_joint_f,
            elbow_joint_b,
            distal_arm_f,
            distal_arm_b,
            prox_joint_f,
            prox_joint_b,
            proximal_arm_f,
            proximal_arm_b,
        )

        point1, point2, point4, point3 = bounds["distal"]

        # distal1_l1, = ax.plot(
        #     [point1[1], point2[1]],
        #     [point1[0], point2[0]],
        #     color='black'
        # )

        # distal1_l2, = ax.plot(
        #     [point1[1], point3[1]],
        #     [point1[0], point3[0]],
        #     color='black'
        # )

        # distal1_l3, = ax.plot(
        #     [point2[1], point4[1]],
        #     [point2[0], point4[0]],
        #     color='black'
        # )

        # distal1_l4, = ax.plot(
        #     [point3[1], point4[1]],
        #     [point3[0], point4[0]],
        #     color='black'
        # )


        point11, point22, point44, point33 = bounds["proximal"]

        # proximal1_l1, = ax.plot(
        #     [point11[1], point22[1]],
        #     [point11[0], point22[0]],
        #     color='black'
        # )

        # proximal1_l2, = ax.plot(
        #     [point11[1], point33[1]],
        #     [point11[0], point33[0]],
        #     color='black'
        # )

        # proximal1_l3, = ax.plot(
        #     [point22[1], point44[1]],
        #     [point22[0], point44[0]],
        #     color='black'
        # )

        # proximal1_l4, = ax.plot(
        #     [point33[1], point44[1]],
        #     [point33[0], point44[0]],
        #     color='black'
        # )

        # lines = [
        #     distal1_l1,
        #     distal1_l2,
        #     distal1_l3,
        #     distal1_l4,
        #     proximal1_l1,
        #     proximal1_l2,
        #     proximal1_l3,
        #     proximal1_l4
        # ]

        #return bounds, lines
        return bounds


def main():
    prox_join2 = np.array([151.16978, -53.04552])
    length_a2 = 225.39945
    length_b2 = 227.54048

    prox_join11 = np.array([150.236, -50.09536])
    length_a1 = 221.0968 #promximal arm
    length_b1 = 220.61278 #distal arm



    p1 = Printer(
        px_ref=np.array([300.0, 300.0]),
        theta=np.pi/2,
        prox_join=prox_join11,
        length_a=length_a1,
        length_b=length_b1
    )

    p2 = Printer(
        px_ref=np.array([0, 0]),
        theta=-np.pi / 2,
        prox_join=prox_join2,
        length_a=length_a2,
        length_b=length_b2
    )

    # p3 = Printer(
    #     px_ref=np.array([300.0, 300.0]),
    #     theta=np.pi / 2,
    #     prox_join=prox_join2,
    #     length_a=length_a2,
    #     length_b=length_b2
    # )
    
    printer = {
        1: p1,
        2: p2
    }

    #######################prepare the files###############






    folder = Path(r"c:\Users\nural\python2\group-of-code2\ulti_maker")


    def gcode_number(file_path):
        try:
            return int(file_path.stem)
        except ValueError:
            return float("inf")


    


    def read_gcode_points(file_path):
        x_pattern = re.compile(r"\bX\s*(-?(?:\d+(?:\.\d*)?|\.\d+))")
        y_pattern = re.compile(r"\bY\s*(-?(?:\d+(?:\.\d*)?|\.\d+))")
        points = []
        
        with open(file_path, 'r') as r:
            line = r.readline().strip()
            while line != 'M107':
                line = r.readline().strip()

            line = r.readline().strip()



            while line:
                x_match = x_pattern.search(line)
                y_match = y_pattern.search(line)
                # Only use lines containing BOTH X and Y
                if x_match and y_match:

                    x = float(x_match.group(1))
                    y = float(y_match.group(1))

                    points.append((x, y))

                line = r.readline().strip()
        return points
    
                
                



    def px_files(p, folder, printer):

        output_fold = folder / f"p{p}"
        output_fold.mkdir(exist_ok=True)

        px = printer[p]

        def p1_to_px(point_p1):

            R = np.array([
                [np.cos(px.theta), -np.sin(px.theta)],
                [np.sin(px.theta),  np.cos(px.theta)]
            ])

            return R.T @ (point_p1 - px.px_ref) + px.p1_ref


        x_pattern = re.compile(
            r"\bX\s*(-?(?:\d+(?:\.\d*)?|\.\d+))"
        )

        y_pattern = re.compile(
            r"\bY\s*(-?(?:\d+(?:\.\d*)?|\.\d+))"
        )


        def format_number(value):
            if float(value).is_integer():
                return str(int(value))

            return str(value)


        for file_path in folder.iterdir():

            if not file_path.is_file():
                continue

            output_lines = []

            for line in file_path.read_text().splitlines():

                # remove G28
                if line.startswith("G28"):
                    continue

                x_match = x_pattern.search(line)
                y_match = y_pattern.search(line)

                # Only transform lines that actually contain BOTH X and Y
                if x_match and y_match:

                    old_x = float(x_match.group(1))
                    old_y = float(y_match.group(1))

                    point_p1 = np.array([
                        old_x, # add -300 when  ultimaker conversion purposes for 600 by 300
                        old_y
                    ])
                   
                    new_x, new_y = p1_to_px(point_p1)

                    line = x_pattern.sub(
                        f"X{format_number(new_x)}",
                        line
                    )

                    line = y_pattern.sub(
                        f"Y{format_number(new_y)}",
                        line
                    )

                output_lines.append(line)


            output_path = output_fold / file_path.name

            output_path.write_text(
                "\n".join(output_lines) + "\n"
            )

    px_files(1, folder, printer)
    px_files(2, folder, printer)
    #px_files(3, folder, printer)

    # 1. Define your base directory properly using raw strings
    base_dir = Path(r"c:\Users\nural\python2\group-of-code2\sim")

    # 2. Use mkdir(parents=True, exist_ok=True) to prevent crashes if folders exist
    
    def processing(p, base_dir, folder):
        x_pattern = re.compile(r"\bX\s*(-?(?:\d+(?:\.\d*)?|\.\d+))")
        y_pattern = re.compile(r"\bY\s*(-?(?:\d+(?:\.\d*)?|\.\d+))")
        path_p1 = base_dir / f"p{p}"
        path_p1.mkdir(parents=True, exist_ok=True)
        for i in os.listdir(folder / f'p{p}'):

            points1 = read_gcode_points(folder / f'p{p}' / f'{i}')
            
            
            
            l = []

            #no machinetoplot and no xy reversal, that will be done in the other script
            for j in points1:
                x, y = j
                l.append((x, y))


            #solid = Polygon(l)
            cloud = MultiPoint(l)

            object = concave_hull(
                cloud,
                ratio=1
            ).simplify(1.0)

            #boundary_points = np.array(object.exterior.coords) #returns (5,2) array so only 5 data points along the boundary
            boundary = object.exterior


            N = 10
            #so .exterior.interpolate() allows you to get points along certain lines if you have a polygon or multipoint based on absolute distance or normalized distance (fraction of the total length)
            boundary_points = np.array([
                boundary.interpolate(z / N, normalized=True).coords[0]
                for z in range(N+1)
            ])

            print(f' boundary points {boundary_points[:, 0], boundary_points[:, 1]}')

            # 3. Define the filename (replacing the missing 'i')
            filename = f"{i}" 
            file_path = path_p1 / filename

            # 4. Open the file and write data with proper spacing and newlines
            with open(file_path, "w") as w:
                for x, y in zip(boundary_points[:, 0], boundary_points[:, 1]):
                    # Added \n so each coordinate pair goes on a new line
                    w.write(f"X{x} Y{y}\n") 

            print(f'boundary point shape {boundary_points.shape}')



    processing(1, base_dir, folder)
    processing(2, base_dir, folder)
    #processing(3, base_dir, folder)









    sim_fold = Path(r"C:\Users\nural\python2\sim_fold")
    sim_fold.mkdir(parents=True, exist_ok=True)




    coord_pattern = {
        "X": re.compile(r"\bX\s*(-?(?:\d+(?:\.\d*)?|\.\d+))"),
        "Y": re.compile(r"\bY\s*(-?(?:\d+(?:\.\d*)?|\.\d+))"),
    }
    px_fold = {
        1: Path(r"C:\Users\nural\python2\group-of-code2\boundaries_store\p1"),
        2: Path(r"C:\Users\nural\python2\group-of-code2\boundaries_store\p2"),
    }

    out_px = {
        1: sim_fold / "p1",
        2: sim_fold / "p2",
    }

    for path in out_px.values():
        path.mkdir(parents=True, exist_ok=True)

    config = SimConfig(
        sim_fold=sim_fold,
        px_fold=px_fold,
        out_px=out_px,
        coord_pattern=coord_pattern,
    )



    folder = Path(r'C:\Users\nural\python2\group-of-code2')


    


    '''this is for when we will send commands to the printers'''




    #later needs to be automated
    target_folder1 = Path(r'c:\Users\nural\python2\processed\p1')
    target_folder1.mkdir(exist_ok=True)

    target_folder2 = Path(r'c:\Users\nural\python2\processed\p2')
    target_folder2.mkdir(exist_ok=True)

    target_fold = {
        1: Path(r'c:\Users\nural\python2\processed\p1'),
        2: Path(r'c:\Users\nural\python2\processed\p2')
    }
    for a in target_fold.values():
        a.mkdir(exist_ok=True)


    for i in printer.keys():
        for j in os.listdir(Path(fr'c:\Users\nural\python2\group-of-code2\ulti_maker\p{i}')):
            parse_beg(Path(fr'c:\Users\nural\python2\group-of-code2\ulti_maker\p{i}') / j, target_fold[i], target_fold[i])
            parse_mid(Path(fr'c:\Users\nural\python2\group-of-code2\ulti_maker\p{i}') / j, target_fold[i], target_fold[i])
            parse_end(Path(fr'c:\Users\nural\python2\group-of-code2\ulti_maker\p{i}') / j, target_fold[i], target_fold[i])



    ip1 = '192.168.0.15'
    ip2 = '192.168.0.16'
    ip = {
        1: ip1,
        2: ip2
    }






    def read_gcode_points(px, file_path, prox_join, length_a, length_b):

        points = []
        check = True
        with open(file_path, "r") as r:


            for line_number, line in enumerate(r, start=1):

                x_match = coord_pattern['X'].search(line)
                y_match = coord_pattern['Y'].search(line)

                if x_match is not None and y_match is not None:

                    x = float(x_match.group(1))
                    y = float(y_match.group(1))
                    #the files that contains the co-ordinates of the points should also include the min/max co-ordinates just in case
                    if np.linalg.norm(np.array([x - prox_join[0], y - prox_join[1]])) < length_a + length_b:
                        points.append((x, y))
                    else:
                        check = False
                        break
        if check:
            px.codes[file_path.stem + '.gcode'] = ['A', points]

        return points
    




    folder_sim = Path(r'C:\Users\nural\python2\group-of-code2\boundaries_store')



    for printer_id, px in printer.items():

        path = folder_sim / f'p{printer_id}'

        for file_name in os.listdir(path):
            read_gcode_points(
                px,
                path / file_name,
                px.prox_join,
                px.length_a,
                px.length_b
            )



    for z in printer.values():
        print(z.codes)






    # #if you are connected to AMBOTS wifi, you need to first initiate connection with rr_connect
    









    # for now i am just uploading everything to both printers,


    '''this stage of uploading needs to occur after reachability filter'''





    for i in target_fold.keys():
        for j in os.listdir(target_fold[i]):
            ch = j[:-8] + '.gcode'
            if ch in printer[i].codes.keys():
                target = target_fold[i] / j
                print(target)
                print(requests.get(f'http://{ip[i]}/rr_connect?password=reprap').json())
                with open(target, "rb") as o:
                    response = requests.post(
                        f"http://{ip[i]}/rr_upload",
                        params={"name": f"0:/gcodes/sub_directory/{j}"},
                        data=o,
                        timeout=1000
                    )

                print(response.text)
                o.close()
        
        

    














    @dataclass
    class RuntimeBundle:
        """Queues and worker processes owned by the main process."""

        execution_request_queues: dict
        execution_event_queue: object
        tracker_control_queue: object
        tracker_event_queue: object
        execution_processes: dict
        tracker_process: object






    requests.get('http://192.168.0.15/rr_gcode', params = {'gcode':'G28'})
    time.sleep(45)
    requests.get('http://192.168.0.16/rr_gcode', params = {'gcode':'G28'})

    time.sleep(45)







    # ########################################################################################



    context = mp.get_context("spawn")
    position_queue1 = context.Queue(maxsize=1)
    stop_requested = context.Event()
    execution_finished = context.Event()
    error_queue = context.Queue()
    simulation_queue = context.Queue(maxsize=500)
    tracker_event_queue = context.Queue(maxsize=len(printer))
    execution_request_queue = context.Queue()
    execution_result_queue = context.Queue()
    tracker_control_queue = context.Queue()
    printing_queue = context.Queue()

    ip = {1: "192.168.0.15", 2: "192.168.0.16"}
    if set(printer) != set(ip):
        raise ValueError("The supplied get_status() supports only p1 and p2")
    manager = context.Manager()
    store_coords = manager.dict({p: (300.0, 0.0) for p in printer})
    store_polygons = manager.dict({p: None for p in printer})
    # Tracking sets each printer's coordinates BEFORE its first free message.

    execution_process = context.Process(
        target=execution_worker,
        args=(ip, execution_request_queue, execution_result_queue,
            tracker_control_queue, stop_requested, error_queue),
    )
    track_process = context.Process(
        target=track,
        args=(ip, store_coords, tracker_event_queue, tracker_control_queue,
            printing_queue, stop_requested, error_queue, position_queue1),
        kwargs={
            "printer": printer,
            "store_polygons": store_polygons,
            "polygon_at_position": polygon_at_position,
        },
    )
    command_process = context.Process(
        target=command_script,
        args=(printer, config, store_coords, tracker_event_queue,
            execution_request_queue, execution_result_queue,
            tracker_control_queue, printing_queue, stop_requested,
            execution_finished, error_queue, simulation_queue, "m"),
        kwargs={'store_polygons': store_polygons}
    )
    track_process.start()
    execution_process.start()
    command_process.start()









    simulation_visualization = True


    simulation_finished = context.Event()



    if simulation_visualization:

        simulation_animation_process = context.Process(
            target=animation_simulation,
            args=(
                simulation_queue,
                simulation_finished,
                stop_requested,
                printer
            ),
            name="simulation animation process"
        )

    else:
        simulation_animation_process = None

    if simulation_animation_process is not None:
        simulation_animation_process.start()






    try:
        animation_script(position_queue1, execution_finished, stop_requested,printer)
    finally:
        stop_requested.set()
        for process in (command_process, execution_process, track_process):
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
                process.join()
        manager.shutdown()




        if simulation_animation_process is not None:
            simulation_animation_process.join(timeout=5)
            if simulation_animation_process.is_alive():
                simulation_animation_process.terminate()
                simulation_animation_process.join()


        while True:
            try:
                worker_name, traceback_text = error_queue.get_nowait()
                print(f"\nError in {worker_name}:\n{traceback_text}")
            except queue.Empty:
                break

        position_queue1.close()
        error_queue.close()

        
        







if __name__ == "__main__":
    mp.freeze_support()
    main()