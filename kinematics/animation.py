import numpy as np
import matplotlib.pyplot as plt
import queue




from matplotlib.animation import FuncAnimation
from matplotlib.ticker import FuncFormatter
from matplotlib.patches import Circle
import time
from shapely.geometry import MultiPoint, Polygon
from shapely import concave_hull
from shapely.ops import unary_union


def closest_point_on_segment(p, q, r):
    p = np.asarray(p, dtype=float)
    q = np.asarray(q, dtype=float)
    r = np.asarray(r, dtype=float)

    direction = p - q
    length_squared = np.dot(direction, direction)


    if np.isclose(length_squared, 0.0):
        closest = q.copy()
        distance = np.linalg.norm(r - closest)
        return closest, 0.0, distance

    t_raw = np.dot(r - q, direction) / length_squared



    t = np.clip(t_raw, 0.0, 1.0)

    closest = q + t * direction
    distance = np.linalg.norm(r - closest)

    return closest, t, distance






def machine2_to_plot(x2, y2):
    R = np.array([[np.cos(np.pi/2), np.sin(np.pi/2)],[-np.sin(np.pi/2), np.cos(np.pi/2)]])
    new_vec = np.array([-600,300]) + np.array([300,0]) + R@np.array([x2, y2])

    return new_vec[0], new_vec[1]

def machine1_to_plot(x2, y2):
    R = np.array([[np.cos(-np.pi/2), np.sin(-np.pi/2)],[-np.sin(-np.pi/2), np.cos(-np.pi/2)]])
    new_vec = np.array([0,0]) + np.array([300,0]) + R@np.array([x2, y2])

    return new_vec[0], new_vec[1]


def inverse_kinematic(cur_x, cur_y, prox_join, length_a, length_b):


    

    length_c = np.sqrt((cur_y - prox_join[1])**2 + (prox_join[0]-cur_x)**2)


    angle_A = np.arccos((-length_c**2 + length_a**2 + length_b**2)/(2*length_a*length_b)) * 180/np.pi
    angle_c3 = 180 - angle_A



    X = cur_x - prox_join[0]
    Y = cur_y - prox_join[1]

    A = length_a + (length_b*np.cos(angle_c3*(np.pi/180)))
    B = length_b * np.sin(angle_c3*(np.pi/180))
    R = np.sqrt(A**2 + B**2); alpha = np.arctan2(B, A)
    theta_1 = np.arctan2(Y, X) - alpha







    return theta_1, prox_join[0] + length_a*np.cos(theta_1), prox_join[1] + length_a*np.sin(theta_1)



def animation_script(position_queue1, execution_finished, stop_requested, printer):

    #width of the nozzle is about 1/3 of 50, so about 17
    #width of the elbow is about 1.5 times 50 = 75
    def circle_p1_1(cur_x, cur_y):
        return Circle(
            xy = (cur_y, cur_x),
            radius = float(17/2),
            transform = ax.transData,
            fill = False,
            linestyle = '--'
        )
    def circle_p1_2(elbow_x, elbow_y):
        return Circle(
            xy = (elbow_y, elbow_x),
            radius = float(75/2),
            transform = ax.transData,
            fill = False,
            linestyle = '--'
        )
    def circle_p2_1(cur_x, cur_y):
        return Circle(
            xy = (cur_y, cur_x),
            radius = float(17/2),
            transform = ax.transData,
            fill = False,
            linestyle = '--'
        )
    def circle_p2_2(elbow_x, elbow_y):
        return Circle(
            xy = (elbow_y, elbow_x),
            radius = float(75/2),
            transform = ax.transData,
            fill = False,
            linestyle = '--'
        )



    def data_width_to_points(ax, width_data):
        """
        Convert a width measured in axes data units into Matplotlib points.

        This assumes equal x/y scaling, which you already enforce with:
            ax.set_aspect("equal")
        """
        start_px = ax.transData.transform((0.0, 0.0))
        end_px = ax.transData.transform((width_data, 0.0))

        width_pixels = np.linalg.norm(end_px - start_px)

        # Matplotlib linewidth is measured in points.
        return width_pixels * 72.0 / ax.figure.dpi

    def update_physical_linewidths(_=None):

        proximal_width_points = data_width_to_points(
            ax,
            10
        )

        distal_width_points = data_width_to_points(
            ax,
            31.25
        )

        for px in printer.values():
            px.line1.set_linewidth(proximal_width_points)
            px.line2.set_linewidth(distal_width_points)



    # prox_join11 = np.array([150.236, -50.09536])
    # length_a1 = 221.0968 
    # length_b1 = 220.61278 


    # prox_join2 = np.array([151.16978, -53.04552])
    # length_a2 = 225.39945
    # length_b2 = 227.54048

    fig, ax = plt.subplots(figsize=(7, 7))


    ax.plot([0, 300], [300, 300], color="black")
    ax.plot([0, 0], [300, 0], color="black")
    ax.plot([300, 300], [300, 0], color="black")
    ax.plot([0, 300], [0, 0], color="black")
    ax.plot([300, 300], [-300, 0], color = 'black')
    ax.plot([0,0], [-300, 0], color = 'black')
    ax.plot([0, 300], [-300, -300], color = 'black')





    # for i in codes1:
    #     for j in codes1[i][1]:
    #         x,y = j
    #         x,y = machine1_to_plot(x,y)
    #         ax.annotate(
    #             f'{i}',
    #             xy = (y,x),
    #             color = 'blue',
    #             fontsize = 5
    #         )

    # for i in codes2:
    #     for j in codes2[i][1]:
    #         x,y = j
    #         x,y = machine2_to_plot(x,y)
    #         ax.annotate(
    #             f'{i}',
    #             xy = (y,x),
    #             color = 'red',
    #             fontsize = 5,
    #         )


    ax.set_xlim(-100, 400)
    ax.set_ylim(-400, 400)


    # ax.yaxis.set_major_formatter(
    #     FuncFormatter(lambda x_plot, pos: f"{300 - x_plot:g}")
    # )

    ax.set_xlabel("machine x1")
    ax.set_ylabel("machine y1")
    ax.tick_params(axis="x", colors="black")
    ax.tick_params(axis="y", colors="black")

    sec_x = ax.secondary_xaxis(
        "top",
        functions=(
            # forward: primary horizontal y1 -> machine y2
            lambda x1: 300 - x1,

            # inverse: machine y2 -> primary horizontal y1
            lambda x2: 300 - x2
        )
    )

    sec_y = ax.secondary_yaxis(
        "right",
        functions=(
            # forward: primary vertical x1 -> machine x2
            lambda x1: -x1-300,

            # inverse: machine x2 -> primary vertical x1
            lambda x2: -x2-300
        )
    )

    sec_x.set_xlabel("machine x2", color="red")
    sec_y.set_ylabel("machine y2", color="red")

    sec_x.tick_params(axis="x", colors="red")
    sec_y.tick_params(axis="y", colors="red")
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)






    # #all these are now in the plotting co-ordinates, not co-ordinates of p1

    # _, elbow_x1, elbow_y1 = inverse_kinematic(cur_x1_raw, cur_y1_raw, prox_join11, length_a1, length_b1)



    # prox_join_x, prox_join_y = machine1_to_plot(
    #     prox_join11[0],
    #     prox_join11[1]
    # )


    # elbow_x, elbow_y = machine1_to_plot(
    #     elbow_x1,
    #     elbow_y1
    # )


    # cur_x1, cur_y1 = machine1_to_plot(
    #     cur_x1_raw,
    #     cur_y1_raw
    # )
    # prox_join1 = np.array([prox_join_x, prox_join_y]) #this is only for plotting purposes, real prox_join is prox_join11


    for printer_id, px in printer.items():
        for i in px.codes:
            for j in px.codes[i][1]:

                x, y = px.px_to_p1(np.array(j))
                x, y = x, y # for plotting purposes
                #offset = [150, -150][printer_id - 1]

                ax.annotate(
                    f'p{printer_id}:{i}',
                    xy=(y, x),
                    fontsize=5
                )


    #all this needs to be in config
    cur_x1_raw = 300
    cur_y1_raw = 0

    cur_x2_raw = 300
    cur_y2_raw = 0

    cur_x3_raw = 300
    cur_y3_raw = 0

    coord_raw = {
        1: (cur_x1_raw, cur_y1_raw),
        2: (cur_x2_raw, cur_y2_raw),
        3: (cur_x3_raw, cur_y3_raw)
    }

    proximal_width = data_width_to_points(ax, 10)
    distal_width = data_width_to_points(ax, 31.25)

    for px in printer:
        printer[px].plot(ax, coord_raw[px][0], coord_raw[px][1], proximal_width, distal_width)

    





    # line1, = ax.plot(
    #     [prox_join1[1], elbow_y],
    #     [prox_join1[0], elbow_x],
    #     "bo-",
    #     linewidth=proximal_width,
    #     label="proximal arm1",
    # )


    # line2, = ax.plot(
    #     [elbow_y, cur_y1],
    #     [elbow_x, cur_x1],
    #     "ro-",
    #     linewidth=distal_width,
    #     label="distal arm1",
    # )







# Coordinates coming from printer





    #all these are co-odinates from plotting POV, not p2

    # _, elbow_x2, elbow_y2 = inverse_kinematic(
    #     cur_x2_raw,
    #     cur_y2_raw,
    #     prox_join2,
    #     length_a2,
    #     length_b2
    # )



    # prox_plot_x, prox_plot_y = machine2_to_plot(
    #     prox_join2[0],
    #     prox_join2[1]
    # )

    # elbow_plot_x, elbow_plot_y = machine2_to_plot(
    #     elbow_x2,
    #     elbow_y2
    # )

    # cur_plot_x, cur_plot_y = machine2_to_plot(
    #     cur_x2_raw,
    #     cur_y2_raw
    # )


    # line3, = ax.plot(
    #     [prox_plot_y, elbow_plot_y],
    #     [prox_plot_x, elbow_plot_x],
    #     "bo-",
    #     linewidth=proximal_width,
    #     label="proximal arm2",
    # )

    # line4, = ax.plot(
    #     [elbow_plot_y, cur_plot_y],
    #     [elbow_plot_x, cur_plot_x],
    #     "ro-",
    #     linewidth=distal_width,
    #     label="distal arm2",
    # )


    #precomputed k values for boundaries


    #  boundaries = {1: {

    #  },
    #  2:{
         
    #  }}

    BOUNDS = {
        1: dict(
            elbow_joint_f=26,
            elbow_joint_b=-26,
            distal_arm_f=246,
            distal_arm_b=-26,
            prox_joint_f=8,
            prox_joint_b=-8,
            proximal_arm_f=230,
            proximal_arm_b=-8,
        ),
        2: dict(
            elbow_joint_f=-25,
            elbow_joint_b=25,
            distal_arm_f=253,
            distal_arm_b=-25,
            prox_joint_f=8,
            prox_joint_b=-8,
            proximal_arm_f=234,
            proximal_arm_b=-8,
        ),
        3: dict(
                elbow_joint_f=26,
                elbow_joint_b=-26,
                distal_arm_f=246,
                distal_arm_b=-26,
                prox_joint_f=8,
                prox_joint_b=-8,
                proximal_arm_f=230,
                proximal_arm_b=-8,
            )
    }
    



    coordinate_text = ax.text(
        0.02,
        0.98,
        "Waiting for execution...",
        transform=ax.transAxes,
        verticalalignment="top",
    )










    

    fig.canvas.mpl_connect(
    "resize_event",
    update_physical_linewidths
    )



    #remember there is a difference between t = ax.plot and t, = ax.plot
    # t --> returns a matplotlib plot list, 't,' returns an actual line object




    def base_artists(): #assembles the core list of objects for animation
        artists = []

        for px in printer.values():
            artists.extend([
                px.line1,
                px.line2,
            ])

        artists.append(coordinate_text)

        return artists

    def initialize():
        for px in printer.values():
            px.line1.set_data([], [])
            px.line2.set_data([], [])

        coordinate_text.set_text("Waiting for execution...")

        return tuple(base_artists()) #need to use tuple form
        
        # line1.set_data([], [])
        # line2.set_data([], [])
        # line3.set_data([], [])
        # line4.set_data([], [])
        #how do i return the line objects inside list 'l' (there could be a variable number of them)?
        return line1, line2, line3, line4, coordinate_text


    temporary_lines = []
    temporary_circles = []





    def animate(_):



        try:
            store_pos = dict()


            frame = position_queue1.get_nowait()

            *positions, p = frame

            for i, j in enumerate(positions):
                store_pos[i+1] = j


        except queue.Empty:
            print(f'execution_finished? {execution_finished.is_set()}')
            if execution_finished.is_set():
                ani.event_source.stop()
                coordinate_text.set_text("All executions finished")
                fig.canvas.draw_idle()
                print('animation ended')
                
            else:

                print('empty queue, delay occured \n\n\n\n\n\n\n\n\n')
            return tuple(base_artists())
        
        for line in temporary_lines:
            line.remove()
        temporary_lines.clear()

        for circle in temporary_circles:
            circle.remove()
        temporary_circles.clear()



                

        

        cur_x_raw, cur_y_raw = store_pos[p]
        cur_x_raw, cur_y_raw = cur_x_raw, cur_y_raw
        px = printer[p]
        prox1, elbow1, cur1 = px.coords_to_plot(cur_x_raw, cur_y_raw)

        px.line1.set_data(
            [prox1[1], elbow1[1]], #here and everywhere we switch places of x and y becaseu matplot needs it for plotting purposes
            [prox1[0], elbow1[0]]
        )

        px.line2.set_data(
            [elbow1[1], cur1[1]], 
            [elbow1[0], cur1[0]]
        )




        #the k dervied lines

        lines_px = px.plot_bounds(
            ax,
            cur_x_raw,
            cur_y_raw,
            **BOUNDS[p]
        )



        # close = closest_point_on_segment([elbow_plot_y, elbow_plot_x], [cur_plot_y, cur_plot_x], [cur_x1, cur_y1])
        # #this is very wrong, need to use machine1_to_plot here
        # min_d = min([np.linalg.norm(np.array([cur_x1, cur_y1]) - np.array([cur_plot_y, cur_plot_x])), close[2], np.linalg.norm(np.array([cur_x1, cur_y1]) - np.array([elbow_plot_y, elbow_plot_x]))])
        


        # coordinate_text.set_text(
        # f"x_p1 = {cur_x1:.3f}\n"
        # f"y_p1 = {cur_y1:.3f}\n"
        # f"x_p2 = {cur_plot_y:.3f}\n"
        # f"y_p2 = {cur_plot_x:.3f}\n"
        # f"p = {p}\n\n"
        # f'p1 to p2 nozzle: {np.linalg.norm(np.array([cur_x1, cur_y1]) - np.array([cur_plot_y, cur_plot_x]))}\n'
        # f'p1 to p2 shortest point {close[2]}\n'
        # f'p1 to p2 elbow: {np.linalg.norm(np.array([cur_x1, cur_y1]) - np.array([elbow_plot_y, elbow_plot_x]))}\n'
        # f"queued = {position_queue1.qsize()}"
        # )   


        # closex, closey = machine1_to_plot(close[0][0], close[0][1])


        ####dont use the thing below but us it as a template)
        # t1, = ax.plot(
        #     [cur_y1, closey],
        #     [cur_x1, closex]    
        # )

        
        circle1 = circle_p1_1(cur1[0], cur1[1])
        ax.add_patch(circle1)
        temporary_circles.append(circle1)

        circle2 = circle_p1_2(elbow1[0], elbow1[1])
        ax.add_patch(circle2)
        temporary_circles.append(circle2)

        #the temperatory lines are for distances i think


            

        return tuple(base_artists() + temporary_lines + temporary_circles)

        



    ani = FuncAnimation(
        fig,
        animate,
        init_func=initialize,
        frames=None,
        interval=30,
        blit=False,
        repeat=False,
        cache_frame_data=False,
    )
    plt.show()

    def close_application(_):
        stop_requested.set()

    fig.canvas.mpl_connect(
        "close_event",
        close_application,
    )
