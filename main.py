"""Main program for the gravity simulation.

Setup and main loop of the gravity simulation.
Includes all functions that combine solar system data with the tkinter interface. 
Handles tkinter event bindings and other general functions.
Hobby project of PhySimdev. 

Typical usage example:

  interface = Interface()
"""

import math
import tkinter
import turtle
import time
from Simulation import Simulation, Planet, Sun, Blackhole
import numpy as np
import random
from PIL import ImageGrab

class Interface():
    """Handles tkinter and graphical methods."""

    def __init__(self):
        """Setup of the Interface.

        Declares variables for the tkinter interface and calls methods that
        start the simulation.
        """

        self.fov_range      = (200, 4000, 2000)
        self.y_rot_range    = (-90, 90, 0)
        self.default_dist   = 10**12
        self.z_rotation     = 0
        self.frame_count    = 0

        self.start_x_rot    = 0
        self.start_y_rot    = 0

        self.mouse_click1   = False
        self.mouse_click2   = False
        self.mouse_click3   = False
        self.pause          = False
        self.finished       = True

        self.timestep       = 100000  # in seconds, "simulation-time" per frame
        self.timepause      = 50    # pause between frames
        self.theta          = 1
        self.restitution_coefficient = 0  # inelastic collisions, if 1 fully elastic if 0 merge of planets

        self.absolute_pos   = True
        self.focus_index    = 1  # ["none","body","cm"] 0,1,2

        self.show_data      = True
        self.show_time      = True
        self.draw_frame_count = True

        self.path_color     = "darkgrey"

        self.bg_color       = "black"
        self.text_color     = "white"
        self.cube_color     = "green"
        self.font           = ("Courier New", 15, "normal")

        self.star_size_factor = 40
        self.planet_size_factor = 80
        self.min_body_size = .5
        self.path_planet_color = True
        self.draw_box       = True
        self.draw_trail     = True
        self.min_trail_size = .5
        self.trail_length   = 100000
        trail_resolution    = .5  # 0-1
        self.thickness_scale = 2.8 / 3
        self.trail_node_number   = int(self.trail_length * trail_resolution)
        self.trail_node_distance = int(self.trail_length / self.trail_node_number)  # amount of calculations between each node

        self.pointer_size   = 50
        self.onscreen       = []

        self.image_folder   = "C:/Users/gerri/Desktop/data"
        self.get_vid        = False
        self.max_frame      = 3600

        self.draw_rot       = True
        self.node_list_index= 1
        self.node_list      = ["regional", "point", "both"]
        self.node_type      = self.node_list[self.node_list_index]
        self.grid_thickness = 2 * 10**9
        self.rot_cube_pos   = [.8, -.8]  # zwischen -1,1, scale of screen
        self.rot_cube_scale = 80
        self.rot_cube_scolor = "grey"
        self.rot_cube_lcolor = "white"

        self.show_cm        = False
        self.cm_rad         = 10
        self.cm_color       = "white"

        self.map_colors     = False
        self.rainbow_rgb    = rgb_colors()
        self.color_attribute = "acceleration"
        self.max_acceleration = .1
        self.max_velocity   = 5 * 10**5
        self.max_force      = 9, 10**27
        self.color_mode_abs = True

        # mass,density, position, velocity, color (data from the web)
        self.starting_data  = [{'suns':    [('Sun',     1    , 1.41, (0., 0., 0.),     (0., 0., 0.),    'yellow')],
                                'planets': [('Mercury', 0.055, 5.43, (-0.449, 0., 0.), (0., 47.36, 0.), 'orange'),
                                            ('Venus',   0.815, 5.24, (-0.728, 0., 0),  (0., 35.02, 0.), 'yellow'),
                                            ('Earth',   1    , 5.51, (-1, 0, 0.),      (0., 29.78, 0.), 'lightgreen'),
                                            ('Mars',    0.107, 3.93, (-1.658, 0., 0.), (0., 24.08, 0.), 'red')]}]

        # Values for random creation
        self.start_random   = False
        self.size           = 50 * 10000000000  # in 10**(-10) au, len of cube size, has to be big number for randint
        self.max_velo       = 500  # in .15 km/s
        self.number_stars   = 100
        self.number_planets = 0  # trail resolution anpassen
        self.planet_colors  = ["beige", "lightgreen", "lightblue"]
        self.sun_colors     = ["yellow", "orange", "red"]

        # Benchmarking
        self.benchmark      = False
        self.setup_canvas()
        self.setup_simulation()
        self.reset()
        self.window.id = self.window.after(self.timepause, self.update_program)
        self.window.mainloop()

    def setup_canvas(self):
        """Setup of tkinter interface and turtle canvas.

        Creates tkinter widgets and the turtle canvas. Decleration of tkinter
        event bindings for keyboardpress and mouse movements.
        """

        self.window = tkinter.Tk()
        self.window.title("Gravity Simulation")
        self.window.attributes("-fullscreen", True)
        window_size_tuple = self.window.maxsize()
        self.width  = window_size_tuple[0]
        self.height = window_size_tuple[1]

        upper_grid        = tkinter.Frame(self.window)
        button_schließen  = tkinter.Button(upper_grid, text="leave", command=self.window.destroy)
        self.pause_button = tkinter.Button(upper_grid, text="pause", command=self.toggle_pause)
        self.reset_button = tkinter.Button(upper_grid, text="reset", command=self.reset)
        self.canvas       = tkinter.Canvas(self.window, width=window_size_tuple[0], height=window_size_tuple[1])

        upper_grid.pack       (side=tkinter.TOP, fill=tkinter.BOTH, expand=True)
        button_schließen.grid (column=0, row=0, sticky=tkinter.NSEW)
        self.pause_button.grid(column=1, row=0, sticky=tkinter.NSEW)
        self.reset_button.grid(column=2, row=0, sticky=tkinter.NSEW)
        self.canvas.pack      (side=tkinter.TOP, fill=tkinter.BOTH, expand=True)

        for i in range(3):
            tkinter.Grid.columnconfigure(upper_grid, i, weight=1)

        self.fenster = turtle.TurtleScreen(self.canvas)
        self.fenster.tracer(0)
        self.fenster.colormode(255)
        self.fenster.bgcolor(self.bg_color)

        self.pointer = turtle.RawTurtle(self.fenster)
        self.pointer.ht()
        self.pointer.up()

        self.data_pointer = turtle.RawTurtle(self.fenster)
        self.data_pointer.up()
        self.data_pointer.ht()
        self.data_pointer.color(self.text_color)

        self.canvas.bind("<MouseWheel>", self.mouse_scroll)
        self.canvas.bind("<B1-Motion>", self.offset)
        self.canvas.bind("<B2-Motion>", self.change_fov)
        self.canvas.bind("<B3-Motion>", self.rotation)

        self.canvas.bind("<ButtonRelease-1>", lambda event: self.mouse_off(event, "b1"))
        self.canvas.bind("<ButtonRelease-2>", lambda event: self.mouse_off(event, "b2"))
        self.canvas.bind("<ButtonRelease-3>", lambda event: self.mouse_off(event, "b3"))
        self.window.bind("<Left>",  lambda event: self.switch_focus(event, "right"))
        self.window.bind("<Right>", lambda event: self.switch_focus(event, "left"))
        self.window.bind("<Up>",    lambda event: self.switch_focus(event, "right"))
        self.window.bind("<Down>",  lambda event: self.switch_focus(event, "left"))

    def getter(self, widget):
        """Saves screen image of a widget.

        Saves pixel data of a tkinter widget and saves it as an image. Uses
        PIL.ImageGrab.grab().crop((x,y,z,h)) to create an image.

        Args:
            widget: A tkinter widget that will be scanned. Preferably the
                turtle canvas.
        """
        x = self.window.winfo_rootx() + widget.winfo_x()
        y = self.window.winfo_rooty() + widget.winfo_y()
        x1 = x + widget.winfo_width()
        y1 = y + widget.winfo_height()
        image = ImageGrab.grab().crop((x, y, x1, y1))
        image.save(f"{self.image_folder}/frame_{self.frame_count}.gif")

    def switch_focus(self, event, direction):
        if direction == "left":
            self.solar_system.switch_focus("previous")
        elif direction == "right":
            self.solar_system.switch_focus("next")
        if not self.absolute_pos and self.solar_system.focus_type == "body":
            self.solar_system.clear_trail()

    def toggle_pause(self):
        """Binary switch of self.pause"

        Method called by the pause button. Changes text on that button.
        """
        if not self.pause:
            self.pause = True
            self.pause_button.config(text="start")
        else:
            self.pause = False
            self.pause_button.config(text="pause")

    def mouse_off(self, event, button):
        if button == "b1":
            self.mouse_click1 = False
        elif button == "b2":
            self.mouse_click2 = False
        elif button == "b3":
            self.mouse_click3 = False

    def change_fov(self, event):
        if not self.mouse_click2:
            self.old_y = event.y
            self.mouse_click2 = True
        elif self.mouse_click2 and self.finished:
            self.FOV -= (event.y - self.old_y) * 10
            self.old_y = event.y

            if self.FOV < self.fov_range[0]:
                self.FOV = self.fov_range[0]
            elif self.FOV > self.fov_range[1]:
                self.FOV = self.fov_range[1]

    def rotation(self, event):
        if not self.mouse_click3:
            self.old_x = event.x
            self.old_y = event.y
            self.mouse_click3 = True
        elif self.mouse_click3 and self.finished:
            self.y_rotation += (event.x - self.old_x) / 10
            self.x_rotation += (event.y - self.old_y) / 10
            self.old_x = event.x
            self.old_y = event.y

            if self.y_rotation < self.y_rot_range[0]:
                self.y_rotation = self.y_rot_range[0]
            elif self.y_rotation > self.y_rot_range[1]:
                self.y_rotation = self.y_rot_range[1]

    def offset(self, event):
        if not self.mouse_click1:
            self.old_x = event.x
            self.old_y = event.y
            self.mouse_click1 = True
        elif self.mouse_click1 and self.finished:
            self.x_offset += (event.x - self.old_x) * self.distance / 1000
            self.y_offset += (event.y - self.old_y) * self.distance / 1000
            self.old_x = event.x
            self.old_y = event.y

    def mouse_scroll(self, event):
        self.distance -= (event.delta) * self.distance / 1000
        if self.distance < 0:
            self.distance = 0

    def draw_data(self, data):
        """Method that writes data onto the screen.

        Method that writes detailed data onto the turtle canvas.
        Uses the data arg adn formats it to a specfic form.

        Args:
            data: List of body and system attributes that will be written
                onto the screen. It includes:
                0: type of focus either "none", "body" or "cm" 
                1: the focused body
                2: the absolute postion of that body
                3: theta of the barnes hut algorythm
                4: the restituion coefficient for collisions
                5: the total kinetic energy 
                6: the total potential energy 
                7: the total energy
                8: the positon of the center of mass
                9: the curretn iteration
                10: the amount of bodies
                11: middle of the octree root node
                12: dimensionof the octree root node
        """
        y_pos = .20
        text = ""
        if data[1] is not None:  # focus on body
            y_pos = .08
            hover_body = data[1]
            name    = hover_body.name
            pos     = hover_body.position.copy()
            radius  = hover_body.radius
            mass    = hover_body.mass
            velocity = hover_body.velocity.copy()
            acceleration = hover_body.acceleration.copy()
            density = hover_body.density
            force = hover_body.force.copy()
            if not self.absolute_pos:
                pos -= hover_body.position

            vectors = [pos, velocity, acceleration, force]
            for i in range(len(vectors)):
                vec_mag = math.sqrt(np.dot(vectors[i], vectors[i]))
                vectors[i] = f"\
mag:{vec_mag:.2e} vec:({vectors[i][0]:.2e},{vectors[i][1]:.2e},{vectors[i][2]:.2e})"

            text += f"""\
Name:         {name}\n\
Radius:       {radius:.2e}\n\
Mass:         {mass:.2e}\n\
Density:      {density}\n\
Position:     {vectors[0]}\n\
Velocity:     {vectors[1]}\n\
Acceleration: {vectors[2]}\n\
Force:        {vectors[3]}\n\
"""

        if data[2]:
            view = "absolute"
        else:
            view = "relative"

        cm_pos = f"vec:({data[8][0]:.2e},{data[8][1]:.2e},{data[8][2]:.2e})"
        text += f"""\
Focus type:   {data[0]}\n\
View:         {view}\n\
Theta:        {data[3]}\n\
R.Coefficient:{data[4]}\n\
Ekin:         {data[5]:.2e}\n\
Epot:         {data[6]:.2e}\n\
Etotal:       {data[7]:.2e}\n\
Center of m.: {cm_pos}\n\
Iteration:    {data[9]:.2e}\n\
Nr. Bodies    {data[10]:.2e}\n\
Root n. size: {data[12]:.2e}\n"""

        self.data_pointer.goto(self.width * -0.46, self.height * y_pos)
        self.data_pointer.write(text, move=True, align="left", font=self.font)

    def draw_time(self, time):
        """Method that writes the physics time.

        Method that writes the accurrate physics time of the simulation.
        Calls the external sec to day function wich returns the accurate 
        time in clear steps.

        Args:
            time: Seconds of physics time since initiation. 
        """
        self.data_pointer.goto(self.width * -0.46, -self.height * 0.42)
        times = ConvertSectoDay(self.solar_system.time)
        text = ""
        if self.draw_frame_count:
            text += f"frame:   {self.frame_count}\n"
        text += f"years:   {times[0]}\ndays:    {times[1]}\nhours:   {times[2]}\nminutes: {times[3]}\nseconds: {times[4]}"
        self.data_pointer.clear()
        self.data_pointer.write(text, align="left", font=self.font)

    def reset(self):
        self.FOV        = self.fov_range[2]
        self.distance   = self.default_dist
        self.x_offset   = 0
        self.y_offset   = 0
        self.x_rotation = self.start_x_rot
        self.y_rotation = self.y_rot_range[2] + self.start_y_rot

    def get_screen_xy(self, x, y, z):
        """Method that transforms a 3d point into 2d point.

        Method that performs matrix multiplication and more to determine
        the correct 2d screen position of a 3d point. Uses rotation,
        transformation and distance aswell as FOV.

        Args:
            x: X component of position in 3d space.
            y: Y component of position in 3d space.
            z: Z component of position in 3d space.

        Returns:
            Either returns a xy position tuple and the point distance
            factor f. If the point is behind the screen None,None is returned.
            The output will be used if it is not None,None during calculations
            in the method update vertices.
        """
        default_pos = np.array([[x], [y], [z]])
        pos = self.rotation_matrix * default_pos

        x = pos[0, 0]
        y = pos[1, 0]
        z = pos[2, 0]

        real_dist = self.distance * (self.FOV / 1000)
        x += self.x_offset
        y += self.y_offset
        z += real_dist

        if z > 0:
            f = self.FOV / z
            sx, sy = x * f, y * f
            return (sx, -sy), f
        else:
            return None, None

    def draw_last_pos(self, last_pos, color, radius, body_size_factor):
        """Method that draws the trail of the body.

        Method that draws the trail of a body. Only draws line
        if the points of that Segment are on the screen wich 
        means screen position and size factor f is not None.

        Args:
            last_pos: List of last positions of the body.
            color: Color of the body.
            radius: Radius of the body.
            body_size_factor: Size factor that 
                scales body size.
        """
        last_screen_pos = []
        last_pos.reverse()
        for i, pos in enumerate(last_pos):
            s_pos, f = self.get_screen_xy(*pos)
            if f is not None:
                rad = radius * body_size_factor * f * self.thickness_scale**i
            else:
                rad = None
            last_screen_pos.append((s_pos, f, rad))

        last_pos.reverse()
        self.pointer.pencolor(self.path_color)
        if self.path_planet_color:
            self.pointer.pencolor(color)
        for posf in last_screen_pos:
            pos = posf[0]
            f   = posf[1]
            rad = posf[2]

            if pos is None:
                self.pointer.up()
            else:
                if rad <= self.min_trail_size:
                    rad = self.min_trail_size
                self.pointer.pensize(rad)
                self.pointer.goto(pos)
                self.pointer.down()
        self.pointer.up()

    def draw_cube(self, qube):
        """Method that draws a cube of octrees.

        Method that draws the corners and lines of octree nodes/cubes.
        Scales the line size by the distance scale factor f.

        Args:
            qube: Data of the cube corners, list of 8 points
            wich are lists of 3 floats.
                right front top, bottom
                right back top, bottom
                left front top, bottom
                left back top, bottom
        """
        for i in range(len(qube)):
            qube[i] = self.get_screen_xy(*qube[i])
        self.pointer.pencolor(self.cube_color)

        lines = [(3, 1), (1, 0), (0, 2),
                 (2, 3), (3, 7), (7, 6),
                 (6, 2), (4, 6), (7, 5),
                 (5, 1), (0, 4), (4, 5)]

        for line in lines:
            pos1 = qube[line[0]][0]
            pos2 = qube[line[1]][0]
            if pos1 and pos2 is not None:
                f = (qube[line[0]][1] + qube[line[1]][1]) / 2
                self.pointer.pensize(self.grid_thickness * f / 2)
                self.pointer.goto(pos1)
                self.pointer.down()
                self.pointer.goto(pos2)
                self.pointer.up()

    def draw_rot_cube(self):
        """Method that draws a rotational indicator.

        Method that draws three intersecting lines displaying the
        x, y and z axes. Highlight of part of axes that is geometrically
        closer to the screen.
        """
        coord  = ["X", "Y", "Z"]
        fig = []
        mx = self.width * self.rot_cube_pos[0] / 2
        my = self.height * self.rot_cube_pos[1] / 2

        for point in [[self.rot_cube_scale, 0, 0], [0, self.rot_cube_scale, 0], [0, 0, self.rot_cube_scale]]:
            line = []
            for i in range(2):
                if i == 0:
                    default_pos = np.array([[point[0]], [point[1]], [point[2]]])
                else:
                    default_pos = np.array([[-point[0]], [-point[1]], [-point[2]]])
                pos = self.rotation_matrix * default_pos
                f = self.FOV / (pos[2, 0] + (self.FOV))
                sx, sy = (pos[0, 0] * f) + (mx), (-pos[1, 0] * f) + (my)
                line.append([[sx, sy], f])
            fig.append(line)

        self.pointer.pencolor(self.rot_cube_scolor)

        for line in fig:
            pos_fore = 0
            if line[0][1] >= 1:  # positve side of axis in foreground, thicker border when drawing that side
                pos_fore = 1
            for n in range(2):
                self.pointer.pensize(2 + (4 * pos_fore))
                pos_fore = (pos_fore - 1) * -1
                self.pointer.goto(mx, my)
                self.pointer.down()
                self.pointer.goto(line[n][0])
                self.pointer.up()

        self.pointer.pencolor(self.rot_cube_lcolor)
        for i in range(len(fig)):
            self.pointer.goto(fig[i][0][0])
            self.pointer.write(coord[i], align="center", font=self.font)

    def draw_cm(self, cm_pos):
        pos, f = self.get_screen_xy(*cm_pos)
        if pos is not None:
            self.draw_circle(self.pointer, pos, self.cm_rad, self.cm_color, self.cm_color)

    def draw_circle(self, pointer, pos, rad, pencolor, fillcolor):
        pointer.goto(pos)
        pointer.fd(rad)
        pointer.left(90)
        pointer.fillcolor(fillcolor)
        pointer.pencolor(pencolor)
        pointer.down()
        pointer.begin_fill()
        pointer.circle(rad)
        pointer.end_fill()
        pointer.up()

    def get_size_factor(self, body_type):
        if body_type is Sun:
            return self.star_size_factor
        elif body_type is Planet:
            return self.planet_size_factor
        else:
            return 1
            print(f"No size indication for bodytype: {body_type}")

    def update_screen(self):
        """Method that updates the turtle canvas.

        Method that calls other functions and draws all data, bodies and 
        symbols onto the screen. Iterates over the active systems and computes 
        body positions, trails.
        """

        self.rotation_matrix = Rx(math.radians(self.x_rotation)) * Ry(math.radians(180 - self.y_rotation)) * Rz(math.radians(180 - self.z_rotation))
        self.finished = False
        self.pointer.clear()

        # onscreen((name,pos,radius,mass,color,trail,velocity,acceleration,density,force,2dpos,f,type))
        self.onscreen = []
        for system in self.simulations:
            self.solar_system = system
            bodies, nodes, system, cm = system.get_data()

            if self.map_colors:
                self.color_map_bodies(bodies)

            # get on screen planets and draw trails, creation of special onscreen datalist
            for body in bodies:
                body_pos, f = self.get_screen_xy(*body[1])
                body_size_factor = self.get_size_factor(body[12])

                if self.draw_trail:
                    self.draw_last_pos(body[5], body[4], body[2], body_size_factor)
                if body_pos is not None:
                    data = (*body, body_pos, f, body_size_factor)
                    self.onscreen.append(data)

        self.onscreen.sort(key=lambda element: element[14])
        for body in self.onscreen:
            body_pos    = body[13]
            radius      = body[2]
            color       = body[4]
            f           = body[14]
            body_size_factor = body[15]

            rad = radius * f * body_size_factor
            if rad < self.min_body_size:
                rad = self.min_body_size
            self.draw_circle(self.pointer, body_pos, rad, color, color)

        if self.show_cm:
            self.draw_cm(cm)

        if self.draw_box:
            for qube in nodes:
                self.draw_cube(qube)

        if self.show_time:
            self.draw_time(self.solar_system.time)

        if self.show_data:
            self.draw_data(system)

        if self.draw_rot:
            self.draw_rot_cube()

        if self.get_vid and not self.pause:
            self.getter(self.canvas)

        self.fenster.update()

        self.finished = True

    def color_map_bodies(self, data):
        """Method that changes a bodies color based on different factors.

        Method that iterates over all bodies and changes their color 
        by mapping the magintude of a certain characteristic like the mass 
        over a predetermined color list.

        Args:
            data: List of attributes of onscreen bodies.
        """
        dic = {"mass": 3, "radius": 2, "density": 8,
               "velocity": [6, self.max_velocity],
               "acceleration": [7, self.max_acceleration],
               "force": [9, self.max_force]}

        if self.color_attribute in ["mass", "radius", "density"]:
            data.sort(key=lambda element: element[dic[self.color_attribute]])
        elif not self.color_mode_abs and self.color_attribute in ["velocity", "acceleration", "force"]:
            data.sort(key=lambda element: math.sqrt(np.dot(element[dic[self.color_attribute][0]], element[dic[self.color_attribute][0]])))

        if not self.color_mode_abs or self.color_attribute in ["mass", "radius", "density"]:
            for i in range(len(data)):
                color = self.rainbow_rgb[int(i * ((len(self.rainbow_rgb) - 1) / len(data)))]
                data[i] = (data[i][0], data[i][1], data[i][2],
                           data[i][3], color, data[i][5], data[i][6],
                           data[i][7], data[i][8], data[i][9])

        else:
            for i in range(len(data)):
                relative_index = math.sqrt(np.dot(data[i][dic[self.color_attribute][0]], data[i][dic[self.color_attribute][0]])) / dic[self.color_attribute][1]
                index = int(len(self.rainbow_rgb) * relative_index)
                if index > len(self.rainbow_rgb) - 1:
                    index = len(self.rainbow_rgb) - 1
                color = self.rainbow_rgb[index]
                data[i] = (data[i][0], data[i][1], data[i][2],
                           data[i][3], color, data[i][5], data[i][6],
                           data[i][7], data[i][8], data[i][9])

    def random_simulation(self):
        """Method that creates data for a random simulation.

        Method that creates data for a random n-body simulation. Not very
        useful for solar system visualizations.
        """
        self.solar_system = Simulation(self.theta, self.restitution_coefficient, self.absolute_pos, self.focus_index)
        for i in range(self.number_stars + self.number_planets):
            position = ((random.randint(0, self.size) - (self.size / 2)) * 10**(-10),
                        (random.randint(0, self.size) - (self.size / 2)) * 10**(-10),
                        (random.randint(0, self.size) - (self.size / 2)) * 10**(-10))
            velocity = ((random.randint(0, self.max_velo) - (self.max_velo / 2)) * .15,
                        (random.randint(0, self.max_velo) - (self.max_velo / 2)) * .15,
                        (random.randint(0, self.max_velo) - (self.max_velo / 2)) * .15)
            if i > self.number_stars:
                name = "sun " + str(i + 1)
                mass = random.randint(1, 1000) / 100
                density = random.randint(100, 8000) / 1000
                color = random.choice(self.sun_colors)
                Sun(self.solar_system, name, mass, density, position, velocity, color, self.trail_node_number, self.trail_node_distance)
            else:
                name = "planet " + str(i + 1)
                mass = random.randint(1, 100000) * 10**(-7)
                density = random.randint(100, 10000) / 1000
                color = random.choice(self.planet_colors)
                Planet(self.solar_system, name, mass, density, position, velocity, color, self.trail_node_number, self.trail_node_distance)
        self.solar_system.set_focus(None)
        self.simulations = [self.solar_system]

    def setup_simulation(self):
        """Method that creates the simulations.

        Method that sets up and gets the starting data for simulations.
        """
        self.simulations = []
        for i in range(len(self.starting_data)):
            self.simulations.append(Simulation(self.theta, self.restitution_coefficient, self.absolute_pos, self.focus_index))

        if self.start_random:
            self.random_simulation()

        elif self.starting_data is not None:
            for i, system in enumerate(self.starting_data):
                for bodytype in system.keys():
                    for body in system[bodytype]:
                        if bodytype == "suns":
                            Sun(self.simulations[i], body[0], body[1], body[2], body[3], body[4], body[5], self.trail_node_number, self.trail_node_distance)
                        elif bodytype == "planets":
                            Planet(self.simulations[i], body[0], body[1], body[2], body[3], body[4], body[5], self.trail_node_number, self.trail_node_distance)
                self.simulations[i].set_focus(None)
        else:
            print("no starting data")

    def update_program(self):
        """Method that updates the main simulation loop.

        Method that calls update in system phsics and screen.
        """
        if not self.pause or self.frame_count == 0:
            self.frame_count += 1
            t1 = time.time()
            for system in self.simulations:
                system.calculate(self.timestep, self.draw_box, self.node_type)
            t2 = time.time()
            self.physics_time = t2 - t1

        t1 = time.time()

        self.update_screen()
        t2 = time.time()
        self.drawing_time = t2 - t1

        if self.frame_count == self.max_frame and self.get_vid:
            self.window.destroy()

        if self.benchmark:
            print(f"\
draw={self.drawing_time: .4f}, \
calc={self.physics_time: .4f}, \
pos/velo={self.time_position: .4f}, \
force={self.time_interactions: .4f}, \
node={self.time_nodes: .4f}")
# self.time_matrix, self.trail_sort, self.video, self.qube_data =
# rot={self.time_matrix: .4f}, planet={self.trail_sort: .4f}, dta/qube={self.qube_data: .4f}, \

        self.window.id = self.window.after(self.timepause, self.update_program)


def ConvertSectoDay(n):
    year = n // (365 * 24 * 3600)
    n %= (365 * 24 * 3600)
    day = n // (24 * 3600)
    n %= (24 * 3600)
    hour = n // 3600
    n %= 3600
    minutes = n // 60
    n %= 60
    seconds = n
    return (year, day, hour, minutes, seconds)


def Rx(theta):
    return np.matrix([[ 1, 0              , 0           ],
                      [ 0, math.cos(theta),-math.sin(theta)],
                      [ 0, math.sin(theta), math.cos(theta)]])
def Ry(theta):
    return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                      [ 0              , 1, 0           ],
                      [-math.sin(theta), 0, math.cos(theta)]])
def Rz(theta):
    return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                      [ math.sin(theta), math.cos(theta) , 0 ],
                      [ 0              , 0               , 1 ]])


def rgb_colors():
    """Method that creates an RGB rainbow list.

    Mehtod that creates an RGB rainbow list used for color mapping bodies.

    Returns:
        List of RGB values in a rainbow pattern.
    """
    rgb_list = []
    for i in range(round(255 / 2)):
        b = 0 + i
        g = round((255 / 2) + i)
        r = 255 - i
        rgb_list.append((r, g, b))
    for i in range(round(255 / 2)):
        b = round((255 / 2) + i)
        g = 255 - i
        r = 0 + i
        rgb_list.append((r, g, b))
    for i in range(round(255 / 2)):
        b = 255 - i
        g = 0 + i
        r = round((255 / 2) + i)
        rgb_list.append((r, g, b))
    return rgb_list


if __name__ == "__main__":
    Interface()
