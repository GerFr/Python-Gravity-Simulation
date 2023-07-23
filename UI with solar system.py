#tkinter 3d gfx interface

import math
import tkinter
import turtle
import time
from SolarSystem import SolarSystem, Planet, Sun
import numpy as np
import tinyarray as ta
import random
from PIL import ImageGrab

#fix bugs in relative movement barnes hut
#upgrade ui, speed





class Interface():
    def __init__(self):

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

        self.timestep       = 60000 #in seconds, "simulation-time" per frame
        self.timepause      = 50#pause between frames
        self.theta          = 1
        self.restitution_coefficient = 0 #inelastic collisions, if 1 fully elastic if 0 merge of planets


        self.absolute_pos   = True         
        self.focus_index    = 0#["none","body","cm"] 0,1,2
        #self.rigid_box      = False -> not implemented, could run faster

        self.show_data      = False
        self.show_time      = True
        self.draw_frame_count = False

        

        self.path_color     = "darkgrey"
        self.bg_color       = "black"
        self.text_color     = "white"
        self.font           = ("Courier New",15, "normal")
        self.cube_color     = "green"

        self.body_size_factor = 1
        self.min_body_size = 8

        self.draw_box       = False
        self.draw_trail     = True
        self.path_size      = 1.5*10**9
        self.trail_length        = 1000
        trail_resolution    = .5 #0-1
        self.path_planet_color = True
        self.start_thickness= 3
        self.thickness_scale=.9
        
        self.trail_node_number   = int(self.trail_length * trail_resolution)
        self.trail_node_distance = int(self.trail_length / self.trail_node_number) #amount of calculations between each node

        self.pointer_size   = 50
        self.onscreen       = []

        self.image_folder   = "C:/Users/gerri/Desktop/data"
        self.get_vid        = False
        self.max_frame      = 3600


        self.draw_rot       = True
        self.rot_cube_pos   = [.8, -.8]#zwischen -1,1, scale of screen
        self.rot_cube_scale = 80
        self.rot_cube_scolor= "grey"
        self.rot_cube_lcolor= "white"

        self.show_cm        = False
        self.cm_rad         = 10
        self.cm_color       = "white"

        
        self.map_colors     = False
        self.rainbow_rgb    = rgb_farben()
        self.color_attribute = "acceleration"
        self.max_acceleration = .1
        self.max_velocity   = 5*10**5
        self.max_force      = 9,10**27
        self.color_mode_abs = True
        
        
        
                        
##        #6
##        speed = 4*10**(-9)
##        size = 10**(-2)
##        self.starting_data = []
##        weights = ta.array([5,8,5,8,5,8])
##        offset = [.9995,1,1.0005,.9995,1,1.0005]
##        nr_systems = 500
##        for i in range(nr_systems):
##            color = self.rainbow_rgb[int((i/(nr_systems))*len(self.rainbow_rgb))]
##            self.starting_data.append({'planets': [('1', weights[0]*10**(-6),  5.5, (0, size/2, size/2), (-speed, 0, 0), color),
##                                                   ('2', weights[1]*10**(-6),  5.5, (-math.tan(math.radians(30))*size, -size/2, size/2), (math.sin(math.radians(30))*speed,-math.cos(math.radians(30))*speed, 0), color),
##                                                   ('3', weights[2]*10**(-6),  5.5, (math.tan(math.radians(30))*size, -size/2, size/2), (math.cos(math.radians(60))*speed,math.sin(math.radians(60))*speed,0), color),
##                                                   ('1', weights[3]*10**(-6),  5.5, (0, size/2, -size/2), (speed, 0, 0), color),
##                                                   ('2', weights[4]*10**(-6),  5.5, (-math.tan(math.radians(30))*size, -size/2, -size/2), (-math.sin(math.radians(30))*speed, math.cos(math.radians(30))*speed, 0), color),
##                                                   ('3', weights[5]*10**(-6),  5.5, (math.tan(math.radians(30))*size, -size/2, -size/2), (-math.cos(math.radians(60))*speed, -math.sin(math.radians(60))*speed,0), color)]})
##            weights *= offset





        #mass,density, position,velocity,      color data from the web
        self.starting_data  = [{'suns':    [('Sun', 1, 1.41, (0, 0, 0), (0, 0,0), 'yellow')],
                              'planets': [('Earth', 3.003*10**(-6),  5.5, (-1, 0, 0), (0, 1.992007*10**(-7), 0), 'lightgreen'),
                                          ('Mercury',  1.651*10**(-7),  5.43, (-0.4, 0, 0), (0, 3.1658205*10**(-7),0 ), 'orange')]}]


            
        #Values for random creation
        self.start_random   = False
        self.size           = 50*10000000000 #in 10**(-10) au, len of cube size, has to be big number for randint
        self.max_velo       = 5000#2 #in 10^-10 AU/s
        self.number_stars   = 100
        self.number_planets = 0 #trail resolution anpassen
        self.planet_colors  = ["beige", "lightgreen", "lightblue"]
        self.sun_colors     = ["yellow","orange","red"]
        
        
        #Benchmarking
        self.benchmark      = False
        self.setup_canvas()
        self.setup_solar_system()
        self.reset()
        self.window.id = self.window.after(self.timepause, self.update_system)
        self.window.mainloop()
        

        

    def setup_canvas(self):
        self.window = tkinter.Tk()
        self.window.title("Gravity Simulation")
        self.window.attributes("-fullscreen", True)
        window_size_tuple = self.window.maxsize()
        self.width = window_size_tuple[0]
        self.height= window_size_tuple[1]

        upper_grid          = tkinter.Frame(self.window)
        button_schließen    = tkinter.Button(upper_grid, text  = "leave", command = self.window.destroy)
        self.pause_button   = tkinter.Button(upper_grid, text  = "pause", command = self.toggle_pause)
        self.reset_button   = tkinter.Button(upper_grid, text  = "reset", command = self.reset)
        self.canvas         = tkinter.Canvas(self.window, width = window_size_tuple[0], height = window_size_tuple[1])
        upper_grid.pack             (side=tkinter.TOP, fill = tkinter.BOTH, expand = True)
        button_schließen.grid       (column = 0, row = 0, sticky = tkinter.NSEW)
        self.pause_button.grid      (column = 1, row = 0, sticky = tkinter.NSEW)
        self.reset_button.grid      (column = 2, row = 0, sticky = tkinter.NSEW)
        self.canvas.pack            (side=tkinter.TOP, fill = tkinter.BOTH, expand = True)

        for i in range (3):
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
        self.window.bind("<Left>",  lambda event: self.switch_focus(event,"right"))
        self.window.bind("<Right>", lambda event: self.switch_focus(event,"left"))
        self.window.bind("<Up>",    lambda event: self.switch_focus(event,"right"))
        self.window.bind("<Down>",  lambda event: self.switch_focus(event,"left"))

    def getter(self, widget):
        x=self.window.winfo_rootx()+widget.winfo_x()
        y=self.window.winfo_rooty()+widget.winfo_y()
        x1=x+widget.winfo_width()
        y1=y+widget.winfo_height()
        ImageGrab.grab().crop((x,y,x1,y1)).save(f"{self.image_folder}/frame_{self.frame_count}.gif")
        print(self.frame_count)



    def switch_focus(self,event,direction):
        if direction == "left":
            self.solar_system.switch_focus("previous")
        elif direction == "right":
            self.solar_system.switch_focus("next")
        if not self.absolute_pos and self.solar_system.focus_type == "body":
            self.solar_system.clear_trail()
        
    
    def toggle_pause(self):
        if self.pause == False:
            self.pause = True
            self.pause_button.config(text = "start")
        else:
            self.pause = False
            self.pause_button.config(text = "pause")
            
            
    def mouse_off(self,event, button):
        if button == "b1":
            self.mouse_click1 = False
        elif button == "b2":
            self.mouse_click2 = False
        elif button == "b3":
            self.mouse_click3 = False
        

    def change_fov (self, event):
        if not self.mouse_click2:
            self.old_y = event.y
            self.mouse_click2 = True
        elif self.mouse_click2 and self.finished:
            self.FOV -= (event.y - self.old_y)*10
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
            self.y_rotation += (event.x - self.old_x)/10
            self.x_rotation += (event.y - self.old_y)/10
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
            self.x_offset += (event.x - self.old_x)*self.distance/1000
            self.y_offset += (event.y - self.old_y)*self.distance/1000
            self.old_x = event.x
            self.old_y = event.y

      
    def mouse_scroll(self, event):
        self.distance -= (event.delta)*self.distance/1000
        if self.distance <0:
            self.distance = 0
        

    #[self.focus_type, self.focused_body, self.absolute_pos, self.theta, self.restitution_coefficient, self.total_ekin, self.total_epot, self.total_e, cm_pos,  self.iteration, len(self.bodies), self.tree.root_node.middle, self.tree.root_node.dimension]]

    def draw_data(self, data):
        y_pos = .28
        text = ""
        if data[1] != None: #focus on body
            y_pos = .08
            hover_body = data[1]
            name    = hover_body.name
            pos     = hover_body.position
            radius  = hover_body.radius
            mass    = hover_body.mass
            velocity= hover_body.velocity
            acceleration = hover_body.acceleration
            density = hover_body.density
            force = hover_body.force
            if not self.absolute_pos:
                pos -= hover_body.position
        
            vectors = [pos, velocity, acceleration, force]
            for i in range(len(vectors)):
                vec_mag = math.sqrt(ta.dot(vectors[i],vectors[i]))
                vectors[i] = f"mag:{vec_mag:.2e} vec:({vectors[i][0]:.2e},{vectors[i][1]:.2e},{vectors[i][2]:.2e})"
            text +=  f"""Name:         {name}\nRadius:       {radius:.2e}\nMass:         {mass:.2e}\nDensity:      {density}\nPosition:     {vectors[0]}\nVelocity:     {vectors[1]}\nAcceleration: {vectors[2]}\nForce:        {vectors[3]}\n"""

        if data[2]:
            view = "absolute"
        else:
            view = "relative"
        
        cm_pos = f"vec:({data[8][0]:.2e},{data[8][1]:.2e},{data[8][2]:.2e})"
        text += f"""Focus type:   {data[0]}\nView:         {view}\nTheta:        {data[3]}\nR.Coefficient:{data[4]}\nEkin:         {data[5]:.2e}\nEpot:         {data[6]:.2e}\nEtotal:       {data[7]:.2e}\nCenter of m.: {cm_pos}\nIteration:    {data[9]:.2e}\nNr. Bodies    {data[10]:.2e}\nRoot n. size: {data[12]:.2e}\n"""

        self.data_pointer.goto(self.width*-0.46, self.height*y_pos)
        self.data_pointer.write(text, move = True, align="left", font=self.font)
            

                    
    def draw_time(self,time):
        self.data_pointer.goto(self.width*-0.46, -self.height*0.42)
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
        self.y_rotation = self.y_rot_range[2]+self.start_y_rot
        


    def get_screen_xy(self,x,y,z):
            default_pos = np.array([[x],[y],[z]])
            pos = self.rotation_matrix * default_pos

            x = pos[0,0]
            y = pos[1,0]
            z = pos[2,0]
            
            real_dist = self.distance*(self.FOV/1000) 
            x += self.x_offset
            y += self.y_offset
            z += real_dist

            if z > 0:
                f = self.FOV / z
                sx, sy = x * f, y * f
                return (sx, -sy), f
            else:
                return None, None


    def draw_last_pos(self, last_pos, color):
        
        last_screen_pos = []

        last_pos.reverse()
        thickness = self.start_thickness
        scale = self.thickness_scale
        
        for i, pos in enumerate(last_pos):
            thickness *= scale
            
            s_pos, f = self.get_screen_xy(*pos)
            last_screen_pos.append((s_pos, f, thickness))


        last_pos.reverse()
        self.pointer.pencolor(self.path_color)
        if self.path_planet_color:
            self.pointer.pencolor(color)
        for posf in last_screen_pos:
            pos = posf[0]
            f   = posf[1]
            thickness = posf[2]
            
            if pos == None:
                self.pointer.up()
            else:
                self.pointer.pensize(self.path_size*f*thickness)
                self.pointer.goto(pos)
                self.pointer.down()
                
        self.pointer.up()

    def draw_cube(self, qube):
        for i in range(len(qube)):
            qube[i] = self.get_screen_xy(*qube[i])
            #qube[i] = {(sx,sy), f}
        self.pointer.pencolor(self.cube_color)
        
        lines = [(3,1),(1,0),(0,2),(2,3),(3,7),(7,6),(6,2),(4,6),(7,5),(5,1),(0,4),(4,5)]#die seiten des würfels ald index der qube liste
        for line in lines:
            pos1 = qube[line[0]][0]
            pos2 = qube[line[1]][0]
            if pos1 and pos2 != None:
                f = (qube[line[0]][1]+qube[line[1]][1])/2
                self.pointer.pensize(self.path_size*f/2)
                self.pointer.goto(pos1)
                self.pointer.down()
                self.pointer.goto(pos2)
                self.pointer.up()

    def draw_rot_cube(self):
        coord  =["X", "Y", "Z"]
        fig = []
        mx = self.width*self.rot_cube_pos[0]/2
        my = self.height*self.rot_cube_pos[1]/2
        
        for point in [[self.rot_cube_scale,0,0],[0,self.rot_cube_scale,0],[0,0,self.rot_cube_scale]]:
            line = []
            for i in range(2):
                if i == 0:
                    default_pos = np.array([[point[0]],[point[1]],[point[2]]])
                else:
                    default_pos = np.array([[-point[0]],[-point[1]],[-point[2]]])
                pos = self.rotation_matrix * default_pos
                f = self.FOV / (pos[2,0]+(self.FOV))
                sx, sy = (pos[0,0]*f)+(mx), (-pos[1,0]*f)+(my)
                line.append([[sx,sy], f])
            fig.append(line)

        self.pointer.pencolor(self.rot_cube_scolor)
        
        for line in fig:
            pos_fore = 0 
            if line[0][1] >=1:  #positve side of axis in foreground, thicker border when drawing that side
                pos_fore = 1
            for n in range(2):
                self.pointer.pensize(2+(4*pos_fore))
                pos_fore = (pos_fore-1)*-1
                self.pointer.goto(mx,my)
                self.pointer.down()
                self.pointer.goto(line[n][0])
                self.pointer.up()

        self.pointer.pencolor(self.rot_cube_lcolor)
        for i in range(len(fig)):
            self.pointer.goto(fig[i][0][0])
            self.pointer.write(coord[i], align="center", font=self.font)

    def draw_cm(self,cm_pos):
        pos, f = self.get_screen_xy(*cm_pos)
        if pos != None:
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
            

    def update_vertices(self):
        
        
        t1a = time.time()
        self.rotation_matrix = Rx(math.radians(self.x_rotation)) * Ry(math.radians(180-self.y_rotation)) * Rz(math.radians(180-self.z_rotation))
        self.finished = False
        t2a = time.time()
        self.pointer.clear()
        
        #onscreen((name,pos,radius,mass,color,trail,velocity,acceleration,density,force,2dpos,f))
        self.onscreen = []
        for system in self.solar_systems:
            self.solar_system = system
            bodies, nodes, system, cm = system.get_data()
        
            t1b = time.time()
            
            
            if self.map_colors:
                self.color_map_bodies(bodies)

            #get on screen planets and draw trails, creation of special onscreen datalist
            
            for body in bodies:
                body_pos, f = self.get_screen_xy(*body[1])
                if self.draw_trail:
                    self.draw_last_pos(body[5],body[4])
                if body_pos != None:
                    data = (*body, body_pos, f)
                    self.onscreen.append(data)


        self.onscreen.sort(key=lambda element: element[11])#sort by f
        for body in self.onscreen:
            body_pos    = body[12]
            radius      = body[2]
            color       = body[4]
            f           = body[13]
            rad = radius*f*self.body_size_factor
            if rad <self.min_body_size:
                rad =self.min_body_size
            self.draw_circle(self.pointer, body_pos, rad, color, color)
        t2b = time.time()
        
        if self.show_cm:
            self.draw_cm(cm)

        if self.draw_box:
            for qube in nodes:
                self.draw_cube(qube)

        if self.show_data:
            self.draw_data(system)
                




        if self.show_time:
            self.draw_time(self.solar_system.time)
        

        t1d = time.time()      
        if self.draw_rot:
            self.draw_rot_cube()

        t1c = time.time()
        if self.get_vid and not self.pause:
            self.getter(self.canvas)
        t2c = time.time()

        
        t2d = time.time()
        
        self.fenster.update()


        self.finished = True

        
            
        return t2a-t1a, t2b-t1b, t2c-t1c, t2d-t1d

    

    def color_map_bodies(self, data):#relative colors 
        
        dic = {"mass":3,"radius":2,"density":8, "velocity":[6, self.max_velocity],"acceleration":[7,self.max_acceleration],"force":[9,self.max_force]}

        if self.color_attribute in ["mass", "radius", "density"]:
            data.sort(key=lambda element:  element[dic[self.color_attribute]])
        elif not self.color_mode_abs and self.color_attribute in ["velocity","acceleration","force"]:
            data.sort(key=lambda element:  math.sqrt(ta.dot(element[dic[self.color_attribute][0]],element[dic[self.color_attribute][0]])))#magnitude of vector

        if not self.color_mode_abs or self.color_attribute in ["mass", "radius", "density"]:
            for i in  range(len(data)):
                color = self.rainbow_rgb[int(i * ((len(self.rainbow_rgb)-1)/len(data)))]
                data[i] = (data[i][0],data[i][1],data[i][2],data[i][3],color,data[i][5],data[i][6],data[i][7],data[i][8],data[i][9])

        else:
            for i in range(len(data)):
                relative_index = math.sqrt(ta.dot(data[i][dic[self.color_attribute][0]],data[i][dic[self.color_attribute][0]]))/dic[self.color_attribute][1]
                index = int(len(self.rainbow_rgb)*relative_index)
                if index > len(self.rainbow_rgb)-1:
                    index= len(self.rainbow_rgb)-1
                color = self.rainbow_rgb[index]
                data[i] = (data[i][0],data[i][1],data[i][2],data[i][3],color,data[i][5],data[i][6],data[i][7],data[i][8],data[i][9])

            
            


        

        

    def random_system(self):
        '''{'suns':    [('sun 1', 1, 1.41, (0, 0, 0), (0, 0,0), 'yellow')],
            'planets': [('Earth', 3.003*10**(-6),  5.5, (-1, 0, 0), (0, 0, 1.992007*10**(-7)), 'lightgreen'),
                        ('Mercury',  1.651*10**(-7),  5.43, (-0.4, 0, 0), (0, 0, 3.1658205*10**(-7)), 'green')]}'''
          
        for i in range(self.number_stars+self.number_planets):
            position =((random.randint(0,self.size)-(self.size/2))*10**(-10),
                       (random.randint(0,self.size)-(self.size/2))*10**(-10),
                       (random.randint(0,self.size)-(self.size/2))*10**(-10))
            velocity = ((random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10))
            if i > self.number_stars:
                name = "sun "+str(i+1)
                mass = random.randint(1,1000)/100 #0.01-10
                density = random.randint(100,8000)/1000 #0.1-8
                color = random.choice(self.sun_colors)
                Sun(self.solar_system,name,mass,density,position,velocity,color,self.trail_node_number,self.trail_node_distance)
            else:
                name = "planet "+str(i+1)
                mass = random.randint(1,100000)*10**(-7) #10**(-7) <--> 10**(-2)
                density = random.randint(100,10000)/1000
                color = random.choice(self.planet_colors)
                Planet(self.solar_system,name,mass,density,position,velocity,color,self.trail_node_number,self.trail_node_distance)
                

    def setup_solar_system(self):
        self.solar_systems = []
        for i in range(len(self.starting_data)):
            self.solar_systems.append(SolarSystem(self.theta,self.restitution_coefficient,self.absolute_pos,self.focus_index))

        if self.start_random:
            self.random_system()

        elif self.starting_data != None:
            for i, system in enumerate(self.starting_data):
                for bodytype in system.keys():
                    for body in system[bodytype]:
                        if bodytype == "suns":
                            Sun(self.solar_systems[i],body[0],body[1],body[2],body[3],body[4],body[5],self.trail_node_number, self.trail_node_distance)
                        elif bodytype == "planets":
                            Planet(self.solar_systems[i],body[0],body[1],body[2],body[3],body[4],body[5],self.trail_node_number, self.trail_node_distance)
                self.solar_systems[i].set_focus(None)
        else:
            print("no starting data")
        
    
    

    def update_system(self):
        if not self.pause or self.frame_count == 0:
            self.frame_count += 1
            t1 = time.time()
            for system in self.solar_systems:
                self.time_position, self.time_interactions, self.time_nodes = system.calculate(self.timestep, self.draw_box)
            t2 = time.time()
            self.physics_time = t2 - t1

        t1 = time.time()
        self.time_matrix, self.trail_sort, self.video, self.qube_data = self.update_vertices()
        t2 = time.time()
        self.drawing_time = t2 - t1
        
        if self.frame_count == self.max_frame and self.get_vid:
            self.window.destroy()

        if self.benchmark:
            print(f"draw={self.drawing_time: .4f}, rot={self.time_matrix: .4f}, planet={self.trail_sort: .4f}, dta/qube={self.qube_data: .4f}, calc={self.physics_time: .4f}, pos/velo={self.time_position: .4f}, force={self.time_interactions: .4f}, node={self.time_nodes: .4f}")

        
        self.window.id = self.window.after(self.timepause, self.update_system)


def ConvertSectoDay(n):
    year = n // (365*24*3600)
    n %= (365 * 24 * 3600)
    day = n // (24 * 3600)
    n %= (24 * 3600)
    hour = n // 3600
    n %= 3600
    minutes = n // 60
    n %= 60
    seconds = n
	
    return (year,day,hour,minutes,seconds)


def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])
def Rz(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])
def rgb_farben():
    rgb_list = []
    for i in range(round(255/2)):
        b = 0+i
        g = round((255/2)+i)
        r = 255-i
        rgb_list.append((r,g,b))
    for i in range(round(255/2)):
        b = round((255/2)+i)
        g = 255-i
        r = 0+i
        rgb_list.append((r,g,b))
    for i in range(round(255/2)):
        b = 255-i
        g = 0+i
        r = round((255/2)+i)
        rgb_list.append((r,g,b))
    return rgb_list

Interface()
