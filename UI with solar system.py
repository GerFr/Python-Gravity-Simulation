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
        self.default_dist   = 2*10**11
        self.z_rotation     = 0
        self.frame_count    = 0
        

        self.mouse_click1   = False
        self.mouse_click2   = False
        self.mouse_click3   = False
        self.pause          = False
        self.finished       = True

        self.timestep       = 10000 #in seconds, "simulation-time" per frame
        self.timepause      = 50#pause between frames
        self.theta          = 1
        self.restitution_coefficient = 0 #inelastic collisions, if 1 fully elastic if 0 merge of planets


        self.absolute_pos  = False #relative motion doesnt seem to work for dynamik n body systems
        self.focus_options = ["none", "body", "cm"]
        self.focus     = self.focus_options[1] #faster if true, no data on screen
        #self.rigid_box      = False -> not implemented, could run faster

        

        self.path_color     = "darkgrey"
        self.bg_color       = "black"
        self.text_color     = "white"
        self.font           = ("Courier New",15, "normal")
        self.cube_color     = "green"

        self.bodie_size_factor = 4 #accurate:1

        self.draw_trail     = True
        self.draw_box       = False
        self.path_size      = .2
        trail_length        = 1000#2
        trail_resolution    = .5 #0-1
        self.trail_node_number   = int(trail_length * trail_resolution)
        self.trail_node_distance = int(trail_length / self.trail_node_number) #amount of calculations between each node

        self.pointer_size   = 50
        self.onscreen       = []

        self.image_folder   = "C:/Users/gerri/Desktop/data"
        self.get_vid        = False
        self.max_frame      = 10000


        self.draw_rot       = True
        self.rot_cube_pos   = [.8, -.8]#zwischen -1,1, scale of screen
        self.rot_cube_scale = 80
        self.rot_cube_scolor= "grey"
        self.rot_cube_lcolor= "white"

        self.show_cm        = True
        self.cm_rad         = 10
        self.cm_color       = "white"

        
        self.map_colors     = True
        self.rainbow_rgb    = rgb_farben()
        self.color_attribute = "acceleration"
        self.max_acceleration = .004
        self.max_velocity   = 5*10**5
        self.max_force      = 9,10**27
        self.color_mode_abs = True
        
        
        
                         #mass,density, position,velocity,      color data from the web
        self.starting_data  = {'suns':    [('Sun', 1, 1.41, (0, 0, 0), (0, 0,0), 'yellow')],
                              'planets': [('Earth', 3.003*10**(-6),  5.5, (-1, 0, 0), (0, 1.992007*10**(-7), 0), 'lightgreen'),
                                          ('Mercury',  1.651*10**(-7),  5.43, (-0.4, 0, 0), (0, 3.1658205*10**(-7),0 ), 'green')]}

        #3 body problem
        speed = 2*10**(-9)
        size = 10**(-2)
        
        self.starting_data = {'planets': [('1', 3*10**(-6),  5.5, (0, size/2, 0), (-speed, 0, 0), 'green'),
                                          ('2', 5*10**(-6),  5.5, (-math.tan(math.radians(30))*size, -size/2, -size), (math.sin(math.radians(30))*speed,-math.cos(math.radians(30))*speed, 0), 'green'),
                                          ('3', 8*10**(-6),  5.5, (math.tan(math.radians(30))*size, -size/2, size), (math.cos(math.radians(60))*speed,math.sin(math.radians(60))*speed,0), 'green')]}

        

        #Values for random creation
        self.start_random   = False
        self.size           = 50*10000000000 #in 10**(-10) au, len of cube size, has to be big number for randint
        self.max_velo       = 5000#2 #in 10^-10 AU/s
        self.number_stars   = 100
        self.number_planets = 0 #trail resolution anpassen
        self.planet_colors  = ["beige", "lightgreen", "lightblue"]
        self.sun_colors     = ["yellow","orange","red"]
        
        #mass ranges
        #density ranges

        
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

        self.mouse = turtle.RawTurtle(self.fenster)
        self.mouse.up()
        self.mouse.ht()
        self.mouse.color(self.text_color)

        self.time_pointer = turtle.RawTurtle(self.fenster)
        self.time_pointer.up()
        self.time_pointer.ht()
        self.time_pointer.color(self.text_color)

        self.canvas.bind("<MouseWheel>", self.mouse_scroll)
        self.canvas.bind("<Motion>", self.mouse_hover)
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
        if not self.absolute_pos and not self.solar_system.focused_body in ["none","cm"]:
            self.solar_system.clear_trail()
        self.mouse_hover(None)
        
    
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
        


#onscreen((name,pos,radius,mass,color,trail,velocity,acceleration,density,force,2dpos,f))
    def mouse_hover(self, event):
        found = False
        if event != None:
            
            x =  event.x-(self.width/2)
            y = -event.y+(self.height/2)-30
            for body in self.onscreen:
                pos_x = body[10][0]
                pos_y = body[10][1] 
                pointer_range =  body[11]/10 * self.pointer_size
                if x > (pos_x - pointer_range) and x < (pos_x + pointer_range) and y > (pos_y - pointer_range) and y < (pos_y + pointer_range):
                    found = True
                    name    = body[0]
                    pos     = body[1]
                    radius  = body[2]
                    mass    = body[3]
                    velocity= body[6]
                    acceleration = body[7]
                    density = body[8]
                    force = body[9]
                    
        elif self.solar_system.focused_body not in self.focus_options:#body is a body
            found = True
            hover_body = self.solar_system.focused_body
            name    = hover_body.name
            pos     = hover_body.position
            radius  = hover_body.radius
            mass    = hover_body.mass
            velocity= hover_body.velocity
            acceleration = hover_body.acceleration
            density = hover_body.density
            force = hover_body.force
            if not self.absolute_pos:
                pos -= self.solar_system.focused_body.position

        

        if found:
           
            vectors = [pos, velocity, acceleration, force]
            for i in range(len(vectors)):
                vectors[i] = f"{vectors[i][0]:.2e}, {vectors[i][1]:.2e}, {vectors[i][2]:.2e}"
                
                    
            text =  f"""Name:         {name}\nRadius:       {radius:.2e}\nMass:         {mass:.2e}\nDensity:      {density}\nPosition:     {vectors[0]}\nVelocity:     {vectors[1]}\nAcceleration: {vectors[2]}\nForce:        {vectors[3]}\n"""
            self.mouse.clear()
            self.mouse.goto(self.width*-0.46, self.height*0.25)
            self.mouse.write(text, align="left", font=self.font)
            self.fenster.update()

                    
    def draw_time(self,time):
        self.time_pointer.goto(self.width*-0.46, -self.height*0.42)
        times = ConvertSectoDay(self.solar_system.time)                                                                                                                                                         
        text = f"years:   {times[0]}\ndays:    {times[1]}\nhours:   {times[2]}\nminutes: {times[3]}\nseconds: {times[4]}"
        self.time_pointer.clear()
        self.time_pointer.write(text, align="left", font=self.font)
        
        

    def reset(self):
        self.FOV        = self.fov_range[2]
        self.distance   = self.default_dist
        self.x_offset   = 0
        self.y_offset   = 0
        self.x_rotation = 0
        self.y_rotation = self.y_rot_range[2]

        #self.update_vertices()
        


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


    def draw_last_pos(self, last_pos):
        last_screen_pos = []
        for pos in last_pos:
            s_pos, f = self.get_screen_xy(*pos)
            last_screen_pos.append((s_pos, f))
        
        self.pointer.pencolor(self.path_color)
        for posf in last_screen_pos:
            pos = posf[0]
            f   = posf[1]
            
            if pos == None:
                self.pointer.up()
            else:
                self.pointer.pensize(self.path_size*f)
                self.pointer.goto(pos)
                self.pointer.down()
                
        self.pointer.up()

    def draw_cube(self, qube):
        for i in range(len(qube)):
            qube[i] = self.get_screen_xy(*qube[i])
            #qube[i] = {(sx,sy), f}
        self.pointer.pencolor(self.cube_color)
        
        seiten = [(3,1),(1,0),(0,2),(2,3),(3,7),(7,6),(6,2),(4,6),(7,5),(5,1),(0,4),(4,5)]#die seiten des würfels ald index der qube liste
        for seite in seiten:
            pos1 = qube[seite[0]][0]
            pos2 = qube[seite[1]][0]
            if pos1 and pos2 != None:
                f = (qube[seite[0]][1]+qube[seite[1]][1])/2
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
            self.draw_circle(pos, self.cm_rad, self.cm_color, self.cm_color)



    def draw_circle(self, pos, rad, pencolor, fillcolor):
        self.pointer.goto(pos)
        self.pointer.fd(rad)
        self.pointer.left(90)
        self.pointer.fillcolor(fillcolor)
        self.pointer.pencolor(pencolor)
        self.pointer.down()
        self.pointer.begin_fill()
        self.pointer.circle(rad)
        self.pointer.end_fill()
        self.pointer.up()
            

    def update_vertices(self):
        data, nodes, cm_pos = self.solar_system.get_data()
        
        t1a = time.time()
        self.rotation_matrix = Rx(math.radians(self.x_rotation)) * Ry(math.radians(180-self.y_rotation)) * Rz(math.radians(180-self.z_rotation))
        self.finished = False
        t2a = time.time()

        #onscreen((name,pos,radius,mass,color,trail,velocity,acceleration,density,force,2dpos,f))
        
        t1b = time.time()
        self.pointer.clear()
        
        if self.map_colors:
            self.color_map_bodies(data)

        #get on screen planets and draw trails, creation of special onscreen datalist
        self.onscreen = []
        for body in data:
            body_pos, f = self.get_screen_xy(*body[1])
            if self.draw_trail:
                self.draw_last_pos(body[5])
            if body_pos != None:
                self.onscreen.append((*body, body_pos, f))


        self.onscreen.sort(key=lambda element: element[11])#sort by f
        for body in self.onscreen:
            body_pos    = body[10]
            radius      = body[2]
            color       = body[4]
            f           = body[11]
            rad = radius*f*self.bodie_size_factor
            if rad <.5:
                rad =.5
            self.draw_circle(body_pos, rad, color, color)
        t2b = time.time()

        if self.show_cm:
            self.draw_cm(cm_pos)

        t1c = time.time()
        if self.get_vid and not self.pause:
            self.getter(self.canvas)
        t2c = time.time()

        t1d = time.time()
        if self.draw_box:
            for qube in nodes:
                self.draw_cube(qube)
        if self.draw_rot:
            self.draw_rot_cube()
        self.draw_time(self.solar_system.time)
        self.mouse_hover(None)
        t2d = time.time()


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
          
        self.starting_data = {}
        suns = []
        self.starting_data["suns"] = []
        for i in range(self.number_stars):
            name = "sun "+str(i+1)
            mass = random.randint(1,1000)/100 #0.01-10
            density = random.randint(100,8000)/1000 #0.1-8
            position =((random.randint(0,self.size)-(self.size/2))*10**(-10),
                       (random.randint(0,self.size)-(self.size/2))*10**(-10),
                       (random.randint(0,self.size)-(self.size/2))*10**(-10))
            velocity = ((random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10))
            color = random.choice(self.sun_colors)
            self.starting_data["suns"] += [(name,mass,density,position,velocity,color)]
            suns.append(Sun(self.solar_system,name,mass,density,position,velocity,color,self.trail_node_number,self.trail_node_distance))

        planets = []
        self.starting_data["planets"] = []
        for i in range(self.number_planets):
            name = "planet "+str(i+1)
            mass = random.randint(1,100000)*10**(-7) #10**(-7) <--> 10**(-2)
            density = random.randint(100,10000)/1000
            position =((random.randint(0,self.size)-(self.size/2))*10**(-10),
                       (random.randint(0,self.size)-(self.size/2))*10**(-10),
                       (random.randint(0,self.size)-(self.size/2))*10**(-10))
            velocity = ((random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10))
            color = random.choice(self.planet_colors)
            self.starting_data["planets"] += [(name,mass,density,position,velocity,color)]
            planets.append(Planet(self.solar_system,name,mass,density,position,velocity,color,self.trail_node_number,self.trail_node_distance))

        if self.focus == self.focus_options[0] or self.focus == self.focus_options[2]:
            self.solar_system.set_focus(self.focus)
        else:
            #sun with the biggest mass
            suns.sort(key=lambda sun: sun.mass)
            self.solar_system.set_focus(random.choice(suns))
        

    def setup_solar_system(self):
        self.solar_system = SolarSystem()

        if self.start_random or self.starting_data == None:
            self.random_system()

        elif self.starting_data != None:
            self.solar_system = SolarSystem()
            suns =[]
            planets = []
            for bodytype in self.starting_data.keys():
                for body in self.starting_data[bodytype]:
                    if bodytype == "suns":
                        suns.append(Sun(self.solar_system,body[0],body[1],body[2],body[3],body[4],body[5],self.trail_node_number, self.trail_node_distance))
                    elif bodytype == "planets":
                        planets.append(Planet(self.solar_system,body[0],body[1],body[2],body[3],body[4],body[5],self.trail_node_number, self.trail_node_distance))

            if self.focus == self.focus_options[0] or self.focus == self.focus_options[2]:
                self.solar_system.set_focus(self.focus)
            else:
                self.solar_system.set_focus(planets[0])

        else:
            print("no starting data")
        print(self.starting_data)
        self.mouse_hover(None)
        self.solar_system.absolute_pos = self.absolute_pos
    
    

    def update_system(self):
        if not self.pause or self.frame_count == 0:
            t1 = time.time()
            self.time_position, self.time_interactions, self.time_nodes = self.solar_system.calculate(self.timestep, self.theta, self.restitution_coefficient, self.draw_box)
            t2 = time.time()
            self.physics_time = t2 - t1

        t1 = time.time()
        self.time_matrix, self.trail_sort, self.video, self.qube_data = self.update_vertices()
        t2 = time.time()
        #print(f"epot: {self.solar_system.total_epot:.5e}, ekin: {self.solar_system.total_ekin:.5e}, virality: {(2*self.solar_system.total_ekin+self.solar_system.total_epot):.5e}, etotal: {self.solar_system.total_e:.5e}")
        self.drawing_time = t2 - t1
        self.frame_count += 1
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
