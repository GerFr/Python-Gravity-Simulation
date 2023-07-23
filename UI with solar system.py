#tkinter 3d gfx interface

import math
import tkinter
import turtle
import time
from SolarSystem import SolarSystem, Planet, Sun
import numpy as np
import random
from PIL import ImageGrab

#slider length factor einbauen
#better vid production
#fix bugs in relative movement barnes hut
#upgrade ui, speed





class Interface():
    def __init__(self):

        self.fov_range      = (200, 4000, 2000)
        self.y_rot_range    = (-90, 90, 0)
        self.default_dist   = 10**12
        self.z_rotation     = 0
        self.frame_count    = 0
        

        self.mouse_click1   = False
        self.mouse_click2   = False
        self.mouse_click3   = False
        self.pause          = False
        self.finished       = True

        self.timestep       = 18000 #5 min
        self.timepause      = 50
        self.theta          = 1
        self.restitution_coefficient = 1
        
        self.absolute_pos   = True
        self.none_focus     = True

        self.path_color     = "darkgrey"
        self.bg_color       = "black"
        self.text_color     = "white"
        self.font           = ("Verdana",15, "normal")
        self.cube_color     = "green"

        self.draw_trail     = False
        self.draw_box       = False
        self.path_size      = .2
        trail_length        = 10000
        trail_resolution    = .5 #0-1
        self.trail_node_number   = int(trail_length * trail_resolution)
        self.trail_node_distance = int(trail_length / self.trail_node_number) #amount of calculations between each node

        self.pointer_size   = 50
        self.onscreen       = []

        #self.image_folder   = "C:/Users/gerri/Desktop/test_vid/imagedata/"
        self.get_vid        = False
        self.max_frame      = 2000
        
        
        self.start_random   = True                 #mass,density, position,velocity,      color data from the web
        self.starting_data  = {'suns':    [('sun 1', 1, 1.41, (0, 0, 0), (0, 0,0), 'yellow')],
                              'planets': [('Earth', 3.003*10**(-6),  5.5, (-1, 0, 0), (0, 0, 1.992007*10**(-7)), 'lightgreen'),
                                          ('Mercury',  1.651*10**(-7),  5.43, (-0.4, 0, 0), (0, 0, 3.1658205*10**(-7)), 'green')]}

        #Values for random creation
        self.size           = 4 #in au, len of cube size
        self.max_velo       = 50000#2 #in 10^-10 AU/s
        self.number_stars   = 100
        self.number_planets = 0 #trail resolution anpassen
        self.planet_colors  = ["beige", "lightgreen", "lightblue"]
        self.sun_colors     = ["yellow","orange","red"]
        #distance range
        #velo range
        #mass star range
        #mass planets range
        #radius if density not implemented

        
        #Benchmarking        
        self.physics_time       = 0
        self.drawing_time       = 0
        self.time_position      = 0
        self.time_interactions  = 0
        self.time_velocity      = 0
        self.time_matrix        = 0
        self.trail_sort         = 0
        self.planet_update      = 0
        

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
        ImageGrab.grab().crop((x,y,x1,y1)).save(f"{self.image_folder}{self.frame_count}.gif")



    def switch_focus(self,event,direction):
        if direction == "left":
            self.solar_system.switch_focus("previous")
        elif direction == "right":
            self.solar_system.switch_focus("next")
        if not self.absolute_pos:
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
            self.x_rotation += (event.x - self.old_x)/10
            self.y_rotation -= (event.y - self.old_y)/10
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
            self.y_offset -= (event.y - self.old_y)*self.distance/1000
            self.old_x = event.x
            self.old_y = event.y

      
    def mouse_scroll(self, event):
        self.distance -= (event.delta)*self.distance/1000
        if self.distance <0:
            self.distance = 0
        



    def mouse_hover(self, event):
        found = False
        if event != None:
            
            x =  event.x-(self.width/2)
            y = -event.y+(self.height/2)-30
            for body in self.onscreen:
                pos_x = body[1][0]
                pos_y = body[1][1] 
                pointer_range =  body[6]/10 * self.pointer_size
                if x > (pos_x - pointer_range) and x < (pos_x + pointer_range) and y > (pos_y - pointer_range) and y < (pos_y + pointer_range):
                    found = True
                    name    = body[0]
                    pos     = body[5]
                    radius  = body[2]
                    mass    = body[3]   
        elif self.solar_system.focused_body != None:
            found = True
            hover_body = self.solar_system.focused_body
            name    = hover_body.name
            pos     = hover_body.position
            radius  = hover_body.radius
            mass    = hover_body.mass

        if found:
            round_pos = []
            for val in pos:
                round_pos.append(round(val))
                    
            text =  f"Name:     {name}\nPosition:  {round_pos}\nRadius:   {radius}\nMass:      {mass}"
            self.mouse.clear()
            self.mouse.goto(self.width*-0.46, self.height*0.35)
            self.mouse.write(text, align="left", font=self.font)
            self.fenster.update()

                    
    def draw_time(self,time):
        self.time_pointer.goto(self.width*-0.46, -self.height*0.42)
        times = ConvertSectoDay(self.solar_system.time)                                                                                                                                                         
        text = f"years: {times[0]}\ndays: {times[1]}\nhours: {times[2]}:{times[3]}:{times[4]}"
        
        print(f"draw = {self.drawing_time: .4f},   rot = {self.time_matrix: .4f},   trail/planet = {self.trail_sort: .4f},   calc = {self.physics_time: .4f},   pos/velo = {self.time_position: .4f},   forces = {self.time_interactions: .4f}")#, impact ={self.time_velocity: .4f}, update ={self.planet_update: .4f}
        
        self.time_pointer.clear()
        self.time_pointer.write(text, align="left", font=self.font)
        #self.fenster.update()
        
        

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
                return (sx, sy), f
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

            
            
        


    def update_vertices(self):
        t1a = time.time()
        data, nodes = self.solar_system.get_data()
        self.draw_time(self.solar_system.time)
        self.rotation_matrix = Rx(math.radians(self.y_rotation-90)) * Ry(math.radians(self.z_rotation)) * Rz(math.radians(-self.x_rotation))
        self.finished = False
        t2a = time.time()
        
        t1b = time.time()
        self.pointer.clear()
        self.onscreen = []
        for body in data:
            body_pos, f = self.get_screen_xy(*body[1])
            
            if self.draw_trail:
                self.draw_last_pos(body[5])
                
            if body_pos != None:
                self.onscreen.append((body[0], body_pos, body[2], body[3], body[4], body[1], f))
        self.onscreen.sort(key=lambda element: element[6])

        if self.draw_box:
            for qube in nodes:
                self.draw_cube(qube)
        


        
        for body in self.onscreen:
            body_pos    = body[1]
            radius      = body[2]
            color       = body[4]
            f           = body[6]
            rad = radius*f
            if rad <1:
                rad =1
            self.pointer.goto(body_pos)
            self.pointer.fd(rad)
            self.pointer.left(90)
            self.pointer.fillcolor(color)
            self.pointer.pencolor(color)
            self.pointer.down()
            self.pointer.begin_fill()
            self.pointer.circle(rad)
            self.pointer.end_fill()
            self.pointer.up()
        t2b = time.time()

        t1c = time.time()
        if self.get_vid:
            self.getter(self.canvas)
        t2c = time.time()

        self.finished = True
        return t2a-t1a, t2b-t1b, t2c-t1c 

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
            position =(random.randint(0,self.size)-(self.size/2),
                       random.randint(0,self.size)-(self.size/2),
                       random.randint(0,self.size)-(self.size/2))
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
            position =(random.randint(0,self.size)-(self.size/2),
                       random.randint(0,self.size)-(self.size/2),
                       random.randint(0,self.size)-(self.size/2))
            velocity = ((random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10),
                        (random.randint(0, self.max_velo)-(self.max_velo/2))*10**(-10))
            color = random.choice(self.planet_colors)
            self.starting_data["planets"] += [(name,mass,density,position,velocity,color)]
            planets.append(Planet(self.solar_system,name,mass,density,position,velocity,color,self.trail_node_number,self.trail_node_distance))

        if self.none_focus:
            self.solar_system.set_focus(None)
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
            if self.none_focus:
                self.solar_system.set_focus(None)
            else:
                self.solar_system.set_focus(suns[0])

        else:
            print("no starting data")

        

        print(self.starting_data)
        self.mouse_hover(None)
        self.solar_system.absolute_pos = self.absolute_pos
    
    

    def update_system(self):
        if not self.pause:
            t1 = time.time()
            self.time_position, self.time_interactions, self.time_velocity = self.solar_system.calculate(self.timestep, self.theta, self.restitution_coefficient)
            t2 = time.time()
            self.physics_time = t2 - t1

        t1 = time.time()
        self.time_matrix, self.trail_sort, self.planet_update = self.update_vertices()
        t2 = time.time()
        self.drawing_time = t2 - t1
        self.frame_count += 1
        if self.frame_count == self.max_frame and self.get_vid:
            self.window.destroy()
        
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

Interface()
