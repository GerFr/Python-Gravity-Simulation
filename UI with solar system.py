#tkinter 3d gfx interface

import math
import tkinter
import turtle
from time import sleep
from SolarSystem import SolarSystem, Planet, Sun
import numpy as np


#slider length factor einbauen

class Interface():
    def __init__(self):

        self.fov_range      = (200, 4000, 2000)
        self.y_rot_range    = (-90, 90, 0)
        self.default_dist   = 400
        self.z_rotation     = 0 

        self.mouse_click1   = False
        self.mouse_click2   = False
        self.mouse_click3   = False
        self.pause          = False
        self.finished       = True
        self.draw_trail     = True
        self.absolute_pos   = False

        self.path_color     = "darkgrey"
        self.bg_color       = "black"
        self.text_color     = "white"
        self.font           = ("Verdana",15, "normal")
        self.path_size      = .3

        self.timestep       = 250
        self.timepause      = 20

        self.pointer_size   = 50
        self.onscreen       = []

        

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
        self.window.bind("<Left>",  lambda event: self.switch_focus(event,"left"))
        self.window.bind("<Right>", lambda event: self.switch_focus(event,"right"))
        self.window.bind("<Up>",    lambda event: self.switch_focus(event,"right"))
        self.window.bind("<Down>",  lambda event: self.switch_focus(event,"left"))

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
        if event != None:
            found = False
            x =  event.x-(self.width/2)
            y = -event.y+(self.height/2)-30
            for body in self.onscreen:
                pos_x = body[1][0]
                pos_y = body[1][1] 
                pointer_range =  body[6]/10 * self.pointer_size
                if x > (pos_x - pointer_range) and x < (pos_x + pointer_range) and y > (pos_y - pointer_range) and y < (pos_y + pointer_range):
                    found = True
                    name    = body[0]
                    pos     = body[7]
                    radius  = body[2]
                    mass    = body[3]   
        else:
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
        self.time_pointer.clear()
        self.time_pointer.write(text, align="left", font=self.font)
        self.fenster.update()
        
        

    def reset(self):
        self.FOV        = self.fov_range[2]
        self.distance   = self.default_dist
        self.x_offset   = 0
        self.y_offset   = 0
        self.x_rotation = 0
        self.y_rotation = self.y_rot_range[2]

        self.update_vertices()
        


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


    def update_vertices(self):
        data = self.solar_system.get_data()
        self.draw_time(self.solar_system.time)

        self.rotation_matrix = Rx(math.radians(self.y_rotation-90)) * Ry(math.radians(self.z_rotation)) * Rz(math.radians(-self.x_rotation))

        self.finished = False

        self.pointer.clear()


        self.onscreen = []
        for body in data:
            body_pos, f = self.get_screen_xy(*body[1])
            if body_pos != None:
                self.onscreen.append((body[0], body_pos, body[2], body[3], body[4], body[5], f, body[1]))
        self.onscreen.sort(key=lambda element: element[5])


        for body in self.onscreen:
            body_pos    = body[1]
            radius      = body[2]
            mass        = body[3]
            color       = body[4]
            last_pos    = body[5]
            f           = body[6]

            if self.draw_trail:
                self.draw_last_pos(last_pos)

            self.pointer.goto(body_pos)
            self.pointer.fd(radius*f)
            self.pointer.left(90)
            self.pointer.fillcolor(color)
            self.pointer.pencolor(color)
            
            self.pointer.down()
            self.pointer.begin_fill()
            self.pointer.circle(radius*f)
            self.pointer.end_fill()
            self.pointer.up()

      
        self.fenster.update()
        self.finished = True



    def setup_solar_system(self):
        self.solar_system = SolarSystem()
        sun1 = Sun(self.solar_system,
                    name       = "sun1",
                    mass       = 5864306,          
                    radius     = 10,
                    position   = (0, 0, 0),
                    velocity   = (0, 0, 0),
                    color      = "yellow",
                    nr_pos     = 100)
                
        sun2 = Sun(self.solar_system,
                    name       = "sun2",
                    mass       = 5864306,          
                    radius     = 10,
                    position   = (100, -100, 100),
                    velocity   = (10**(-4), 0, -10**(-2.9)),
                    color      = "yellow",
                    nr_pos     = 100)
                
        planet1 = (Planet(self.solar_system,
                    name       = "planet1",
                    mass       = 20943,
                    radius     = 1,
                    position   = (50, -50, 0), 
                    velocity   = (-10**(-4), -10**(-4), -2*10**(-3)),
                    color      = "lightgreen",
                    nr_pos     = 100)
                )
        self.solar_system.set_focus(planet1)
        self.mouse_hover(None)
        self.solar_system.absolute_pos = self.absolute_pos

    

    def update_system(self):
        if not self.pause:
            self.solar_system.calculate(self.timestep)
        self.update_vertices()
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
