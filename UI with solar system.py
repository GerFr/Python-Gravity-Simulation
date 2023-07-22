#tkinter 3d gfx interface

import math
import tkinter
import turtle
from time import sleep
from SolarSystem import SolarSystem, Planet, Sun


#slider length factor einbauen

class Interface():
    def __init__(self):

        self.fov_range      = (200, 4000, 2000)
        self.x_rot_range    = (-360, 360, 0)
        self.y_rot_range    = (-360, 360, 0)
        self.default_dist   = 400
        self.z_rotation = 0 #const

        self.mouse_click1   = False
        self.mouse_click2   = False
        self.mouse_click3   = False
        self.pause          = False
        self.finished       = True
        self.draw_trail     = True
        self.absolute_pos   = False

        self.path_color = "darkgrey"
        self.path_size  = .3

        self.timestep = 250
        self.timepause = 5

        self.pointer_size = 50
        

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
        self.fenster.bgcolor("black")
        self.pointer = turtle.RawTurtle(self.fenster)
        self.pointer.ht()
        self.pointer.up()

        self.mouse = turtle.RawTurtle(self.fenster)
        self.mouse.up()
        self.mouse.ht()
        self.mouse.color("white")

        self.canvas.bind("<MouseWheel>", self.mouse_scroll)
        self.canvas.bind("<Motion>", self.get_object)
        self.canvas.bind("<B1-Motion>", self.offset)
        self.canvas.bind("<B2-Motion>", self.change_fov)
        self.canvas.bind("<B3-Motion>", self.rotation)
        self.canvas.bind("<ButtonRelease-1>", lambda event: self.mouse_off(event, "b1"))
        self.canvas.bind("<ButtonRelease-2>", lambda event: self.mouse_off(event, "b2"))
        self.canvas.bind("<ButtonRelease-3>", lambda event: self.mouse_off(event, "b3"))
        
    
        
    def toggle_pause(self):
        if self.pause == False:
            self.pause = True
            self.pause_button.config(text = "start")
        else:
            self.pause = False
            self.pause_button.config(text = "pause")
            self.window.id = self.window.after(self.timepause, self.update_system)
            
            

        
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
            self.update_vertexes()
            


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

            if self.x_rotation < self.x_rot_range[0]:
                self.x_rotation = self.x_rot_range[0]
            elif self.x_rotation > self.x_rot_range[1]:
                self.x_rotation = self.x_rot_range[1]
                
            if self.y_rotation < self.y_rot_range[0]:
                self.y_rotation = self.y_rot_range[0]
            elif self.y_rotation > self.y_rot_range[1]:
                self.y_rotation = self.y_rot_range[1]

            self.update_vertexes()
        



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

            self.update_vertexes()
        

      
    def mouse_scroll(self, event):
        self.distance -= (event.delta)*self.distance/1000
        if self.distance <0:
            self.distance = 0
        self.update_vertexes()



    def get_object(self, event):
        x =  event.x-(self.window.maxsize()[0]/2)
        y = -event.y+(self.window.maxsize()[1]/2)-30
        

        #y -= self.y_offset

        for body in self.onscreen:
            pos_x = body[1][0]
            pos_y = body[1][1] 
            pointer_range =  body[5]/10 * self.pointer_size
            if x > (pos_x - pointer_range) and x < (pos_x + pointer_range) and y > (pos_y - pointer_range) and y < (pos_y + pointer_range):
                self.mouse.clear()
                self.mouse.goto(self.window.maxsize()[0]*-0.46, self.window.maxsize()[1]*0.42)
                self.mouse.write(body[0], font=("Verdana",15, "normal"))
                self.fenster.update()
                
            

    def reset(self):
        self.FOV        = self.fov_range[2]
        self.distance   = self.default_dist
        self.x_offset   = 0
        self.y_offset   = 0
        self.x_rotation = self.x_rot_range[2]
        self.y_rotation = self.y_rot_range[2]

        self.update_vertexes()
        

    


    def rotate(self, x, y, r):
      s, c = math.sin(math.radians(r)), math.cos(math.radians(r))
      return x * c - y * s, x * s + y * c


    def get_screen_xy(self,x,y,z):
            y, z = self.rotate(y, z, self.y_rotation-90)
            x, z = self.rotate(x, z, self.x_rotation)
            x, y = self.rotate(x, y, self.z_rotation)
            
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


    def update_vertexes(self):
        data = self.solar_system.get_data()
        
        self.finished = False
        self.pointer.clear()

        self.onscreen = []
        for body in data:
            body_pos, f = self.get_screen_xy(*body[1])
            if body_pos != None:
                self.onscreen.append((body[0], body_pos, body[2], body[3], body[4], f))
        self.onscreen.sort(key=lambda element: element[5])


        for body in self.onscreen:
            body_pos    = body[1]
            radius      = body[2]
            color       = body[3]
            last_pos    = body[4]
            f           = body[5]

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
                    velocity   = (0, 0, 0),#(-10**(-2.9), 0, 10**(-4))
                    color      = "yellow",
                    nr_pos     = 100)
                
##        sun2 = Sun(self.solar_system,
##                    name       = "sun2",
##                    mass       = 5864306,          
##                    radius     = 10,
##                    position   = (100, 100, 100),
##                    velocity   = (10**(-4), 0, -10**(-2.9)),
##                    color      = "yellow",
##                    nr_pos     = 1000)
                
        planet1 = (Planet(self.solar_system,
                    name       = "planet1",
                    mass       = 20943,
                    radius     = 1,
                    position   = (50, 50, 0), #100 -60 -100
                    velocity   = (10**(-4), 10**(-4), 2*10**(-3)),#(-4*10**(-4), -3*10**(-4), -2*10**(-4))
                    color      = "lightgreen",
                    nr_pos     = 100)
                )
        self.solar_system.set_focus(sun1)
        self.solar_system.absolute_pos = self.absolute_pos

    

    def update_system(self):
        if not self.pause:
            self.solar_system.calculate(self.timestep)
            self.update_vertexes()
            self.window.id = self.window.after(self.timepause, self.update_system)




Interface()
