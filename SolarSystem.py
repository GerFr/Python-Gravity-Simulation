#SolarSystem.py
#Gerrit Fritz
import math
import tinyarray as np#from vectors import Vector3D
import time




#https://gamedev.stackexchange.com/questions/15708/how-can-i-implement-gravity
#They are second order integrators, meaning that, even with varying acceleration, the average integration error is only proportional to the square of the timestep. This can allow for larger timesteps without compromising accuracy.



class SolarSystem:
    def __init__(self):
        self.bodies = []
        self.focused_body = None
        self.absolute_pos = False
        self.first = True
        self.time = 0
        
    def add_body(self, body):
        self.bodies.append(body)

    def remove_body(self, body):
        self.bodies.remove(body)
        if body == self.focused_body:
            self.focused_body = None


    def set_focus (self, body):
        if body in self.bodies:
            self.focused_body = body
            
    def calculate(self, timestep):
        if self.first:
            self.first = False
            self.update_interactions()
            for body in self.bodies:
                body.acceleration = body.force/body.mass
                
        t1a = time.time()
        if self.focused_body != None: 
            self.focused_body.update_position(timestep)
            for body in self.bodies:
                if body != self.focused_body:
                    body.update_position(timestep)
        else:
            for body in self.bodies:
                body.update_position(timestep)
        t2a = time.time()

        t1b = time.time()
        self.update_interactions()
        for body in self.destroyed:
            self.remove_body(body)
        t2b = time.time()

        t1c = time.time()
        for body in self.bodies:
            body.update_velocity(timestep)
        t2c =  time.time()

        self.time += timestep

        return t2a - t1a, t2b - t1b, t2c - t1c
            

    def get_bodies(self):
        return self.bodies

    
    def get_data(self):
        all_data =[]
        if self.focused_body == None:
            default_pos = (0, 0, 0)
        else:
            default_pos = self.focused_body.position
        for body in self.bodies:
            name    = body.name
            pos     = (body.position[0] - default_pos[0],
                       body.position[1] - default_pos[1],
                       body.position[2] - default_pos[2])
            radius  = body.radius
            mass    = body.mass
            color   = body.color
            last_pos= []
            for position in body.last_pos:
                last_pos.append((position[0] - default_pos[0],
                                 position[1] - default_pos[1],
                                 position[2] - default_pos[2]))
            all_data.append((name,pos,radius,mass,color,last_pos))
        return all_data


    def switch_focus(self,direction):
        if self.focused_body != None:
            focused_index = self.bodies.index(self.focused_body)

            if direction == "previous":
                focused_index += 1
                if focused_index > len(self.bodies)-1:
                    focused_index = 0

            elif direction == "next":
                focused_index -= 1
                if focused_index < 0:
                    focused_index = len(self.bodies)-1

            self.set_focus(self.bodies[focused_index])

    def clear_trail(self):
        for body in self.bodies:
            body.last_pos= []


    def update_interactions(self):
        for body in self.bodies:
            body.force = np.array([0.,0.,0.])

        self.destroyed = []
        for i, body in enumerate(self.bodies):
            for other in self.bodies[i+1:]:
                force, distance_mag = body.gravitational_force(other)
                body.force += force
                other.force -= force
                self.destroyed += body.check_collision(other, distance_mag)
                
            


                


class SolarSystemBody:
    
    def __init__(self, solar_system, name, mass, radius, position=(0, 0, 0), velocity=(0, 0, 0), color = "black", nr_pos = 50, point_dist = 10):
        
        self.solar_system   = solar_system
        self.name           = name 
        self.mass           = mass
        self.position       = position 
        self.velocity       = np.array([*velocity]) 
        self.color          = color
        self.radius         = radius
        self.nr_pos         = nr_pos
        self.counter        = 0
        self.point_dist     = point_dist
        self.last_pos       = []

        self.acceleration   = None
        self.force          = None

        
        self.solar_system.add_body(self)

        
    def update_position(self, timestep):
        self.position = (self.position[0] + (timestep* (self.velocity[0]+ timestep*self.acceleration[0]/2)),
                         self.position[1] + (timestep* (self.velocity[1]+ timestep*self.acceleration[1]/2)),
                         self.position[2] + (timestep* (self.velocity[2]+ timestep*self.acceleration[2]/2)))

        if not self.solar_system.absolute_pos and self.solar_system.focused_body != None:
            self.position = (self.position[0] - (timestep* (self.solar_system.focused_body.velocity[0]+ timestep*self.solar_system.focused_body.acceleration[0]/2)),
                             self.position[1] - (timestep* (self.solar_system.focused_body.velocity[1]+ timestep*self.solar_system.focused_body.acceleration[1]/2)),
                             self.position[2] - (timestep* (self.solar_system.focused_body.velocity[2]+ timestep*self.solar_system.focused_body.acceleration[2]/2)))
            
        self.counter += 1
        if self.counter == self.point_dist:
            self.last_pos.append(self.position)
            self.counter = 0
        
        if len(self.last_pos)> self.nr_pos:
            del self.last_pos[0]
         
        
    def update_velocity(self, timestep):
        newacc = self.force/self.mass
        self.velocity += (self.acceleration + newacc) * timestep / 2
        self.acceleration = newacc
        
             
    def gravitational_force(self, body):
        force = np.array([0.,0.,0.])
        distance = np.array([*body.position]) - np.array([*self.position])
        distance_mag = math.sqrt(distance[0]**2 + distance[1]**2 + distance[2]**2)#distance.get_magnitude()
        force_mag = 6.6743 * 10**(-11) * self.mass * body.mass / (distance_mag ** 2)
        force = (distance/distance_mag * force_mag)
        return force, distance_mag
        

    def check_collision(self, other, distance):
##        if isinstance(self, Planet) and isinstance(other, Planet):
##            return#merge? lets do it!

        removed = []
        if distance < self.radius + other.radius:
            for body in self, other:
                if isinstance(body, Planet):
                    removed.append(body)
        return removed
                    
    

#---------------------------------------------------------------------------------------------------------------------
class Sun(SolarSystemBody):
    factor  = 2.
    solM    = 1.989  *(10**30)
    solR    = 696*(10**6)
    
    def __init__(self, solar_system,
                 name       = "default sun",
                 mass       = factor*solM,          #(avg solar mass  = 2x solar mass)
                 radius     = (factor**0.8)*solR,   #R and M are both in solar units (Rsun and Msun), then R = M ** 0.8
                 position   = (0, 0, 0),
                 velocity   = (0, 0, 0),
                 color      = "yellow",
                 nr_pos     = 50,
                 point_dist = 10):
        
        super(Sun, self).__init__(solar_system, name, mass, radius, position, velocity, color, nr_pos, point_dist)


class Planet(SolarSystemBody):
    def __init__(self, solar_system,
                 name       = "default planet",
                 mass       = 3.0025*(10**24),
                 radius     = 8*10**6,
                 position   = (0, 0, 0),
                 velocity   = (0, 0, 0),
                 color      = "blue",
                 nr_pos     = 50,
                 point_dist = 10):
        
        super(Planet, self).__init__(solar_system, name, mass, radius, position, velocity, color, nr_pos, point_dist)

