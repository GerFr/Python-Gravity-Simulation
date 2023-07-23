#SolarSystem.py
#Gerrit Fritz
import math
import tinyarray as ta#from vectors import Vector3D
import time
import random
from BarnesHut import Octree, Node



#https://gamedev.stackexchange.com/questions/15708/how-can-i-implement-gravity
#They are second order integrators, meaning that, even with varying acceleration, the average integration error is only proportional to the square of the timestep. This can allow for larger timesteps without compromising accuracy.



class SolarSystem:
    def __init__(self):
        self.bodies = []
        self.focused_body = "none"
        self.absolute_pos = False
        self.first = True
        self.time = 0
        self.total_ekin = 0
        self.total_epot = 0
        self.total_e = 0
        self.cm_pos = ta.array([0,0,0])
        self.cm_velo = None
        
    def add_body(self, body):
        self.bodies.append(body)

    def remove_body(self, body):
        self.bodies.remove(body)
        if body == self.focused_body and len(self.bodies)>0:
            self.focused_body = random.choice(self.bodies)
        elif body == self.focused_body:
            print("error: no bodies left")
        

    def set_focus (self, body):
        if body in self.bodies:
            self.focused_body = body
        else:
            self.focused_body = body

            
    def calculate(self, timestep, theta, restitution_coefficient, draw_box):
        if self.first:
            self.first = False
            self.update_interactions(theta, restitution_coefficient)
            for body in self.bodies:
                body.acceleration = body.force/body.mass
                
        t1a = time.time()
        self.update_center_of_mass(timestep)
        for body in self.bodies:
            body.update_velocity(timestep)
            body.update_position(timestep)
        
        t2a = time.time()
        

        t1b = time.time()
        self.update_interactions(theta, restitution_coefficient)
        t2b = time.time()
        
        t1c = time.time()
        self.tree_nodes = []
        if draw_box:
            self.tree.get_all_nodes(self.tree.root_node,self.tree_nodes)
        t2c = time.time()

        for body in self.destroyed:
            self.remove_body(body)


        self.total_ekin = 0
        self.total_epot = 0
        self.total_e = 0
        for body in self.bodies:
            self.total_ekin += body.e_kin
            self.total_epot += (body.e_pot/2)
        self.total_ekin /= len(self.bodies)
        self.total_epot /= len(self.bodies)
        
        self.total_e = self.total_ekin + self.total_epot #epot is negative, higher energy the further you are

        self.time += timestep

        return t2a - t1a, t2b - t1b, t2c - t1c


    def update_center_of_mass(self, timestep):
        new_pos = ta.array(self.tree.root_node.center_of_mass)
        old_pos = self.cm_pos
        self.cm_velo = (new_pos-old_pos)/timestep
        self.cm_pos = new_pos
            

    def get_bodies(self):
        return self.bodies

    
    def get_data(self):
        all_data =[]
        if self.focused_body == "none":
            default_pos = ta.array([0, 0, 0])
        elif self.focused_body == "cm":
            default_pos = self.cm_pos
        else:
            default_pos = self.focused_body.position

            
        for body in self.bodies:
            name    = body.name
            pos     = body.position - default_pos #array math
            print(pos)
            radius  = body.radius
            mass    = body.mass
            color   = body.color
            trail= [position - default_pos for position in body.trail]
            all_data.append((name,pos,radius,mass,color,trail, body.velocity, body.acceleration, body.density, body.force, body.e_kin, body.e_pot))


        for cube in self.tree_nodes:
            for point in cube:
                point[0] -= default_pos[0]
                point[1] -= default_pos[1]
                point[2] -= default_pos[2]

        self.cm_pos -= default_pos
     
        return all_data, self.tree_nodes, self.cm_pos




    def switch_focus(self,direction):
        if self.focused_body not in ["none","cm"]:
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
            body.trail= []



    def update_interactions(self, theta, restitution_coefficient):
        if self.absolute_pos or self.focused_body == "none":
            default_pos = ta.array([0,0,0])
        elif self.focused_body == "cm":
            default_pos = self.cm_pos
        else:
            default_pos = self.focused_body.position

        
        

        largest_val = 0
        furthest_bod = None
        for bodie in self.bodies:
            for i, val in enumerate(bodie.position):
                dist_sqr = (val-default_pos[i])**2 
                if dist_sqr > largest_val:
                    largest_val = dist_sqr
                    largest_index = i
                    furthest_bod = bodie
            
        dimension = math.sqrt(((furthest_bod.position[largest_index]-default_pos[largest_index])*2.5)**2)#
        root = Node(default_pos, dimension)#[0,0,0]
        self.tree = Octree(self.bodies, root, theta)
        root.compute_mass_distribution()
        self.tree.update_forces_collisions()
        self.compute_collisions(self.tree.collision_dic, restitution_coefficient)
        
        

    def compute_collisions(self, collisions, restitution_coefficient):
        #sort collisions from lowest to highest mass
        self.destroyed = []
        bodies = list(collisions.keys())
        bodies.sort(key=lambda element: element.mass)
        for body in bodies:
            other_bodies = collisions[body]
            for other in other_bodies:
                if other not in self.destroyed:
                    body.inelastic_collision(other, restitution_coefficient)#we know that body has bigger mass than other so other is deleted we have to check if other was deleted by a previous collision with another planet
                    
                
            



                


class SolarSystemBody:
    
    def __init__(self, solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist):
        
        self.solar_system   = solar_system
        self.name           = name 
        self.mass           = mass #mass in kg
        self.density        = density #density in g/cm^3
        self.radius         = self.calculate_radius()
        self.position       = ta.array([*position])
        self.velocity       = ta.array([*velocity]) 
        self.color          = color
        self.nr_pos         = nr_pos
        self.counter        = 0
        self.point_dist     = point_dist
        self.trail          = []
        self.other_relative_trails = {}
        self.acceleration   = ta.array([0,0,0])
        self.force          = ta.array([0,0,0])
        self.e_pot          = 0
        self.e_kin          = 0
        self.solar_system.add_body(self)

        
    def update_position(self, timestep):
        self.position += (timestep* (self.velocity+ timestep*self.acceleration/2))

        if not self.solar_system.absolute_pos:
            if self.solar_system.focused_body not in ["none", "cm"]:
                self.position -= (timestep* (self.solar_system.focused_body.velocity+ timestep*self.solar_system.focused_body.acceleration/2))
            elif self.solar_system.focused_body == "cm":
                self.position -= self.solar_system.cm_velo

                
        self.counter += 1
        if self.counter == self.point_dist:
            self.trail.append(self.position)
            self.counter = 0
        
        if len(self.trail)> self.nr_pos:
            del self.trail[0]
             
        
    def update_velocity(self, timestep):
        newacc = self.force/self.mass
        self.velocity += (self.acceleration + newacc) * timestep / 2
        self.acceleration = newacc
        self.e_kin = .5*self.mass*ta.dot(self.velocity,self.velocity)


    def calculate_radius(self):
        density_kg = self.density *1000
        return ((3*self.mass)/(4*math.pi*density_kg))**(1/3)
        

        
#https://www.plasmaphysics.org.uk/collision3d.htm
    def inelastic_collision(self, other, restitution_coefficient):#perfectly inelastic collisions, could add coefficient of restitution later
        #only if restitution_coefficient == 0: bodies merge

        velo_u = ((self.mass*self.velocity) + (other.mass*other.velocity)) / (self.mass + other.mass)#tinyarray is created, vector math checks out if division is done skalarwise

        if restitution_coefficient == 0:
            #merge of planets (other is destroyed)
            self.velocity = velo_u
            self.mass += other.mass
            self.radius = self.calculate_radius()
            self.solar_system.destroyed.append(other)
            #print("inelastic collision")
            
        else:
            #somewhat elastic collision
            r = restitution_coefficient
            self.velocity = ((self.velocity-velo_u)*r)+velo_u
            other.velocity = ((other.velocity-velo_u)*r)+velo_u
            
            
        
        
 

#---------------------------------------------------------------------------------------------------------------------
class Sun(SolarSystemBody):
    
    def __init__(self, solar_system,name = "default sun", sol_mass = 2, density =1.41, position_AU = (0, 0, 0), velocity_AU = (0, 0, 0), color = "yellow", nr_pos = 50, point_dist = 10):
        au = 1.496*10**11
        solM = 1.989  *(10**30)
        mass = sol_mass * solM #multiple of solar mass
        position = tuple(au*pos for pos in position_AU)
        velocity = tuple(au*velo for velo in velocity_AU)
        super(Sun, self).__init__(solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist)


class Planet(SolarSystemBody):
    def __init__(self, solar_system, name = "default planet", sol_mass = 3*10**(-6), density=5.5, position_AU = (0, 0, 0), velocity_AU = (0, 0, 0),color = "blue", nr_pos = 50, point_dist = 10):
        au = 1.496*10**11
        solM = 1.989  *(10**30)
        mass = sol_mass * solM #multiple of solar mass
        position = tuple(au*pos for pos in position_AU)
        velocity = tuple(au*velo for velo in velocity_AU)
        super(Planet, self).__init__(solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist)


