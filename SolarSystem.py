# Solarsystem module
# Gerrit Fritz

import math
import numpy as np
import random
from BarnesHut import Octree, Node


class SolarSystem:

    def __init__(self, theta=1, rc=0, absolute_pos=True, focus_index=0):

        self.restitution_coefficient = rc
        self.focus_options  = ["none", "body", "cm"]
        self.absolute_pos   = absolute_pos
        self.theta          = theta
        self.iteration      = 0
        self.bodies         = []
        self.focused_body   = None
        self.first          = True
        self.time           = 0
        self.total_ekin     = 0
        self.total_epot     = 0
        self.total_e        = 0
        self.cm_pos         = np.array([0, 0, 0])
        self.cm_velo        = None

        if focus_index >= 0 and focus_index < len(self.focus_options):
            self.focus_index = focus_index
        else:
            self.focus_index = 0
            print(f"focus index {focus_index} not in focus options {self.focus_options}, swithing to default {self.focus_options[0]}")
        self.focus_type = self.focus_options[focus_index]

    def add_body(self, body):
        self.bodies.append(body)

    def remove_body(self, body):
        self.bodies.remove(body)
        if body == self.focused_body and len(self.bodies) > 0:
            self.focused_body = random.choice(self.bodies)
        elif body == self.focused_body:
            print("error: no bodies left")

    def set_focus(self, body):
        if body in self.bodies:
            self.focused_body = body
        elif self.focus_type == "body":
            self.focused_body = random.choice(self.bodies)
        else:
            self.focused_body = None

    def calculate(self, timestep, draw_box):
        if self.first:
            self.first = False
            self.update_interactions()
            for body in self.bodies:
                body.acceleration = body.force / body.mass

        self.update_center_of_mass(timestep)
        for body in self.bodies:
            body.update_velocity(timestep)
            body.update_position(timestep)

        self.update_interactions()

        self.tree_nodes = []
        if draw_box:
            self.tree.get_all_nodes(self.tree.root_node, self.tree_nodes)

        for body in self.destroyed:
            self.remove_body(body)

        self.total_ekin = 0
        self.total_epot = 0
        self.total_e = 0
        for body in self.bodies:
            self.total_ekin += body.e_kin
            self.total_epot += (body.e_pot / 2)  # no clue if this is right
        self.total_ekin /= len(self.bodies)
        self.total_epot /= len(self.bodies)

        self.total_e = self.total_ekin + self.total_epot  # epot is negative, higher energy the further you are

        self.time += timestep
        self.iteration += 1

    def update_center_of_mass(self, timestep):
        new_pos = np.array(self.tree.root_node.center_of_mass)
        old_pos = self.cm_pos
        self.cm_velo = (new_pos - old_pos) / timestep
        self.cm_pos = new_pos

    def get_bodies(self):
        return self.bodies

    def get_focus_pos(self):
        if self.focus_type == "body":
            pos = self.focused_body.position
        elif self.focus_type == "cm":
            pos = self.cm_pos
        else:
            pos = np.array([0, 0, 0])
        return pos

    def get_data(self):
        default_pos = self.get_focus_pos()

        body_data = []
        for body in self.bodies:
            body_type = type(body)
            name    = body.name
            pos     = body.position - default_pos
            radius  = body.radius
            mass    = body.mass
            color   = body.color
            trail = [position - default_pos for position in body.trail]
            body_data.append((name, pos, radius, mass, color, trail, body.velocity,
                              body.acceleration, body.density, body.force,
                              body.e_kin, body.e_pot, body_type))

        for cube in self.tree_nodes:
            for point in cube:
                point[0] -= default_pos[0]
                point[1] -= default_pos[1]
                point[2] -= default_pos[2]

        system_data = [self.focus_type, self.focused_body, self.absolute_pos,
                       self.theta, self.restitution_coefficient, self.total_ekin,
                       self.total_epot, self.total_e, self.cm_pos, self.iteration,
                       len(self.bodies), self.tree.root_node.middle, self.tree.root_node.dimension]

        return body_data, self.tree_nodes, system_data, self.cm_pos - default_pos

    def switch_focus(self, direction):
        if self.focus_type == "body":
            focused_index = self.bodies.index(self.focused_body)

            if direction == "previous":
                focused_index += 1
                if focused_index > len(self.bodies) - 1:
                    focused_index = 0

            elif direction == "next":
                focused_index -= 1
                if focused_index < 0:
                    focused_index = len(self.bodies) - 1

            self.set_focus(self.bodies[focused_index])

    def clear_trail(self):
        for body in self.bodies:
            body.trail = []

    def update_interactions(self):
        center = self.get_focus_pos()

        largest_val = 0
        furthest_body = None
        for body in self.bodies:
            for i, val in np.ndenumerate(body.position):
                dist_sqr = (val - center[i])**2
                if dist_sqr > largest_val:
                    largest_val = dist_sqr
                    largest_index = i
                    furthest_body = body

        dimension = math.sqrt(((furthest_body.position[largest_index] - center[largest_index]) * 2.5)**2)
        root = Node(center, dimension)
        self.tree = Octree(self.bodies, root, self.theta)
        root.compute_mass_distribution()
        self.tree.update_forces_collisions()
        self.compute_collisions(self.tree.collision_dic)

    def compute_collisions(self, collisions):
        # sort collisions from lowest to highest mass
        # we know that body has bigger mass than other so other is deleted
        # we have to check if other was deleted by a previous collision with another planet
        self.destroyed = []
        bodies = list(collisions.keys())
        bodies.sort(key=lambda element: element.mass)
        for body in bodies:
            other_bodies = collisions[body]
            for other in other_bodies:
                if other not in self.destroyed:
                    body.inelastic_collision(other, self.restitution_coefficient)


class SolarSystemBody:

    def __init__(self, solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist):

        self.solar_system   = solar_system
        self.name           = name
        self.mass           = mass  # mass in kg
        self.density        = density  # density in g/cm^3
        self.radius         = self.calculate_radius()
        self.position       = np.array([*position])
        self.velocity       = np.array([*velocity])
        self.color          = color
        self.nr_pos         = nr_pos
        self.counter        = 0
        self.point_dist     = point_dist
        self.trail          = []
        self.acceleration   = np.array([0, 0, 0])
        self.force          = np.array([0, 0, 0])
        self.e_pot          = 0
        self.e_kin          = 0
        self.solar_system.add_body(self)

    def update_position(self, timestep):
        self.position += (timestep * (self.velocity + timestep * self.acceleration / 2))

        if not self.solar_system.absolute_pos:  # relative movement
            if self.solar_system.focus_type == "body":
                self.position -= (timestep * (self.solar_system.focused_body.velocity + timestep * self.solar_system.focused_body.acceleration / 2))
            elif self.solar_system.focus_type == "cm":
                self.position -= self.solar_system.cm_velo

        self.counter += 1
        if self.counter == self.point_dist:
            self.trail.append(self.position.copy())
            self.counter = 0

        if len(self.trail) > self.nr_pos:
            del self.trail[0]

    def update_velocity(self, timestep):
        newacc = self.force / self.mass
        self.velocity += (self.acceleration + newacc) * timestep / 2
        self.acceleration = newacc
        self.e_kin = .5 * self.mass * np.dot(self.velocity, self.velocity)

    def calculate_radius(self):
        density_kg = self.density * 1000
        return ((3 * self.mass) / (4 * math.pi * density_kg))**(1 / 3)

    def inelastic_collision(self, other, restitution_coefficient):
        # only if restitution_coefficient is 0 bodies merge
        velo_u = ((self.mass * self.velocity) + (other.mass * other.velocity)) / (self.mass + other.mass)

        if restitution_coefficient == 0:
            # merge of planets (other is destroyed)
            self.velocity = velo_u
            self.mass += other.mass
            self.radius = self.calculate_radius()
            self.solar_system.destroyed.append(other)
        else:
            # somewhat elastic collision
            r = restitution_coefficient
            self.velocity = ((self.velocity - velo_u) * r) + velo_u
            other.velocity = ((other.velocity - velo_u) * r) + velo_u


# ---------------------------------------------------------------------------------------------------------------------
class Sun(SolarSystemBody):

    def __init__(self, solar_system, name="default sun", sol_mass=1, density=1.41, position_AU=(0, 0, 0), velocity_km=(0, 0, 0), color="yellow", nr_pos=50, point_dist=10):
        au = 1.496 * 10**11
        km = 10**3
        solM = 1.989 * (10**30)
        mass = sol_mass * solM  # multiple of solar mass
        position = tuple(au * pos for pos in position_AU)
        velocity = tuple(km * velo for velo in velocity_km)
        super(Sun, self).__init__(solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist)


class Planet(SolarSystemBody):

    def __init__(self, solar_system, name="default planet", ea_mass=1, density=5.5, position_AU=(0, 0, 0), velocity_km=(0, 0, 0), color="blue", nr_pos=50, point_dist=10):
        au = 1.496 * 10**11
        km = 10**3
        eam = 5.972 * 10**24
        mass = ea_mass * eam  # multiple of solar mass
        position = tuple(au * pos for pos in position_AU)
        velocity = tuple(km * velo for velo in velocity_km)
        super(Planet, self).__init__(solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist)


class Blackhole(SolarSystemBody):

    def __init__(self, solar_system, name="default hole", sol_mass=10, density=4 * 10**14, position_AU=(0, 0, 0), velocity_km=(0, 0, 0), color="black", nr_pos=50, point_dist=10):
        au = 1.496 * 10**11
        km = 10**3
        solM = 1.989 * (10**30)
        mass = sol_mass * solM  # multiple of solar mass
        position = tuple(au * pos for pos in position_AU)
        velocity = tuple(km * velo for velo in velocity_km)
        super(Planet, self).__init__(solar_system, name, mass, density, position, velocity, color, nr_pos, point_dist)
