# Barnes hut module
# Gerrit Fritz

import numpy as np
import math


class Node():
    def __init__(self, middle, dimension):
        self.particle   = None
        self.middle     = middle
        self.dimension  = dimension
        self.mass       = None
        self.corners    = None
        self.center_of_mass = None
        self.nodes = []
        self.subnodes = [[[None, None],      # right front (top, bottom)
                          [None, None]],     # right back (top, bottom)
                         [[None, None],      # left front (top, bottom)
                          [None, None]]]     # left back (top, bottom)

    def insert_particle(self, particle):
        self.particle = particle

    def get_subnode(self, quad):
        return (self.subnodes[quad[0]][quad[1]][quad[2]])

    def create_subnode(self, quad):
        dimension  = self.dimension / 2

        x, y, z = 1, 1, 1
        if quad[0] == 1:
            x = -1
        if quad[1] == 1:
            y = -1
        if quad[2] == 1:
            z = -1

        middle = [self.middle[0] + ((dimension / 2) * x),  # quad[0] == 0: value  1, right
                  self.middle[1] + ((dimension / 2) * y),  # quad[1] == 0: value  1, front
                  self.middle[2] + ((dimension / 2) * z)]  # quad[2] == 0: value  1, top
        node       = Node(middle, dimension)
        self.subnodes[quad[0]][quad[1]][quad[2]] = node
        self.nodes.append(node)

    def get_quad(self, point):
        x, y, z = 1, 1, 1
        if point[0] > self.middle[0]:  # right
            x = 0
        if point[1] > self.middle[1]:  # front
            y = 0
        if point[2] > self.middle[2]:  # top
            z = 0
        return [x, y, z]

    def get_corners(self):
        if self.corners is None:
            self.corners = []
            for x in [1, -1]:          # right or left
                for y in [1, -1]:      # front or back
                    for z in [1, -1]:  # top or bottom
                        pos = [self.middle[0] + ((self.dimension / 2) * x),
                               self.middle[1] + ((self.dimension / 2) * y),
                               self.middle[2] + ((self.dimension / 2) * z)]
                        self.corners.append(pos)
        return self.corners

    def in_bounds(self, point):
        val = False
        if point[0] <= self.middle[0] + (self.dimension / 2) and point[0] >= self.middle[0] - (self.dimension / 2) and\
           point[1] <= self.middle[1] + (self.dimension / 2) and point[1] >= self.middle[1] - (self.dimension / 2) and\
           point[2] <= self.middle[2] + (self.dimension / 2) and point[2] >= self.middle[2] - (self.dimension / 2):
            val = True
        return val

    def compute_mass_distribution(self):
        if self.particle is not None:
            self.center_of_mass = np.array([*self.particle.position])
            self.mass = self.particle.mass
        else:
            # Compute the center of mass based on the masses of all child quadrants
            # position based on child quadrants weights with their mass
            self.mass = 0
            self.center_of_mass = np.array([0., 0., 0.])
            for node in self.nodes:
                if node is not None:
                    node.compute_mass_distribution()
                    self.mass += node.mass
                    self.center_of_mass += node.mass * node.center_of_mass
            self.center_of_mass /= self.mass


class Octree():
    def __init__(self, particles, root_node, theta):
        self.theta = theta
        self.root_node = root_node
        self.particles = particles
        for particle in self.particles:
            self.insert_to_node(self.root_node, particle)

    def insert_to_node(self, node, particle):
        # check if point is in cuboid of present node
        if not node.in_bounds(particle.position) and not np.array_equal(particle.position, self.root_node.middle):
            print("error particle not in bounds")
            print(f"middle: {node.middle}, dimension: {node.dimension}, particle position: {particle.position}, type: {type(particle)}")
            return

        # determine the appropriate child node
        quad = node.get_quad(particle.position)
        if node.get_subnode(quad) is None:
            node.create_subnode(quad)
        subnode = node.get_subnode(quad)

        # if subnode is empty, insert point, stop insertion
        if subnode.particle is None and len(subnode.nodes) == 0:  # case empty node
            subnode.insert_particle(particle)

        # If the child node is a point node, replace it with a region node.
        # Call insert for the point that just got replaced.
        # Set current node as the newly formed regionnode.
        elif subnode.particle is not None:  # case point node
            old_particle = subnode.particle
            subnode.insert_particle(None)
            self.insert_to_node(subnode, old_particle)
            self.insert_to_node(subnode, particle)

        # If the child node is a point node, replace it with a region node.
        # Call insert for the point that just got replaced. Set current node as the newly formed region node.
        elif subnode.particle is None and len(subnode.nodes) >= 1:  # case region node
            self.insert_to_node(subnode, particle)

    def update_forces_collisions(self):

        self.collision_dic = {}
        for particle in self.particles:
            self.collision_dic[particle] = []
            particle.force = np.array([0., 0., 0.])
            particle.e_pot = 0
            self.calc_forces(self.root_node, particle)
            particle.e_pot /= 1  # 2

            if len(self.collision_dic[particle]) == 0:
                del self.collision_dic[particle]

    def calc_forces(self, node, particle):
        # Gravitational force and collision between two particles
        if node.particle is not None and node.particle != particle:
            force, e_pot, distance = self.gravitational_force(particle, node, np.array([]), np.array([]))
            particle.force -= force
            particle.e_pot -= e_pot
            if distance < particle.radius + node.particle.radius and particle.mass > node.particle.mass:
                self.collision_dic[particle].append(node.particle)

        # find regional node were particle is not the center of mass (subnodes particle is particle)
        elif node.particle is None and not np.array_equal(particle.position, node.center_of_mass):
            distance = np.array([*particle.position]) - np.array([*node.center_of_mass])
            r = math.sqrt(np.dot(distance, distance))
            d = node.dimension
            if d / r < self.theta:
                force, e_pot, distance = self.gravitational_force(particle, node, distance, r)
                particle.force -= force
                particle.e_pot -= e_pot
            else:
                for subnode in node.nodes:
                    self.calc_forces(subnode, particle)

    def gravitational_force(self, particle, node, distance_vec, distance_val):  # can be ragional or point node
        force = np.array([0., 0., 0.])
        if len(distance_vec) == 0 and len(distance_val) == 0:
            distance = np.array([*particle.position]) - np.array([*node.center_of_mass])
            distance_mag = math.sqrt(np.dot(distance, distance))
        else:
            distance = distance_vec
            distance_mag = distance_val

        G = 6.6743 * 10**(-11)
        e_pot = G * particle.mass * node.mass / distance_mag
        force_mag = G * particle.mass * node.mass / np.dot(distance, distance)
        force = (distance / distance_mag) * force_mag
        return force, e_pot, distance_mag

    def get_all_nodes(self, node, lst):  # all point nodes, could include regional nodes

        if node.particle is None and len(node.nodes) >= 1 or node.particle is not None:
            if len(node.nodes) >= 1:
                # lst.append(node.get_corners())  # if regional are shown aswell
                for subnode in node.nodes:
                    self.get_all_nodes(subnode, lst)
            if node.particle is not None:
                lst.append(node.get_corners())


if __name__ == "__main__":
    from SolarSystem import Planet
    planet1 = Planet
    planet1.position = (10, 20, 30)
    planet1.mass = 200

    planet2 = Planet
    planet2.position = (-10, -20, -30)
    planet2.mass = 20

    data = [planet1, planet2]  # planet2]
    root = Node([0, 0, 0], 100)
    theta = 1

    print(root.in_bounds(planet2.position))
    quad = root.get_quad(planet2.position)
    print(quad)
    root.create_subnode(quad)
    subnode = root.get_subnode(quad)
    print(subnode.middle)
    print(subnode.get_corners())
