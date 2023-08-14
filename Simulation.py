# Simulation module
# Gerrit Fritz

import math
import numpy as np
import random
from BarnesHut import Octree, Node


class Simulation:
    """Handles data and connection between simulation bodies."""

    def __init__(self, theta=1, rc=0, absolute_pos=True, focus_index=0):
        """Setup of the simulation.

        Method that sets up the simulation with parameters and type of
        focus.

        Args:
            theta: Theta value for the Barnes Hut simulation.
            rc: Restitution coefficient for collisions.
            absolute_pos: Bool value to determine type of movement.
            focus_index: Index of the list focus_options form 0 to 2.
            node_type: String that determines what nodes are displayed.
        """
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

    def get_bodies(self):
        return self.bodies

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

    def update_center_of_mass(self, timestep):
        new_pos = np.array(self.tree.root_node.center_of_mass)
        old_pos = self.cm_pos
        self.cm_velo = (new_pos - old_pos) / timestep
        self.cm_pos = new_pos

    def get_focus_pos(self):
        if self.focus_type == "body":
            pos = self.focused_body.position
        elif self.focus_type == "cm":
            pos = self.cm_pos
        else:
            pos = np.array([0, 0, 0])
        return pos

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

    def calculate(self, timestep, draw_box, node_type):
        """Method that calculates a simulation physics step.

        Method that calls functions for physics calculations. 
        Also includes caclulations for the total energy. If draw_box is
        true the boxes of the octree are also extracted.

        Args:
            timestep: Amount of seconds that are counted with in the
                physics calculations.
            draw_box: Boolean value that determines if cube data should be 
                extracted. Implemented to run faster.
        """
        if self.first:
            self.first = False
            self.update_interactions(node_type)
            for body in self.bodies:
                body.acceleration = body.force / body.mass

        self.update_center_of_mass(timestep)
        for body in self.bodies:
            body.update_velocity(timestep)
            body.update_position(timestep)

        self.update_interactions(node_type)

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

    def get_data(self):
        """Method that gets simulation data.

        Method that exports simulation data in the form of a list.
        The list includes almost all values of the simulation excluding the
        bodies themselves.

        Returns:
            Body data, Octree data and system data for further usage.
        """
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

    def update_interactions(self, node_type):
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
        self.tree = Octree(self.bodies, root, self.theta, node_type)
        root.compute_mass_distribution()
        self.tree.update_forces_collisions()
        self.compute_collisions(self.tree.collision_dic)

    def compute_collisions(self, collisions):
        """Method that computes body collisions.

        Method that computes body collisions based on the resitution
        coefficient. Sort collisions from lowest to highest mass. So all
        collisions can be determined. Calls body to body inelastic collision.

        Args:
            collisions: All collisons as extractes from the Barnes-Hut program.
        """
        self.destroyed = []
        bodies = list(collisions.keys())
        bodies.sort(key=lambda element: element.mass)
        for body in bodies:
            other_bodies = collisions[body]
            for other in other_bodies:
                if other not in self.destroyed:
                    body.inelastic_collision(other, self.restitution_coefficient)


class SimulationBody:
    """Data storage and caclulation of a simulation body"""

    def __init__(self, simulation, name, mass, density, position, velocity, color, nr_pos, point_dist):
        """Method that sets up a simulation body.

        Method sets up and declares variables for a simulation body.

        Args:
            simulation: Simulation that stores all other bodies.
            name: Name/Unique identifier of body.
            mass: Mass of the body in kg.
            density: Density of the body in g/cm3.
            position: Position vector in meters.
            velocity: Velocity vector in m/s.
            color: Color of the body.
            nr_pos: Trail node number of the body.
            point_dist: Distance between nodes in the trail.
        """
        self.simulation     = simulation
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
        self.simulation.add_body(self)

    def update_position(self, timestep):
        """Method that calculates the body position.

        Method that calculates the body position follwing verlet velocity
        integration. Subtracts movement of focus_body if the view is movement
        is relative.

        Args:
            timestep: Amount of seconds for the calculations.
        """
        self.position += (timestep * (self.velocity + timestep * self.acceleration / 2))

        if not self.simulation.absolute_pos:  # relative movement
            if self.simulation.focus_type == "body":
                self.position -= (timestep * (self.simulation.focused_body.velocity + timestep * self.simulation.focused_body.acceleration / 2))
            elif self.simulation.focus_type == "cm":
                self.position -= self.simulation.cm_velo

        self.counter += 1
        if self.counter == self.point_dist:
            self.trail.append(self.position.copy())
            self.counter = 0

        if len(self.trail) > self.nr_pos:
            del self.trail[0]

    def update_velocity(self, timestep):
        """Method that calculates body velocity.

        Method that calculates body velocity following velocity
        verlet ingtegration.

        Args:
            timestep: Amount of seconds for the calculations.
        """
        newacc = self.force / self.mass
        self.velocity += (self.acceleration + newacc) * timestep / 2
        self.acceleration = newacc
        self.e_kin = .5 * self.mass * np.dot(self.velocity, self.velocity)

    def calculate_radius(self):
        density_kg = self.density * 1000
        return ((3 * self.mass) / (4 * math.pi * density_kg))**(1 / 3)

    def inelastic_collision(self, other, restitution_coefficient):
        """Method that calculates an inelastic collision.

        Method that calculates the collision between two objects based
        on the restitution coefficient. If that coefficient if 0 the planets
        merge. If the coefficient is higher than 0 to 1 the collision is more
        elastic. Destroys body that is smaller than the other.

        Args:
            other: Other body that takes part in the collision.
            restitution_coefficient: Coefficient that determines type of
                collision.
        """
        # only if restitution_coefficient is 0 bodies merge
        velo_u = ((self.mass * self.velocity) + (other.mass * other.velocity)) / (self.mass + other.mass)

        if restitution_coefficient == 0:
            # merge of planets (other is destroyed)
            self.velocity = velo_u
            self.mass += other.mass
            self.radius = self.calculate_radius()
            self.simulation.destroyed.append(other)
        else:
            # somewhat elastic collision
            r = restitution_coefficient
            self.velocity = ((self.velocity - velo_u) * r) + velo_u
            other.velocity = ((other.velocity - velo_u) * r) + velo_u


# ---------------------------------------------------------------------------------------------------------------------
class Sun(SimulationBody):

    def __init__(self, simulation, name="default sun", sol_mass=1, density=1.41, position_AU=(0, 0, 0), velocity_km=(0, 0, 0), color="yellow", nr_pos=50, point_dist=10):
        au = 1.496 * 10**11
        km = 10**3
        solM = 1.989 * (10**30)
        mass = sol_mass * solM  # multiple of solar mass
        position = tuple(au * pos for pos in position_AU)
        velocity = tuple(km * velo for velo in velocity_km)
        super(Sun, self).__init__(simulation, name, mass, density, position, velocity, color, nr_pos, point_dist)


class Planet(SimulationBody):

    def __init__(self, simulation, name="default planet", ea_mass=1, density=5.5, position_AU=(0, 0, 0), velocity_km=(0, 0, 0), color="blue", nr_pos=50, point_dist=10):
        au = 1.496 * 10**11
        km = 10**3
        eam = 5.972 * 10**24
        mass = ea_mass * eam  # multiple of solar mass
        position = tuple(au * pos for pos in position_AU)
        velocity = tuple(km * velo for velo in velocity_km)
        super(Planet, self).__init__(simulation, name, mass, density, position, velocity, color, nr_pos, point_dist)


class Blackhole(SimulationBody):

    def __init__(self, simulation, name="default hole", sol_mass=10, density=4 * 10**14, position_AU=(0, 0, 0), velocity_km=(0, 0, 0), color="black", nr_pos=50, point_dist=10):
        au = 1.496 * 10**11
        km = 10**3
        solM = 1.989 * (10**30)
        mass = sol_mass * solM  # multiple of solar mass
        position = tuple(au * pos for pos in position_AU)
        velocity = tuple(km * velo for velo in velocity_km)
        super(Planet, self).__init__(simulation, name, mass, density, position, velocity, color, nr_pos, point_dist)
