#tree has to be recreated for every step of the sim maybe sometimes it can be skipped
#seems very expensive for python
#calculate the tree every 5 steps or with multithreading, parrallel the previous tree is analyzed in the main thread and the new data is made to a new tree
import tinyarray as ta
import math

class Node():
    def __init__(self, middle, dimension):
        self.particle   = None
        self.middle     = middle            #middle point
        self.dimension  = dimension         
        self.mass       = None
        self.corners    = None
        self.center_of_mass = None          #whats the starting value? maybe self. middle
        self.nodes = []
        self.subnodes = [[[None,None],      #rechts vorne (oben, unten)
                          [None,None]],     #rechts hinten (oben, unten)
                         [[None,None],      #links vorne (oben, unten)
                          [None,None]]]     #links hinten (oben, unten)

    def insert_particle(self, particle):
        self.particle = particle
        
    def get_subnode(self, quad):
        return(self.subnodes[quad[0]][quad[1]][quad[2]])

    def create_subnode(self, quad):#case 1,1,1
        dimension  = self.dimension/2
        
        x,y,z = 1,1,1
        if quad[0] == 1:
            x = -1
        if quad[1] == 1:
            y = -1
        if quad[2] == 1:
            z = -1
            
        middle = [self.middle[0]+((dimension/2)*x), #quad[0] == 0: wert  1, rechts, positiver bereich
                  self.middle[1]+((dimension/2)*y), #quad[1] == 0: wert  1, vorne, nagativer bereich
                  self.middle[2]+((dimension/2)*z)] #quad[2] == 0: wert  1, oben, positiver bereich
        node       = Node(middle, dimension)
        self.subnodes[quad[0]][quad[1]][quad[2]] = node
        self.nodes.append(node)
                          
    def get_quad(self, point):
        x,y,z = 1,1,1
        if point[0] > self.middle[0]:#rechts 
            x = 0
        if point[1] > self.middle[1]: #vorne #anscheinend muss das so sein trial and error eigl hätte ich andersrum bei y gemacht
            y = 0
        if point[2] > self.middle[2]: #oben
            z = 0
        return [x,y,z]
    
    def get_corners(self):
        if self.corners == None:
            self.corners = []
            for x in [1,-1]:        #rechts, links
                for y in [1,-1]:    #vorne,  hinten
                    for z in [1,-1]:#oben,   unten
                        pos = [self.middle[0]+((self.dimension/2)*x),
                               self.middle[1]+((self.dimension/2)*y),
                               self.middle[2]+((self.dimension/2)*z)]
                        self.corners.append(pos)
        return self.corners
    
    def in_bounds(self,point):
        val = False
        if point[0] <= self.middle[0]+(self.dimension/2) and point[0] >= self.middle[0]-(self.dimension/2) and\
           point[1] <= self.middle[1]+(self.dimension/2) and point[1] >= self.middle[1]-(self.dimension/2) and\
           point[2] <= self.middle[2]+(self.dimension/2) and point[2] >= self.middle[2]-(self.dimension/2):
            val = True
        return val
    
    def compute_mass_distribution(self):
        if self.particle != None:
            self.center_of_mass = ta.array([*self.particle.position])
            self.mass = self.particle.mass
        else:
            #Compute the center of mass based on the masses of all child quadrants and the center of mass as the center of mass of the child quadrants weightes with their mass
            self.mass = 0
            self.center_of_mass = ta.array([0,0,0])
            for node in self.nodes:
                if node != None:
                    node.compute_mass_distribution()
                    self.mass += node.mass
                    self.center_of_mass += node.mass*node.center_of_mass#is this correct? 
            self.center_of_mass /= self.mass
            
      
        



#https://iq.opengenus.org/octree/ #für funktionsweise des Baumes
class Octree():
    def __init__(self, particles, root_node, theta):
        self.theta = theta
        self.root_node = root_node
        self.particles = particles
        for particle in self.particles:
            self.insert_to_node(self.root_node, particle)

    def insert_to_node(self, node, particle):
        #check if point is in cuboid of present node
        if not node.in_bounds(particle.position):
            #print("error particle not in bounds")
            #print(f"middle: {node.middle}, dimension: {node.dimension}, particle position: {particle.position}, type: {type(particle)}")
            return

        #determine the appropriate child node
        quad = node.get_quad(particle.position)
        if node.get_subnode(quad) == None:
            node.create_subnode(quad)
        subnode = node.get_subnode(quad)

        #if subnode is empty, insert point, stop insertion
        if subnode.particle == None and len(subnode.nodes) == 0: #case empty node
            subnode.insert_particle(particle)
            
        #If the child node is a point node, replace it with a region node. Call insert for the point that just got replaced. Set current node as the newly formed regionnode.
        elif subnode.particle != None: #case point node
            old_particle = subnode.particle
            subnode.insert_particle(None)
            self.insert_to_node(subnode, old_particle)
            self.insert_to_node(subnode, particle)

        #If the child node is a point node, replace it with a region node. Call insert for the point that just got replaced. Set current node as the newly formed region node.
        elif subnode.particle == None and len(subnode.nodes) >= 1: #case region node
            self.insert_to_node(subnode, particle)

            
#----------------force calculations----------------
            
    def setup_forces(self):
        for particle in self.particles:
            particle.force = ta.array([0.,0.,0.])
            self.calc_forces(self.root_node, particle)

    def calc_forces(self, node, particle):
        if node.particle != None and node.particle != particle:
            particle.force -= self.gravitational_force(particle,node,ta.array([]),ta.array([]))
        elif node.particle == None and particle.position != node.center_of_mass:
            distance = ta.array([*particle.position]) - ta.array([*node.center_of_mass])
            r = math.sqrt(ta.dot(distance,distance))#magnitude(distance)#distance.get_magnitude(), #distance(node.center of mass, particle)
            d = node.dimension
            #print(particle.position, node.center_of_mass, type(particle))
            if d/r < self.theta:
                particle.force -= self.gravitational_force(particle,node, distance, r)
            else:
                for subnode in node.nodes:
                    self.calc_forces(subnode, particle)
        
                    
    def gravitational_force(self, particle, node, distance_vec, distance_val):#can be ragional or point node
        force = ta.array([0.,0.,0.])
        if len(distance_vec) == 0  and  len(distance_val) == 0:
            distance = ta.array([*particle.position]) - ta.array([*node.center_of_mass])
            distance_mag = math.sqrt(ta.dot(distance,distance))#magnitude(distance)#distance.get_magnitude()
        else:
            distance = distance_vec
            distance_mag = distance_val
        force_mag = 6.6743 * 10**(-11) * particle.mass * node.mass / ta.dot(distance,distance)
        force = (distance/distance_mag * force_mag)
        return force
        
    

    def get_all_nodes(self, node, lst):#all point nodes, could include regional nodes
        
        if node.particle == None and len(node.nodes) >= 1 or node.particle != None:
            if len(node.nodes) >= 1:
                #lst.append(node.get_corners()) #if regional are shown aswell
                for subnode in node.nodes:
                    self.get_all_nodes(subnode, lst)
            if node.particle != None: 
                lst.append(node.get_corners())

if __name__ == "__main__":
    from SolarSystem import Planet
    planet1 = Planet
    planet1.position = (10,20,30)
    planet1.mass = 200

    planet2 = Planet
    planet2.position = (-10, -20, -30)
    planet2.mass = 20
    
    data = [planet1,planet2]#, #planet2]
    root = Node([0,0,0],100)
    theta= 1
    #tree = Octree(data, root , theta)
    
##    root.create_subnode([0,0,0]) #rechts vorne oben
##    subnode = root.get_subnode([0,0,0])
##    print(subnode.middle)
##    print(subnode.get_corners())

    print(root.in_bounds(planet2.position))
    quad = root.get_quad(planet2.position)
    print(quad)
    root.create_subnode(quad)
    subnode = root.get_subnode(quad)
    print(subnode.middle)
    print(subnode.get_corners())

    
