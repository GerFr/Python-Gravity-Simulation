#Simulation of gravity
#29.03.2023
#Gerrit Fritz
import math

class Vector3D():

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z})"

    def __getitem__(self, item):
        if item == 0:
            return self.x
        elif item == 1:
            return self.y
        elif item == 2:
            return self.z
        else:
            raise indexError("there are only three values")
        
    def __add__(self, other):
        return Vector3D(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z)
               
    def __sub__(self, other):
        return Vector3D(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z)
    
    def __mul__(self,other):
        if isinstance(other, Vector3D):#dot product
            return(
                self.x * other.x +
                self.y * other.y +
                self.z * other.z)
        elif isinstance(other, (int, float)):#scalar multiplication
            return Vector3D(
                self.x * other,
                self.y * other,
                self.z * other)
        else:
            raise TypeError("operand must be Vector, int, or float")


    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Vector3D(
                self.x / other,
                self.y / other,
                self.z / other)
        else:
            raise TypeError("operand must be int or float")

    def get_magnitude(self):
        return(math.sqrt(self.x**2+self.y**2+self.z**2))
    
    def normalize(self):
        mag = self.get_magnitude()
        return Vector3D(
            self.x/mag,
            self.y/mag,
            self.z/mag)
        

##test = Vector3D(3, 6, 9)
##print(test.get_magnitude())
##print(test.normalize())
##print(test.normalize().get_magnitude())
