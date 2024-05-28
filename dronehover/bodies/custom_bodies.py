import numpy as np
from numpy import sin, cos, pi

class Custombody:
    def __init__(self, mass, cg, Ix, Iy, Iz, Ixy, Ixz, Iyz, props):
        """Class for custom drone bodies

        Args:
            mass (float): Mass of the drone
            Ix (float): Moment of inertia about x axis
            Iy (float): Moment of inertia about y axis
            Iz (float): Moment of inertia about z axis
            Ixy (float): Products of inertia (x-y)
            Ixz (float): Products of inertia (x-z)
            Iyz (float): Products of inertia (y-z)
            props (dict): Propeller properties
        """        
        self.mass = mass
        self.cg = cg
        
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz
        
        self.Ixy = Ixy
        self.Ixz = Ixz
        self.Iyz = Iyz
        
        self.props = props
        

class Biquadcopter:
    def __init__(self, length):
        # Inertia properties
        self.mass = 1
        self.cg = [0, 0, 0]
        
        self.Ix = 1
        self.Iy = 1
        self.Iz = 1

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0
        
        self.length = length
        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(1/3*pi), length*sin(1/3*pi), 0], "dir": [0, 1, 0, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 1, 0, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 1, 0, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(5/3*pi), length*sin(5/3*pi), 0], "dir": [0, 1, 0, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]

        
        
class Dualquad:
    def __init__(self):
        # Inertia properties
        self.mass = 1
        self.cg = [0, 0, 0]
        
        self.Ix = 1
        self.Iy = 1
        self.Iz = 1

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0
        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[1, 1, 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[-1, 1, 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[-1, -1, 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[1, -1, 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[1, 1, 0], "dir": [0, 0, 1, "ccw"], "constants": [7.60e-06, 1.14e-07], "wmax": 1963},
                      {"loc":[-1, 1, 0], "dir": [0, 0, 1, "cw"], "constants": [7.60e-06, 1.14e-07], "wmax": 1963},
                      {"loc":[-1, -1, 0], "dir": [0, 0, 1, "ccw"], "constants": [7.60e-06, 1.14e-07], "wmax": 1963},
                      {"loc":[1, -1, 0], "dir": [0, 0, 1, "cw"], "constants": [7.60e-06, 1.14e-07], "wmax": 1963}]