import numpy as np
from numpy import sin, cos, pi

class Biquadcopter:
    def __init__(self, length):
        # Inertia properties
        self.mass = 1
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
        # force: maximum force and torque (force, torque)
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, 1, 1], "force": [10, 1]},
                      {"loc":[length*cos(1/3*pi), length*sin(1/3*pi), 0], "dir": [0, 1, 0, -1], "force": [10, 1]},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 1, 0, 1], "force": [10, 1]},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, 1, -1], "force": [10, 1]},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 1, 0, -1], "force": [10, 1]},
                      {"loc":[length*cos(5/3*pi), length*sin(5/3*pi), 0], "dir": [0, 1, 0, 1], "force": [10, 1]}]
        
        self.num_props = len(self.props)
        
class Countercopter:
    def __init__(self):
        # Inertia properties
        self.mass = 1
        self.Ix = 1
        self.Iy = 1
        self.Iz = 1

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0
        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # force: maximum force and torque (force, torque)
        self.props = [{"loc":[0, 0, 1], "dir": [0, 0, 1, 1], "force": [10, 1]},
                      {"loc":[0, 0, -1], "dir": [0, 0, -1, 1], "force": [10, 1]}]
        
        self.num_props = len(self.props)
        
class Monocopter:
    def __init__(self):
        # Inertia properties
        self.mass = 1
        self.Ix = 1
        self.Iy = 1
        self.Iz = 1

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0
        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # force: maximum force and torque (force, torque)
        self.props = [{"loc":[0, 0, 1], "dir": [0, 0, 1, 1], "force": [10, 1]}]
        
        self.num_props = len(self.props)
        
class Dualquad:
    def __init__(self):
        # Inertia properties
        self.mass = 1
        self.Ix = 1
        self.Iy = 1
        self.Iz = 1

        self.Ixy = 0
        self.Ixz = 0
        self.Iyz = 0
        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # force: maximum force and torque (force, torque)
        self.props = [{"loc":[1, 1, 0], "dir": [0, 0, 1, 1], "force": [10, 1]},
                      {"loc":[-1, 1, 0], "dir": [0, 0, 1, -1], "force": [10, 1]},
                      {"loc":[-1, -1, 0], "dir": [0, 0, 1, 1], "force": [10, 1]},
                      {"loc":[1, -1, 0], "dir": [0, 0, 1, -1], "force": [10, 1]},
                      {"loc":[1, 1, 0], "dir": [0, 0, -1, 1], "force": [10, 1]},
                      {"loc":[-1, 1, 0], "dir": [0, 0, -1, -1], "force": [10, 1]},
                      {"loc":[-1, -1, 0], "dir": [0, 0, -1, 1], "force": [10, 1]},
                      {"loc":[1, -1, 0], "dir": [0, 0, -1, -1], "force": [10, 1]}]
        
        self.num_props = len(self.props)