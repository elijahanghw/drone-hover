from numpy import sin, cos, pi
                
# Standard x config quadcopter
class Quadcopter:
    def __init__(self, length):
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
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927}]
        

# Standard tricopter without tilt rotor
class Tricopter:
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
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927}]
        

# Standard hexacopter  
class Hexacopter:
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
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(1/3*pi), length*sin(1/3*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(5/3*pi), length*sin(5/3*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927}]
        

# Standard Octacopter
class Octacopter:
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
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(2/4*pi), length*sin(2/4*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(6/4*pi), length*sin(6/4*pi), 0], "dir": [0, 0, -1, 1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927},
                      {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, -1], "constants": [7.25e-07, 2.69e-08], "wmax": 3927}]