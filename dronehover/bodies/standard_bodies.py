import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import norm as norm
                
# Standard x config quadcopter
class Quadcopter:
    def __init__(self, length):        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]

        self.get_inertia()

    def get_inertia(self):
        controller_mass = 0.3 # based on 4S, 3200 mAh lipo
        prop_mass = 0.018 # based on 1806 BLDC
        beam_density = 1650*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass + prop_mass*len(self.props)

        self.cg = np.zeros(3)
        for prop in self.props:
            self.mass += beam_density*np.linalg.norm(np.array(prop["loc"])) # Add mass of carbon beam
            self.cg += prop_mass/self.mass * np.array(prop["loc"])

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass

        for prop in self.props:
            r = np.asarray(prop["loc"]) - self.cg
            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass
            self.Iy += norm(np.cross(np.array([1,0,0]),r))**2 * prop_mass
            self.Iz += norm(np.cross(np.array([1,0,0]),r))**2 * prop_mass

        self.cg = self.cg.tolist()
        

# Standard tricopter without tilt rotor
class Tricopter:
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
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]
        

# Standard hexacopter  
class Hexacopter:
    def __init__(self, length):
        
        self.length = length
        
        # Propeller parameters
        # loc: Propeller location (x, y, z)
        # dir: Unit vector of propeller direction + rotation direction (x,y,z,r=1(ccw) or -1(cw))
        # constants: propeller force and torque constants (force, torque)
        # wmax: maximum rotation speed in rad/s
        
        self.props = [{"loc":[length, 0, 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(1/3*pi), length*sin(1/3*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(2/3*pi), length*sin(2/3*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(4/3*pi), length*sin(4/3*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(5/3*pi), length*sin(5/3*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]
        self.get_inertia()

    def get_inertia(self):
        controller_mass = 0.3 # based on 4S, 3200 mAh lipo
        prop_mass = 0.018 # based on 1806 BLDC
        beam_density = 1650*0.005*0.01 # kg/m, carbon fiber plates, 5mm thickness, 10mm width

        self.mass = controller_mass + prop_mass*len(self.props)

        self.cg = np.zeros(3)
        for prop in self.props:
            self.mass += beam_density*np.linalg.norm(np.array(prop["loc"])) # Add mass of carbon beam
            self.cg += prop_mass/self.mass * np.array(prop["loc"])

        self.Ix = norm(np.cross(np.array([1,0,0]),self.cg))**2 * controller_mass
        self.Iy = norm(np.cross(np.array([0,1,0]),self.cg))**2 * controller_mass
        self.Iz = norm(np.cross(np.array([0,0,1]),self.cg))**2 * controller_mass

        for prop in self.props:
            r = np.asarray(prop["loc"]) - self.cg
            self.Ix += norm(np.cross(np.array([1,0,0]),r))**2  * prop_mass
            self.Iy += norm(np.cross(np.array([1,0,0]),r))**2 * prop_mass
            self.Iz += norm(np.cross(np.array([1,0,0]),r))**2 * prop_mass

        self.cg = self.cg.tolist()

# Standard Octacopter
class Octacopter:
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
                      {"loc":[length*cos(1/4*pi), length*sin(1/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(2/4*pi), length*sin(2/4*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(3/4*pi), length*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(pi), length*sin(pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(5/4*pi), length*sin(5/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(6/4*pi), length*sin(6/4*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[length*cos(7/4*pi), length*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]