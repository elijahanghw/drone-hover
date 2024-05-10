import numpy as np
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
        
        # Drone parameters (for now standard quadcopter)
        # prop_loc: Propeller location (x, y, z)
        self.prop_loc = np.array([[length*cos(1/4*pi), length*sin(1/4*pi), 0],   
                                  [length*cos(3/4*pi), length*sin(3/4*pi), 0],
                                  [length*cos(5/4*pi), length*sin(5/4*pi), 0],
                                  [length*cos(7/4*pi), length*sin(7/4*pi), 0]])

        # prop_dir: Unit vector of propeller direction (x,y,z,r=1(ccw) or -1(cw))
        self.prop_dir = np.array([[0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1],
                                  [0, 0, 1, -1]])
        
        # Propeller properties
        self.f_max = np.array([10])   # max thrust for each propeller
        self.tw_max = np.array([1])  # max torque for each propeller
        
        self.prop_size = np.array([0, 0, 0, 0])


        self.num_props = self.prop_loc.shape[0]

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

        # Drone parameters (for now standard quadcopter)
        # prop_loc: Propeller location (x, y, z)
        self.prop_loc = np.array([[length, 0, 0],   
                                  [length*cos(2/3*pi), length*sin(2/3*pi), 0],
                                  [length*cos(4/3*pi), length*sin(4/3*pi), 0]])

        # prop_dir: Unit vector of propeller direction (x,y,z,r=1(ccw) or -1(cw))
        self.prop_dir = np.array([[0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1]])
        
        # Propeller properties
        self.f_max = np.array([10])   # max thrust for each propeller
        self.tw_max = np.array([1])  # max torque for each propeller
        
        self.prop_size = np.array([0, 0, 0])

        self.num_props = self.prop_loc.shape[0]
        

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

        # Drone parameters (for now standard quadcopter)
        # prop_loc: Propeller location (x, y, z)
        self.prop_loc = np.array([[length, 0, 0],   
                                  [length*cos(1/3*pi), length*sin(1/3*pi), 0],
                                  [length*cos(2/3*pi), length*sin(2/3*pi), 0],
                                  [length*cos(pi), length*sin(pi), 0],
                                  [length*cos(4/3*pi), length*sin(4/3*pi), 0],
                                  [length*cos(5/3*pi), length*sin(5/3*pi), 0]])

        # prop_dir: Unit vector of propeller direction (x,y,z,r=1(ccw) or -1(cw))
        self.prop_dir = np.array([[0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1],
                                  [0, 0, 1, -1]])
        
        # Propeller properties
        self.f_max = np.array([10])   # max thrust for each propeller
        self.tw_max = np.array([1])  # max torque for each propeller
        
        self.prop_size = np.array([0, 0, 0, 0, 0, 0])

        self.num_props = self.prop_loc.shape[0]
        

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

        # Drone parameters (for now standard quadcopter)
        # prop_loc: Propeller location (x, y, z)
        self.prop_loc = np.array([[length, 0, 0],   
                                  [length*cos(1/4*pi), length*sin(1/4*pi), 0],
                                  [length*cos(2/4*pi), length*sin(2/4*pi), 0],
                                  [length*cos(3/4*pi), length*sin(3/4*pi), 0],
                                  [length*cos(pi), length*sin(pi), 0],
                                  [length*cos(5/4*pi), length*sin(5/4*pi), 0],
                                  [length*cos(6/4*pi), length*sin(6/4*pi), 0],
                                  [length*cos(7/4*pi), length*sin(7/4*pi), 0]])

        # prop_dir: Unit vector of propeller direction (x,y,z,r=1(ccw) or -1(cw))
        self.prop_dir = np.array([[0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1],
                                  [0, 0, 1, -1],
                                  [0, 0, 1, 1],
                                  [0, 0, 1, -1]])

        # Propeller properties
        self.f_max = np.array([10])   # max thrust for each propeller
        self.tw_max = np.array([1])  # max torque for each propeller
        
        self.prop_size = np.array([0, 0, 0, 0, 0, 0, 0, 0])

        self.num_props = self.prop_loc.shape[0]