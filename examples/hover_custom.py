import numpy as np
from numpy import sin, cos, pi

from dronehover.bodies.custom_bodies import Custombody

from dronehover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    props = [{"loc":[cos(1/4*pi), sin(1/4*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[cos(3/4*pi), sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[cos(5/4*pi), sin(5/4*pi), 0], "dir": [0, 0, -1, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[cos(7/4*pi), sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[1, 0, 1], "dir": [1, 0, 0, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[0, 0, 0], "dir": [1, 0, 0, "ccw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927},
                      {"loc":[0, 0, 0], "dir": [1, 0, 0, "cw"], "constants": [7.24e-07, 8.20e-09], "wmax": 3927}]
    
    mass = 1
    cg = [0,0,0]
    Ix = 0.1
    Iy = 0.1
    Iz = 0.5
    Ixy = 0
    Ixz = 0
    Iyz = 0
    
    drone = Custombody(mass, cg, Ix, Iy, Iz, Ixy, Ixz, Iyz, props)

    # Define hovering optimizer for drone
    sim = Hover(drone)
    
    # Compute most efficient hover
    sim.compute_hover(verbose=True)