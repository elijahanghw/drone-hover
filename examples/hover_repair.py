import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import norm

from dronehover.bodies.custom_bodies import Custombody
from dronehover.optimization import Hover
from dronehover.utils import *

def hover_repair(drone):
    sim = Hover(drone)
    sim.compute_hover(verbose=True)

    if sim.static_success:
        # Get thrust direction
        f = sim.Bf @ sim.eta
        f = f / norm(f)

        R2 = align_vectors(f, [0, 0, -1])

        repaired_props = []

        for prop in drone.props:
            # Create a copy of the prop dictionary
            new_prop = prop.copy()
            
            new_pos = R2 @ np.array(prop["loc"])
            new_dir = R2 @ np.array(prop["dir"][0:3])

            new_pos = new_pos.tolist()
            new_dir = new_dir.tolist()

            new_prop["loc"] = new_pos
            new_prop["dir"] = new_dir + [prop["dir"][3]]  # Keep rotation direction (ccw/cw)

            repaired_props.append(new_prop)

        repaired_drone = Custombody(repaired_props)         # Automatic inertia computation

        print("Drone repaired for hover. Returning new drone.")
        return repaired_drone, repaired_props
    
    else:
        print("Drone cannot hover, not repaired. Returning original drone")
        return drone, drone.props


if __name__ == "__main__":
    # Standard Quad
    props = [{"loc":[0.060*cos(1/4*pi), 0.060*sin(1/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 2},
                      {"loc":[0.060*cos(3/4*pi), 0.060*sin(3/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 2},
                      {"loc":[0.060*cos(5/4*pi), 0.060*sin(5/4*pi), 0], "dir": [0, 0, -1, "ccw"], "propsize": 2},   
                      {"loc":[0.060*cos(7/4*pi), 0.060*sin(7/4*pi), 0], "dir": [0, 0, -1, "cw"], "propsize": 2}]

    # Randomly rotate Quad by pitch and roll
    R1 = rotation_matrix(roll=np.random.uniform(-pi, pi), pitch=np.random.uniform(-pi, pi), yaw=np.random.uniform(-pi, pi))
    
    rotated_props = []

    for prop in props:
        # Create a copy of the prop dictionary
        new_prop = prop.copy()
        
        new_pos = R1 @ np.array(prop["loc"])
        new_dir = R1 @ np.array(prop["dir"][0:3])

        new_pos = new_pos.tolist()
        new_dir = new_dir.tolist()

        new_prop["loc"] = new_pos
        new_prop["dir"] = new_dir + [prop["dir"][3]]  # Keep rotation direction (ccw/cw)

        rotated_props.append(new_prop)

    # Test rotated drone
    rotated_drone = Custombody(rotated_props)
    sim = Hover(rotated_drone)
    sim.compute_hover(verbose=True)

    # Repair drone
    repaired_drone, repaired_props = hover_repair(rotated_drone)

    # Test repaired drone
    sim = Hover(repaired_drone)
    sim.compute_hover(verbose=True)

    print(repaired_props)


    
