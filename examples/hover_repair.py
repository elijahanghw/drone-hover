import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import norm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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


def plot_props(props_list, titles, colors=None):
    """
    Plot multiple propeller configurations in 3D.
    
    Args:
        props_list: List of propeller configurations to plot
        titles: List of titles for each subplot
        colors: Optional list of colors for each configuration
    """
    if colors is None:
        colors = ['blue', 'red', 'green', 'purple']
    
    n_configs = len(props_list)
    fig = plt.figure(figsize=(5*n_configs, 5))
    
    for idx, (props, title) in enumerate(zip(props_list, titles)):
        ax = fig.add_subplot(1, n_configs, idx+1, projection='3d')
        
        # Extract positions and directions
        for i, prop in enumerate(props):
            loc = np.array(prop["loc"])
            direction = np.array(prop["dir"][0:3])
            
            # Plot propeller position
            ax.scatter(loc[0], loc[1], loc[2], c=colors[idx], s=100, marker='o')
            
            # Plot thrust direction as arrow
            arrow_scale = 0.03
            ax.quiver(loc[0], loc[1], loc[2], 
                     direction[0], direction[1], direction[2],
                     length=arrow_scale, color=colors[idx], arrow_length_ratio=0.3, linewidth=2)
            
            # Label propeller
            ax.text(loc[0], loc[1], loc[2], f'  P{i+1}', fontsize=8)
            
            # Draw line from motor to origin (0,0,0)
            ax.plot([0, loc[0]], [0, loc[1]], [0, loc[2]], 
                   'k-', linewidth=2, alpha=0.5)
        
        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title)
        
        # Set equal aspect ratio
        max_range = 0.08
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])
        
        # Invert Z axis
        ax.invert_zaxis()
        
        # Add grid
        ax.grid(True)
    
    plt.tight_layout()
    # plt.savefig("propellers.png")
    plt.show()



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

    # Plot all configurations
    plot_props([rotated_props, repaired_props], 
               ['Rotated Quad', 'Repaired Quad'],
               colors=['blue', 'red'])



    
