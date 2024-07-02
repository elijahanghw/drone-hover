from dronehover.bodies.standard_bodies import Quadcopter, Hexacopter

from dronehover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    length = 0.1
    drone = Hexacopter(length)

    # Define hovering optimizer for drone
    sim = Hover(drone)
    
    # Compute most efficient hover
    sim.compute_hover(verbose=True)