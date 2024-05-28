from dronehover.bodies.standard_bodies import Quadcopter

from dronehover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    length = 1
    drone = Quadcopter(length)

    # Define hovering optimizer for drone
    sim = Hover(drone)
    
    # Compute most efficient hover
    sim.compute_hover()