from dronehover.bodies.standard_bodies import Tricopter

from dronehover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    length = 1
    drone = Tricopter(length)

    # Define hovering optimizer for drone
    sim = Hover(drone)
    
    # Compute most efficient hover
    sim.compute_hover(verbose=True)
    
    print(sim.hover_status)