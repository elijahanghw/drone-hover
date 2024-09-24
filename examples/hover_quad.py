from dronehover.bodies.standard_bodies import Quadcopter

from dronehover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    length = 0.110
    drone = Quadcopter(length)
    print(drone.props)

    # Define hovering optimizer for drone
    sim = Hover(drone)
    
    # Compute most efficient hover
    sim.compute_hover(verbose=True, tol=1e-10)