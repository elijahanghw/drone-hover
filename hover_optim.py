from drone_hover.standard_bodies import Quadcopter, Tricopter, Hexacopter, Octacopter
from drone_hover.custom_bodies import Biquadcopter, Countercopter, Monocopter, Dualquad

from drone_hover.optimization import Hover

if __name__ == "__main__":
    # Import drone body
    length = 1
    drone = Hexacopter(length)

    # Define hovering optimizer for drone
    sim = Hover(drone)
    
    # Compute most efficient hover
    sim.compute_hover()