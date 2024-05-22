import warnings
import numpy as np
from numpy.linalg import inv, norm
from scipy.optimize import minimize

G = 9.81    # gravitational acceleration

class Hover:
    def __init__(self, drone):
        """Optimal hover optimizer which computes the hovering capabilities of a drone.

        Args:
            drone (class): Drone class containing inertial properties and propeller configurations.
        """        
        self.drone = drone
        self.num_props = len(self.drone.props)
        
        self.drone_checker()
        
        m = drone.mass * np.eye(3)
        
        I = np.array([[drone.Ix, 0, 0],
                      [0, drone.Iy, 0],
                      [0, 0, drone.Iz]])

        self.command_bounds = np.array((0.02, 1))

        # Compute effectiveness matrices Bf and Bm
        self.Bf = np.zeros((3, self.num_props))
        self.Bm = np.zeros((3, self.num_props))

        for idx, prop in enumerate(drone.props):
            k_f = prop["constants"][0]
            k_m = prop["constants"][1]
            w_max = prop["wmax"] 
            prop_loc = np.array(prop["loc"])[np.newaxis,:]
            prop_dir = np.array(prop["dir"])[np.newaxis,:]
            prop_dir[0,:3] = prop_dir[0,:3]/norm(prop_dir[0,:3])
            
            self.Bf[:,idx] = k_f * w_max * prop_dir[0,:3].T
            
            self.Bm[:,idx] = (np.cross(prop_loc[0,:], k_f*w_max**2*prop_dir[0,:3])
                              + k_m*w_max**2*prop_dir[0,:3]*prop_dir[0,3:]).T
            
        self.Bf = inv(m) @ self.Bf
        self.Bm = inv(I) @ self.Bm

        self.W = np.eye(self.num_props)
        
        self.control_limits = np.ones((self.num_props, 2))
        self.control_limits[:,0] *= self.command_bounds[0]
        self.control_limits[:,1] *= self.command_bounds[1]
        
        
    def compute_hover(self):
        """Calls the static function to check if drone is able to achieve static hover.
           If static hover fails, call spinning function.
        """        
        self.static()
        if self.static_success == False:
            self.spinning()
        
            
    def static(self):
        """Check if drone is able to achieve static hover.
           Prints hovering capability, optimal hovering inputs and input cost.
        """ 
        print("Testing static hover...")
        A = self.Bf.T @ self.Bf
        
        # Defining eta as a shorthand (eta = u**2)
        eta0 = np.random.uniform(low=self.command_bounds[0]**2, high=self.command_bounds[1]**2, size=self.num_props)
        
        def objective_function(eta):
            return eta.T @ eta
        
        def force_constraint(eta):
            return eta.T @ A @ eta - G**2

        def moment_constraint(eta):
            return eta.T @ self.Bm.T @ self.Bm @ eta
        
        cons = [{"type":"eq", "fun":force_constraint},
                {"type":"eq", "fun":moment_constraint}]
        
        bnds = []
        for i in range(self.control_limits.shape[0]):
            bnds.append((self.command_bounds[0]**2, self.command_bounds[1]**2)) 
        opt = {'maxiter':1000}
        
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Values in x were outside bounds")
            static_hover = minimize(objective_function, eta0, constraints=cons, bounds=bnds, method='SLSQP', options=opt)
            
        self.static_success = static_hover.success
        
        # Checking if no torque configuration can achieve sufficient thrust
        if static_hover.success == True:
            print("----------Static Hover Achieved----------")
            self.eta = static_hover.x
            self.u = np.sqrt(self.eta)
            f = self.Bf @ self.eta
            tau = self.Bm @ self.eta
            input_cost = self.eta.T @ self.eta
            print(f'Optimum input = {self.u}')
            print(f'Resultant specific force: {norm(f):.2f}')
            print(f'Resultant specific torque: {norm(tau):.2f}')
            print(f"Input cost: {input_cost:.5f}")

        else:
            print("Drone cannot achieve static hover")

    
    def spinning(self):
        """Check if drone is able to achieve spinning hover.
           Prints hovering capability, optimal hovering inputs and input cost.
        """        
        print("Testing spinning hover...")
        A = self.Bf.T @ self.Bf
        
        # Defining eta as a shorthand (eta = u**2)
        # Somehow if values of u are all equal it does not work
        eta0 = np.random.uniform(low=self.command_bounds[0]**2, high=self.command_bounds[1]**2, size=self.num_props)
        
        def objective_function(eta):
            return eta.T @ eta
        
        def force_constraint(eta):
            return eta.T @ A @ eta - G**2
        
        def moment_constraint(eta):
            # This SLSQP constraint does not work for Monocopter and Countercopter
            # cross(f,tau) always 0, constraint cannot be differentiated twice
            # i.e. constraint automatically satisfied            
            f = self.Bf @ eta
            tau = self.Bm @ eta
            return norm(np.cross(f, tau))
        
        cons = [{"type":"eq", "fun":force_constraint},
                {"type":"eq", "fun":moment_constraint}]
        
        bnds = []
        for i in range(self.control_limits.shape[0]):
            bnds.append((self.command_bounds[0]**2, self.command_bounds[1]**2)) 
        opt = {'maxiter':1000}
        
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Values in x were outside bounds")
            spinning_hover = minimize(objective_function, eta0, constraints=cons, bounds=bnds, method='SLSQP', options=opt)
            
        if spinning_hover.success == True:
            print("----------Spinning Hover Achieved----------")
            self.eta = spinning_hover.x
            self.u = np.sqrt(self.eta)
            f = self.Bf @ self.eta
            tau = self.Bm @ self.eta
            input_cost = self.eta.T @ self.eta
            print(f'Optimum input = {self.u}')
            print(f'Resultant specific force: {norm(f)}:.2f')
            print(f'Resultant specific torque: {norm(tau)}:.2f')
            print(f"Force-torque cross product norm: {norm(np.cross(f,tau)):.5f}")
            print(f"Input cost: {input_cost}")
            
        else:
            print("----------Drone Cannot Hover----------")
            self.eta = spinning_hover.x
            self.u = np.sqrt(self.eta)
            f = self.Bf @ self.eta
            tau = self.Bm @ self.eta
            print(f'Best input = {self.u}')
            print(f'Resultant specific force: {norm(f):.2f}')
            print(f'Resultant specific torque: {norm(tau):.2f}')
            print(f"Force-torque cross product norm: {norm(np.cross(f,tau)):.5f}")
            
            
    def drone_checker(self):
        """Check that drone propeller dictionary has the required format.

        Raises:
            KeyError: Required key in propellers dictionary missing.
        """        
        keys = ["loc", "dir", "constants", "wmax"]
        for i, prop in enumerate(self.drone.props):
            for key in keys:
                if key in prop.keys():
                    pass
                else:
                    raise KeyError(f"\"{key}\" is missing in propeller {i}")