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
        
        cg = np.asarray(drone.cg)
        
        I = np.array([[drone.Ix, 0, 0],
                    [0, drone.Iy, 0],
                    [0, 0, drone.Iz]])

        self.w_hat_bounds = np.array((0.02, 1))

        # Compute effectiveness matrices Bf and Bm
        self.Bf = np.zeros((3, self.num_props))
        self.Bm = np.zeros((3, self.num_props))

        for idx, prop in enumerate(drone.props):
            k_f = prop["constants"][0]
            k_m = prop["constants"][1]
            w_max = prop["wmax"] 
            prop_loc = np.array(prop["loc"])[np.newaxis,:] # Position of propeller from body-fixed axis
            prop_r = prop_loc - cg[np.newaxis,:]    # Position of propeller from C.G.
            prop_dir = np.array(prop["dir"][:3]) # Direction of thrust vector
            prop_dir = (prop_dir/norm(prop_dir))[np.newaxis,:]
            
            prop_rot = 1 if prop["dir"][-1] == "ccw" else -1    # Direction of propeller rotation
            
            self.Bf[:,idx] = k_f * w_max**2 * prop_dir[0,:].T
            
            self.Bm[:,idx] = (np.cross(prop_r[0,:], k_f*w_max**2*prop_dir[0,:])
                            + k_m*w_max**2*prop_rot*prop_dir[0,:]).T
            
            
        self.Bf = inv(m) @ self.Bf
        
        self.Bm = inv(I) @ self.Bm

        self.W = np.eye(self.num_props)
        
        self.control_limits = np.ones((self.num_props, 2))
        self.control_limits[:,0] *= self.w_hat_bounds[0]
        self.control_limits[:,1] *= self.w_hat_bounds[1]
        
        
    def compute_hover(self, verbose=False):
        """Calls the static function to check if drone is able to achieve static hover.
           If static hover fails, call spinning function.
        """        
        self.static(verbose)
        if self.static_success == False:
            self.spinning(verbose)
        
            
    def static(self, verbose):
        """Check if drone is able to achieve static hover.
           Prints hovering capability, optimal hovering inputs and input cost.
        """ 
        if verbose:
            print("Testing static hover...")
            
        A = self.Bf.T @ self.Bf
        
        # Defining eta as a shorthand (eta = w_hat**2)
        eta0 = np.random.uniform(low=self.w_hat_bounds[0]**2, high=self.w_hat_bounds[1]**2, size=self.num_props)
        
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
            bnds.append((self.w_hat_bounds[0]**2, self.w_hat_bounds[1]**2)) 
        opt = {'maxiter':100}
        
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Values in x were outside bounds")
            static_hover = minimize(objective_function, eta0, constraints=cons, bounds=bnds, method='SLSQP', options=opt)
            
        self.static_success = static_hover.success
        
        # Checking if no torque configuration can achieve sufficient thrust
        if static_hover.success == True:
            self.hover_status = "ST"
            self.eta = static_hover.x
            self.w_hat = np.sqrt(self.eta)
            self.u = self.w_to_u(self.w_hat)
            f = self.Bf @ self.eta
            self.tau = self.Bm @ self.eta
            self.input_cost = self.eta.T @ self.eta
            
            self.u_max = self.u / max(self.u)
            self.w_hat_max = self.u_to_w(self.u_max)
            
            self.f_max = self.Bf @ (self.w_hat_max)**2
            
            if verbose:
                print("----------Static Hover Achieved----------")
                print(f'Optimum input = {self.u}')
                print(f'Thrust vector direction: {f/norm(f)}')
                print(f'Resultant specific force: {norm(f):.2f}')
                print(f'Resultant specific torque: {norm(self.tau):.2f}')
                print(f'Max thrust to weight: {norm(self.f_max)/G:.2f}')
                print(f"Input cost: {self.input_cost:.5f}")

        else:
            if verbose:
                print("Drone cannot achieve static hover")

    
    def spinning(self, verbose):
        """Check if drone is able to achieve spinning hover.
           Prints hovering capability, optimal hovering inputs and input cost.
        """        
        if verbose:
            print("Testing spinning hover...")
        A = self.Bf.T @ self.Bf
        
        # Defining eta as a shorthand (eta = u**2)
        # Somehow if values of u are all equal it does not work
        eta0 = np.random.uniform(low=self.w_hat_bounds[0]**2, high=self.w_hat_bounds[1]**2, size=self.num_props)
        
        def objective_function(eta):
            return eta.T @ eta
        
        def force_constraint(eta):
            return eta.T @ A @ eta - G**2
        
        def moment_constraint(eta):
            # This SLSQP constraint does not work for if cross(f,tau) always 0
            # Constraint cannot be differentiated twice
            # i.e. constraint automatically satisfied            
            f = self.Bf @ eta
            tau = self.Bm @ eta
            return norm(np.cross(f, tau))
        
        cons = [{"type":"eq", "fun":force_constraint},
                {"type":"eq", "fun":moment_constraint}]
        
        bnds = []
        for i in range(self.control_limits.shape[0]):
            bnds.append((self.w_hat_bounds[0]**2, self.w_hat_bounds[1]**2)) 
        opt = {'maxiter':100}
        
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Values in x were outside bounds")
            spinning_hover = minimize(objective_function, eta0, constraints=cons, bounds=bnds, method='SLSQP', options=opt)
        
        self.spinning_success = spinning_hover.success
        
        if spinning_hover.success == True:
            self.hover_status = "SP"
            self.eta = spinning_hover.x
            self.w_hat = np.sqrt(self.eta)
            self.u = self.w_to_u(self.w_hat)
            f = self.Bf @ self.eta
            self.tau = self.Bm @ self.eta
            self.input_cost = self.eta.T @ self.eta
            
            self.u_max = self.u / max(self.u)
            self.w_hat_max = self.u_to_w(self.u_max)
            
            self.f_max = self.Bf @ (self.w_hat_max)**2
            
            if verbose:
                print("----------Spinning Hover Achieved----------")
                print(f'Optimum input = {self.u}')
                print(f'Thrust vector direction: {f/norm(f)}')
                print(f'Resultant specific force: {norm(f):.2f}')
                print(f'Resultant specific torque: {norm(self.tau):.2f}')
                print(f"Force-torque cross product norm: {norm(np.cross(f,self.tau)):.5f}")
                print(f'Max thrust to weight: {norm(self.f_max)/G:.2f}')
                print(f"Input cost: {self.input_cost}")
            
        else:
            self.hover_status = "N"
            self.eta = spinning_hover.x
            self.u = np.sqrt(self.eta)
            f = self.Bf @ self.eta
            self.tau = self.Bm @ self.eta
            
            if verbose:
                print("----------Drone Cannot Hover----------")
                print(f'Best input = {self.u}')
                print(f'Resultant specific force: {norm(f):.2f}')
                print(f'Resultant specific torque: {norm(self.tau):.2f}')
                print(f"Force-torque cross product norm: {norm(np.cross(f,self.tau)):.5f}")
            
            
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
                
            if prop["dir"][-1] != "ccw" and prop["dir"][-1] != "cw":
                raise ValueError(f"Invalid value for propeller spinning direction. Use only \"ccw\" or \"cw\"")
           
                
    def w_to_u(self, w_hat):
        return (w_hat - 0.02)/0.98
    
    def u_to_w(self, u):
        return 0.02 + 0.98*u