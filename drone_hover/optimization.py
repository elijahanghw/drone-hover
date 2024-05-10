import numpy as np
from numpy.linalg import inv, pinv, norm
from scipy.linalg import null_space
from scipy.optimize import minimize

G = 9.81

class Hover:
    def __init__(self, drone):
        self.drone = drone
        
        self.drone_checker()
        
        m = drone.mass * np.eye(3)
        
        I = np.array([[drone.Ix, 0, 0],
                      [0, drone.Iy, 0],
                      [0, 0, drone.Iz]])

        self.control_bounds = np.array((0.02, 1))

        # Compute effectiveness matrices Bf and Bm
        self.Bf = np.zeros((3, drone.num_props))
        self.Bm = np.zeros((3, drone.num_props))

        for idx in range(drone.num_props):
            prop_size = drone.prop_size[idx]
            self.Bf[:,idx] = drone.f_max[prop_size] * drone.prop_dir[idx,:3].T
            
            self.Bm[:,idx] = (np.cross(drone.prop_loc[idx,:], drone.f_max[prop_size]*drone.prop_dir[idx,:3])
                        - drone.tw_max[prop_size]*drone.prop_dir[idx,:3]*drone.prop_dir[idx,3:]).T
            
        self.Bf = inv(m) @ self.Bf
        self.Bm = inv(I) @ self.Bm

        self.W = np.eye(drone.num_props)
        
        self.control_limits = np.ones((drone.num_props, 2))
        self.control_limits[:,0] *= self.control_bounds[0]
        self.control_limits[:,1] *= self.control_bounds[1]
        
            
    def static(self):
        print("Testing static hover...")
        A = self.Bf.T @ self.Bf
        
        u0 = np.random.uniform(low=self.control_bounds[0], high=self.control_bounds[1], size=self.drone.num_props)
        
        def objective_function(u):
            return u.T @ u
        
        def force_constraint(u):
            return u.T @ A @ u - G**2

        def moment_constraint(u):
            return u.T @ self.Bm.T @ self.Bm @ u
        
        cons = [{"type":"eq", "fun":force_constraint},
                {"type":"eq", "fun":moment_constraint}]
        
        bnds = []
        for i in range(self.control_limits.shape[0]):
            bnds.append((self.control_bounds[0], self.control_bounds[1])) 
        opt = {'maxiter':1000}
        
        static_hover = minimize(objective_function, u0, constraints=cons, bounds=bnds, method='SLSQP', options=opt)
        self.static_success = static_hover.success
        
        # Checking if no torque configuration can achieve sufficient thrust
        if static_hover.success == True:
            print("----------Static Hover Achieved----------")
            self.u = static_hover.x
            f = self.Bf @ self.u
            tau = self.Bm @ self.u
            input_cost = self.u.T @ self.u
            print(f'Optimum input = {self.u}')
            print(f'Resultant specific force: {norm(f)}')
            print(f'Resultant specific torque: {norm(tau)}')
            print(f"Input cost: {input_cost}")

    
        else:
            print("Drone cannot achieve static hover")

    
    def spinning(self):
        print("Testing spinning hover...")
        A = self.Bf.T @ self.Bf
        
        # Somehow if values of u are all equal it does not work
        u0 = np.random.uniform(low=self.control_bounds[0], high=self.control_bounds[1], size=self.drone.num_props)
        
        def objective_function(u):
            return u.T @ u
        
        def force_constraint(u):
            return u.T @ A @ u - G**2
        
        def moment_constraint(u):
            # This SLSQP constraint does not work for Monocopter and Countercopter
            # cross(f,tau) always 0, constraint cannot differentiate twice
            # i.e. constraint automatically satisfied            
            f = self.Bf @ u
            tau = self.Bm @ u
            return norm(np.cross(f, tau))
        
        cons = [{"type":"eq", "fun":force_constraint},
                {"type":"eq", "fun":moment_constraint}]
        
        bnds = []
        for i in range(self.control_limits.shape[0]):
            bnds.append((self.control_bounds[0], self.control_bounds[1])) 
        opt = {'maxiter':1000}
        
        spinning_hover = minimize(objective_function, u0, constraints=cons, bounds=bnds, method='SLSQP', options=opt)
        if spinning_hover.success == True:
            print("----------Spinning Hover Achieved----------")
            self.u = spinning_hover.x
            f = self.Bf @ self.u
            tau = self.Bm @ self.u
            input_cost = self.u.T @ self.u
            print(f'Optimum input = {self.u}')
            print(f'Resultant specific force: {norm(f)}')
            print(f'Resultant specific torque: {norm(tau)}')
            print(f"Force-torque cross product norm: {norm(np.cross(f,tau))}")
            print(f"Input cost: {input_cost}")
            
        else:
            print("----------Drone Cannot Hover----------")
            self.u = spinning_hover.x
            f = self.Bf @ self.u
            tau = self.Bm @ self.u
            print(f'Best input = {self.u}')
            print(f'Resultant specific force: {norm(f)}')
            print(f'Resultant specific torque: {norm(tau)}')
            print(f"Force-torque cross product norm: {norm(np.cross(f,tau))}")
            
    def drone_checker(self):
        
        return