""" 
Sequential Convex Programming (SCP) Class for 2D Rover Path Planning.
Algorithm based on: "GuSTO: Guaranteed Sequential Trajectory Optimization via Sequential Convex Programming" by R. Bonalli et al. (ICRA, 2019).
"""

import cvxpy as cp
import numpy as np
import jax.numpy as onp

class SCP():
    def __init__(self, Rover, initialization):
        # Rover and Constraints Parameters:
        self.Rover = Rover
        self.f_continuous = Rover.f_continuous
        self.A = Rover.A
        self.B = Rover.B
        self.c = Rover.c
        self.dt = Rover.dt
        self.num_states = Rover.num_states
        self.num_control = Rover.num_control
        self.u_max = Rover.u_max
        self.u_min = Rover.u_min
        self.initial_state = Rover.x0
        self.terminal_state = Rover.xf
        self.state_min = Rover.state_min
        self.state_max = Rover.state_max
        self.sdf = Rover.sdf
        self.dmin = Rover.dmin
        self.sdf_value = Rover.sdf_value
        self.sdf_gradient = Rover.sdf_gradient
        self.delta = Rover.delta
        # Initial Guess:
        self.initialization = initialization
        self.nominal_trajectory = {}
        self.nominal_trajectory["valid"] = self.initialization["valid"]
        if self.nominal_trajectory["valid"] == True:
            self.nominal_trajectory["state"] = self.initialization["state"]
            self.nominal_trajectory["control"] = self.initialization["control"]
            self.num_tsteps = self.nominal_trajectory["state"].shape[0]
        else: #generate straight line as guess (Forward Euler Integration doesn't work well here // RRT would be better than straight line)
            self.num_tsteps = self.Rover.num_tsteps
            print('start: ', self.initial_state,'end: ',self.terminal_state)
            self.nominal_trajectory["state"] = np.linspace(self.initial_state,self.terminal_state,self.num_tsteps) #straight line guess
            self.nominal_trajectory["control"] = 0.01*np.ones([int(self.num_tsteps-1),self.num_control]) #non-zero constant thrust guess
        # Solution Storage:
        self.sol= {}
        self.sol["state"] = []
        self.sol["control"] = []
        self.sol["dt"] = self.Rover.dt
        # SCP Parameters:
        self.trust = self.Rover.delta
        self.scp_param = {}
        self.scp_param["beta_fail"] = 0.8 #trust region scaling factor (reduction)
        self.scp_param["beta_success"] = 1 #trust region scaling factor (expansion)
        self.scp_param["rho_0"] = 0.5 #accuracy threshold
        self.scp_param["rho_1"] = 1e-1 #accuracy threshold
        self.scp_param["step_tolerance"] = 0.001 #convergence tolerance
        self.scp_param["iter_max"] = 20 #maximum SCP iterations
    
    def scp(self):
        max_state_dev = np.inf
        it = 0
        xprev = self.nominal_trajectory["state"]
        uprev = self.nominal_trajectory["control"]
        while max_state_dev >= self.scp_param["step_tolerance"] and it < self.scp_param["iter_max"]:
            result = self.convex_program(xprev, uprev)
            if result == 'DECREASE_TRUST': #in case of solver failure (no solution found): if failure happens because model inaccurate -> shrink / if happens because no feasible move exists near nominal -> expand
                self.trust['x'] *= self.scp_param["beta_fail"]
                self.trust['u'] *= self.scp_param["beta_fail"]
                print("Solve failed, shrink trust to:", self.trust['x'], self.trust['u'])
                it += 1
                continue
            x_sol, u_sol = result[0], result[1]
            delta_x = np.linalg.norm(x_sol - xprev, axis=1) #step deviation in state within trust region
            delta_u = np.linalg.norm(u_sol - uprev, axis=1) #step deviation in control within trust region
            max_step_dev_x = np.max(delta_x) #maximum deviation in state across all time steps
            max_step_dev_u = np.max(delta_u) #maximum deviation in control across all time steps
            if (max_step_dev_x > self.trust['x']) or (max_step_dev_u > self.trust['u']):
                print("Reject solution, outside trust region: shrink trust") #step too far from safe linearization point -> need to shrink region
                self.trust['x'] *= self.scp_param["beta_fail"]
                self.trust['u'] *= self.scp_param["beta_fail"]
                it += 1
                continue
            rho = self.compute_rho(x_sol, u_sol, xprev, uprev) #model accuracy ratio
            if rho > self.scp_param["rho_1"]:
                print(f"Reject solution, model inaccurate: (rho={rho:.4g}), shrink trust")
                self.trust['x'] *= self.scp_param["beta_fail"]
                self.trust['u'] *= self.scp_param["beta_fail"]
                it += 1
                continue
            max_state_dev = np.max(np.linalg.norm(x_sol - xprev, axis=1)) #max state deviation of solution from previous trajectory
            print(f"Accept solution, model accurate with (rho={rho:.4g})  iter: {it+1}  max_state_dev: {max_state_dev:.6g}") 
            if rho < self.scp_param["rho_0"]:
                self.trust['x'] *= self.scp_param["beta_success"] #expand trust if model very accurate
                self.trust['u'] *= self.scp_param["beta_success"]
            xprev = x_sol
            uprev = u_sol
            it += 1
        self.sol["state"] = xprev
        self.sol["control"] = uprev
        print("SCP converged. Access solution using self.sol.")
        return 1

    def convex_program(self, xprev, uprev):
        X = cp.Variable((self.num_tsteps,self.num_states)) #state variable definition
        U = cp.Variable((self.num_tsteps-1,self.num_control)) #control variable definition
        S = cp.Variable(self.num_tsteps, nonneg=True) #slack >= 0
        # State Constraints:
        constraint = []
        constraint.append(X[0,:]==self.initial_state) #initial condition
        constraint.append(X[self.num_tsteps-1,:]==self.terminal_state) #terminal condition
        for t in range(self.num_tsteps):
            constraint.append(X[t,:]>=self.state_min) #state lower bound
            constraint.append(X[t,:]<=self.state_max) #state upper bound
        # Control Constraints:
        for t in range(self.num_tsteps-1):
            constraint.append(U[t,:]<=self.u_max) #control upper bound
            constraint.append(U[t,:]>=self.u_min) #control lower bound
        # Dynamics:
        for t in range(self.num_tsteps-1):
            x_dummy = onp.array(xprev[t,:],dtype=np.float32)
            u_dummy = onp.array(uprev[t,:],dtype=np.float32)
            Ad = self.A(x_dummy,u_dummy)
            Bd = self.B(x_dummy,u_dummy)
            cd = self.c(x_dummy,u_dummy)
            constraint.append(X[t+1,:]== (Ad @ X[t,:] + Bd @ U[t,:] + cd))
        # Collision Constraints:
        for t in range(self.num_tsteps):
            x_nom = xprev[t,:2] #position from previous nominal trajectory (px,py)
            d_nom = self.sdf_value(x_nom) #computed sdf value at nominal point (scalar) -> interpolated
            n_nom = self.sdf_gradient(x_nom) #computed gradient from sdf array (2D unit vector) -> interpolated
            constraint.append(d_nom + n_nom @ (X[t, :2] - x_nom) + S[t] >= self.dmin) #linearized collision constraints
        # Trust Region (OSQP-friendly format):
        for t in range(self.num_tsteps):
            constraint.append(cp.abs(X[t,:2] - xprev[t,:2]) <= self.trust["x"]) 
        for t in range(self.num_tsteps - 1):
            constraint.append(cp.abs(U[t,:] - uprev[t,:]) <= self.trust["u"])
        # Objective Function:
        cost = 0
        cost += cp.sum_squares(U) #minimize control effort (quadratic form since cvxpy needs a QP for OSQP solver)
        w_slack = 1e4 #weight on slack variable to heavily penalize collision constraint violation (soft constraint)
        cost += w_slack * cp.sum(S) #minimize control effort + slack violation (convex cost function)
        # Solve Problem:
        problem = cp.Problem(cp.Minimize(cost), constraint)
        problem.solve(
                    solver=cp.OSQP,
                    warm_start=True,
                    verbose=True,
                    max_iter=50000,
                    eps_abs=1e-3,
                    eps_rel=1e-3,
                    polish=True,
                ) #convex optimization (the solver will try up to 50000 iterations to find a solution)
        print("max slack:", np.max(S.value))
        if problem.status in ["optimal", "optimal_inaccurate"]:
            return [X.value, U.value] #numerical solution of the optimization variables - arrays of shape (num_tsteps, num_states or num_control)
        if problem.status == "user_limit": #OSQP reached iteration limit without convergence, but may still have a valid solution (check slack variable for constraint violation)
            print("Warning: user_limit — accepting solution")
            return [X.value, U.value]
        print("Solver status:", problem.status)
        return "DECREASE_TRUST" #in case of solver failure, decrease trust region (search space) to try to find a more local solution in next iteration
        
    def compute_rho(self, x_sol, u_sol, xprev, uprev):
        # Compute model accuracy ratio ρ = (actual reduction) / (predicted reduction):
        num, den = 0.0, 0.0
        for t in range(self.num_tsteps - 1):
            #Linearized discrete prediction around (xprev, uprev):
            Ad = self.A(xprev[t,:], uprev[t,:])
            Bd = self.B(xprev[t,:], uprev[t,:])
            cd = self.c(xprev[t,:], uprev[t,:])
            x_lin_next = Ad @ x_sol[t,:] + Bd @ u_sol[t,:] + cd
            #"True" discrete step using Euler on continuous dynamics:
            f = self.f_continuous(x_sol[t,:], u_sol[t,:]) #xdot
            x_true_next = x_sol[t,:] + self.dt * f #Euler step
            #Compare true discrete step vs linearized discrete step along the new solution:
            num += np.linalg.norm(x_true_next - x_lin_next)
            den += np.linalg.norm(x_lin_next)
        return num / (den + 1e-9)
