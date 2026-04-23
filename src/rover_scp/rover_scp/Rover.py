""" 
This module defines the Rover class that includes constraints parameters and will be used in the SCP:
1. Dynamics
2. Control limits
3. Boundary conditions
4. State limits
5. Collision avoidance
6. Trust region

ROS2 CHANGES vs original:
  - Constructor now accepts 'meta' (dict) and 'sdf_array' (np.ndarray) directly as parameters
    instead of loading from disk. File I/O (os, json, path scanning) is removed entirely.
  - The rover_pp_node subscriber receives these from ROS2 topics and passes them here.
"""

import numpy as np
from .rover_model import f_continuous, linearize_and_discretize, params_dict
from scipy.interpolate import RegularGridInterpolator

class Rover():
    def __init__(self, meta: dict, sdf_array: np.ndarray):
        self.num_states = 5
        self.num_control = 2
        pix_to_m = 0.01 #1 pixel = pix_to_m meters (1 grid cell = 10 pixels = 0.1 meter = half rover size in width)
        # Unpack map metadata (dimensions) from passed dict:
        width = meta["width"]
        height = meta["height"]
        self.cell_size = meta["cell_size"]
        start = np.array(meta["start"], dtype=float) #rover initial grid coordinates
        end = np.array(meta["end"], dtype=float) #rover goal grid coordinates
        scale = self.cell_size * pix_to_m  #meters per grid cell (10 pixels * 0.01 m/pixel = 0.1 m)
        # Constraints Parameters:
        # Boundary conditions (set based on map data and assuming linear path from start to goal for initial heading):
        self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) #initial state
        self.xf = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) #final state
        self.x0[:2] = start * scale #convert grid coords to world coords where 1 pixel = pix_to_m meters
        self.xf[:2] = end * scale
        dx = self.xf[0] - self.x0[0] 
        dy = self.xf[1] - self.x0[1]
        th_goal = np.arctan2(dy, dx) #desired heading to go straight from start -> goal
        self.x0[2] = th_goal 
        self.xf[2] = th_goal
        # State limits (x and y positions constrained by map size - origin at top left corner; v and w limits based on Leo rover specs):
        self.state_min = [0.0, 0.0, -np.inf, -0.4, -1.0] #rover heading theoretically unbounded
        self.state_max = [width * self.cell_size * pix_to_m, height * self.cell_size * pix_to_m, np.inf, 0.4, 1.0]
        # Control limits:
        self.u_max = np.array([5.6, 5.6]) #max torque inputs (N*m) from Leo rover specs
        self.u_min = np.negative(self.u_max) #min torque inputs
        # Collision avoidance:
        self.sdf  = sdf_array * scale #convert SDF values from cells -> meters (self.sdf.min()=negative=obstacle present)
        self.dmin = 0.10 #clearance in meters (e.g., rover radius)
        # Dynamics:
        distance = np.linalg.norm(self.xf[:2] - self.x0[:2]) #euclidean distance between start and end positions
        vmax = self.state_max[3] #0.4 m/s max speed from Leo rover specs
        Tmin = distance/vmax #minimum time to reach goal at max speed (for feasibility check and to set num_tsteps)
        self.dt = 0.1 #time step size in seconds (can be adjusted as needed, but should be small enough for SCP convergence and large enough for computational efficiency)
        Nmin = int(np.ceil(Tmin / self.dt)) + 1 #minimum number of time steps needed to reach goal at max speed 
        self.num_tsteps = Nmin + 5 #add some buffer time steps for feasibility (can be adjusted as needed)
        self.f_continuous = lambda x, u: f_continuous(x, u, params_dict())
        self.A = lambda x, u, dt=self.dt: linearize_and_discretize(x, u, dt, params_dict())[0]
        self.B = lambda x, u, dt=self.dt: linearize_and_discretize(x, u, dt, params_dict())[1]
        self.c = lambda x, u, dt=self.dt: linearize_and_discretize(x, u, dt, params_dict())[2]
        # --- SDF Interpolation Setup ---
        y = np.arange(self.sdf.shape[0]) * scale #world y-coordinates of each SDF row
        x = np.arange(self.sdf.shape[1]) * scale #world x-coordinates of each SDF column
        self.sdf_interp = RegularGridInterpolator((y, x), self.sdf, method="linear", bounds_error=False, fill_value=np.min(self.sdf)) #sdf interpolator object
        self.grady, self.gradx = np.gradient(self.sdf, scale, scale) #sdf gradient arrays
        self.gradx_interp = RegularGridInterpolator((y, x), self.gradx, method="linear", bounds_error=False, fill_value=0) #gradient interpolator object
        self.grady_interp = RegularGridInterpolator((y, x), self.grady, method="linear", bounds_error=False, fill_value=0) #gradient interpolator object
        # Trust region:
        self.delta = {"x":10,"u":10} #initial trust region (to tweak if needed, considering SCP performance)
# --- SDF Interpolation Setup ---
    def sdf_value(self, pos):
        return float(self.sdf_interp([pos[::-1]])[0])
    def sdf_gradient(self, pos):
        gx = float(self.gradx_interp([pos[::-1]])[0])
        gy = float(self.grady_interp([pos[::-1]])[0])
        grad = np.array([gx, gy])
        return grad
      
