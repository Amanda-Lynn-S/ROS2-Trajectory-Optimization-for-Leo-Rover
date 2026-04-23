#!/usr/bin/env python3
"""
Rover path planning simulation using Sequential Convex Programming (SCP).

ROS2 CHANGES vs original rover_pp_simulation.py:
rover_pp_simulation.py -> rover_pp_node.py:

  Subscriptions (replaces direct Rover() file-loading):
    /map_meta -> std_msgs/String (JSON metadata published by map_publisher_node)
    /sdf_grid -> std_msgs/Float32MultiArray (SDF array published by map_publisher_node)

  Publications:
    /scp_trajectory_states -> std_msgs/Float32MultiArray (full state trajectory, shape N×5)
    /scp_trajectory_controls -> std_msgs/Float32MultiArray (full control trajectory, shape (N-1)×2)
    /cmd_vel -> geometry_msgs/Twist (time-sequenced velocity commands)
        > linear.x = v (state index 3)
        > angular.z = omega (state index 4)

  Flow:
    1. Node waits to receive both /map_meta and /sdf_grid.
    2. Once both arrive, Rover and SCP are instantiated and scp() is called.
    3. Full trajectory is published once on /scp_trajectory_states and /scp_trajectory_controls.
    4. A ROS2 timer fires every 'dt' seconds and streams /cmd_vel commands step-by-step.

  -> Matplotlib visualisation is preserved; it runs in a background thread so it does not
  block the ROS2 executor.

  NOTE: /cmd_vel publishes (v, omega) from the optimised state sequence to enable rover actuation. 
"""

import threading
import json
import numpy as np
import matplotlib
matplotlib.use("Agg") #non-interactive backend safe for background thread
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist

from .SCP import SCP
from .Rover import Rover 


class RoverPPNode(Node):
    def __init__(self):
        super().__init__('rover_pp_node')
        self.meta_sub = self.create_subscription(String, '/map_meta', self._meta_callback, 10) #subscriber for map metadata (JSON string)
        self.sdf_sub = self.create_subscription(Float32MultiArray, '/sdf_grid', self._sdf_callback, 10) #subscriber for sdf grid (2D float array)
        self.states_pub = self.create_publisher(Float32MultiArray, '/scp_trajectory_states', 10) #publisher for full state trajectory (N×5)
        self.ctrl_pub = self.create_publisher(Float32MultiArray, '/scp_trajectory_controls', 10) #publisher for full control trajectory (N-1)×2)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #publisher for velocity commands
        # Internal state - set once both topics have been received:
        self._meta = None
        self._sdf = None
        self._scp_done = False
        # Trajectory playback:
        self._xscp = None
        self._dt = None
        self._traj_index = 0
        self._cmd_timer = None
        self.get_logger().info("rover_pp_node ready — waiting for /map_meta and /sdf_grid.")

    # --------- Subscriber callbacks ---------
    def _meta_callback(self, msg: String):
        if self._scp_done:
            return
        self._meta = json.loads(msg.data)
        self.get_logger().info("Received /map_meta.")
        self._try_run_scp()

    def _sdf_callback(self, msg: Float32MultiArray):
        if self._scp_done:
            return
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self._sdf = np.array(msg.data, dtype=np.float32).reshape(rows, cols)
        self.get_logger().info(f"Received /sdf_grid ({rows}×{cols}).")
        self._try_run_scp()

    # --------- SCP execution logic ---------
    def _try_run_scp(self):
        """Start planning once both map topics have been received."""
        if self._meta is not None and self._sdf is not None and not self._scp_done:
            self._scp_done = True
            threading.Thread(target=self._run_scp, daemon=True).start() #run SCP in a separate thread so the ROS2 executor is not blocked

    # --------- Planning -----------
    def _run_scp(self):
        from time import time
        # Instantiate Rover and SCP with received map data:
        leo_rover = Rover(meta=self._meta, sdf_array=self._sdf)
        self._dt  = leo_rover.dt
        initialization = {"valid": False}
        scp_planner = SCP(Rover=leo_rover, initialization=initialization)
        # Run SCP and extract trajectory:
        t0 = time()
        scp_planner.scp()
        self.get_logger().info(f"SCP completed in {time() - t0:.2f} s")
        xscp = np.array(scp_planner.sol["state"]) #shape (N,5)
        uscp = np.array(scp_planner.sol["control"]) #shape (N-1,2)
        self._xscp = xscp
        # Publish full trajectory:
        self._publish_trajectory(xscp, uscp)
        # Start timed /cmd_vel streaming:
        self._traj_index = 0
        self._cmd_timer  = self.create_timer(self._dt, self._publish_cmd_vel)
        # Visualisation in background thread:
        threading.Thread(target=self._visualise, args=(xscp,), daemon=True).start()

    # -------- Trajectory publisher helpers ----------
    def _make_float32_msg(self, arr: np.ndarray, dim_labels) -> Float32MultiArray:
        msg = Float32MultiArray()
        stride = arr.size
        msg.layout.dim = []
        for i, (label, size) in enumerate(zip(dim_labels, arr.shape)):
            stride = int(np.prod(arr.shape[i:]))
            msg.layout.dim.append(
                MultiArrayDimension(label=label, size=size, stride=stride)
            )
        msg.layout.data_offset = 0
        msg.data = arr.astype(np.float32).flatten().tolist()
        return msg

    def _publish_trajectory(self, xscp: np.ndarray, uscp: np.ndarray):
        self.states_pub.publish(self._make_float32_msg(xscp, ["timesteps", "states"]))
        self.ctrl_pub.publish(self._make_float32_msg(uscp, ["timesteps", "controls"]))
        self.get_logger().info(f"Published trajectory: states {xscp.shape}, controls {uscp.shape}.")

    def _publish_cmd_vel(self):
        if self._xscp is None or self._traj_index >= len(self._xscp):
            if self._cmd_timer is not None:
                self._cmd_timer.cancel()
                self.get_logger().info("Trajectory playback complete.")
            return
        state = self._xscp[self._traj_index]
        twist = Twist()
        twist.linear.x = float(state[3]) #v (m/s)
        twist.angular.z = float(state[4]) #omega (rad/s)
        self.cmd_pub.publish(twist) #/cmd_vel timer callback -> streams (v, omega) at each dt step
        self._traj_index += 1

    # -------- Visualisation (3-D plot of px, py, theta) ---------
    def _visualise(self, xscp: np.ndarray):
        fig = plt.figure()
        ax  = fig.add_subplot(111, projection="3d")
        ax.plot(xscp[:, 0], xscp[:, 1], xscp[:, 2],
                label="SCP Trajectory", linewidth=3)
        ax.set_title("SCP Optimised Trajectory")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("θ [rad]")
        ax.legend()
        plt.savefig("scp_trajectory.png", dpi=150)
        self.get_logger().info("Trajectory plot saved to scp_trajectory.png")
        plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = RoverPPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
