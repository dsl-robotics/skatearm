"""skate_ros2 — ROS 2 bridge + MuJoCo sim endpoint for the R.Botic Skate.

Layers:
* :mod:`skate_ros2.names`        canonical 26-DoF ordering (no deps)
* :mod:`skate_ros2.protocol`     UDP wire protocol client (numpy only)
* :mod:`skate_ros2.sim_endpoint` MuJoCo twin behind the real UDP contract
* :mod:`skate_ros2.driver_node`  rclpy bridge node (needs ROS 2)
"""

__version__ = "0.1.0"
