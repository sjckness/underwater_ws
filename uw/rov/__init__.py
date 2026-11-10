"""The simulated BlueROV2 Heavy (M8a): Fossen dynamics, frame conversions, the ArduPilot JSON link.

Nothing in here imports Isaac, omni, pxr or rclpy at module level. ``isaac_vehicle`` imports Isaac lazily
inside its functions, so every module of this package imports on a machine with neither Isaac nor ROS.

Layout (docs/ros_hil.md section 6):
* ``frames``    - the only place that converts between tank/ENU/FLU and ArduPilot NED/FRD.
* ``dynamics``  - 6-DOF Fossen rigid body, thrusters, soft tank bound. Pure numpy.
"""
