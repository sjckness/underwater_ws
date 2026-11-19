"""Acoustic SLAM from the forward-looking sonar: front end, registration, pose graph.

Pure numpy, like ``uw.rov``. Nothing in here imports Isaac, omni, pxr, rclpy, scipy or cv2, so every
module imports and every test runs on a bare host with numpy alone — which is also what lets the whole
chain be developed offline against a recording while Isaac has the GPU.

Layout (docs/slam.md):
* ``frontend``  - one ping -> landmarks on the bottom, with per-point covariance.
* ``icp``       - registration of two landmark clouds.
* ``graph``     - an SE(2) pose graph with loop closures.

The estimator reads only what a real vehicle would: the sonar image, its latched metadata, the DVL's
altitude and the dead-reckoned odometry. The scene, the true pose and anything under ``/gt`` belong to
the evaluator and never to this package (``docs/ros_hil.md`` 2.5).
"""
