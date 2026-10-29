# The "stack container": the ROS 2 side of the HIL link, with no Isaac in it at all.
#
# It runs on thor and orin (arm64) and on linus (amd64); ros:jazzy-ros-base is multi-arch, so this one file
# builds natively on each. It carries no project code: the workspace is bind-mounted at the same path as on the
# host (docker/compose.stack.yaml), so ros/sonar_relay/run.sh, ros/link_monitor/run.sh and ros/pose_cmd/run.sh
# work inside it unchanged, exactly as they do on linus today.
#
# Build (natively, on the machine that will run it — much faster than cross-building):
#     docker build -t uw_stack:jazzy -f docker/stack/Dockerfile docker/stack
# Cross-build from linus for a Jetson, if ever needed (needs the arm64 binfmt handler, which this box does not
# have today: `docker run --privileged --rm tonistiigi/binfmt --install arm64` installs it):
#     docker buildx build --platform linux/arm64 -t uw_stack:jazzy -f docker/stack/Dockerfile --load docker/stack
FROM ros:jazzy-ros-base

# marine_acoustic_msgs is what ros/sonar_relay publishes and what the detector consumes; it is the one message
# package the sim's bundled rclpy does NOT have, which is why the relay is a separate node on the sim host
# (docs/topic_map.md §8 item 2). rosbag2 is the detector and evaluation bags; iproute2 and ethtool are for the
# link checks of docs/bridge_setup.md §2 check 3, which have to run where the NIC is.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-marine-acoustic-msgs \
      ros-jazzy-rosbag2 \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-tf2-ros \
      ros-jazzy-vision-msgs \
      ros-jazzy-rosidl-runtime-py \
      iproute2 ethtool iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# No ros-jazzy-mavros here yet: it belongs to M8b, it pulls GeographicLib's datasets, and the arm64 binary
# availability is still an open question (docs/ros_hil.md §7 item 1). Add it in that milestone, not before.

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    PYTHONDONTWRITEBYTECODE=1
# the run.sh scripts source /opt/ros/jazzy themselves, so the entrypoint stays a plain shell
ENTRYPOINT ["/bin/bash"]
