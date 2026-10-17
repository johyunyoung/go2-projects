# go2-mpc-dev: go2-deploy-user + ROS 2 Humble, for developing the SRB-MPC controller.
#
# ROS is only a viewer here (RViz2, PlotJuggler, rosbag2). The controller itself
# links unitree_sdk2 only, so the same binary runs on the robot's Jetson without ROS.
FROM go2-deploy-user

ARG UID=1000
ARG GID=1000

USER root
ENV DEBIAN_FRONTEND=noninteractive

# ---- ROS 2 Humble apt repository ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release software-properties-common \
    && add-apt-repository -y universe \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 viewer stack + build/test tools ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rviz2 \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-tf2-ros \
        ros-humble-plotjuggler-ros \
        ros-humble-rosbag2-storage-mcap \
        python3-colcon-common-extensions \
        libgtest-dev gdb \
    && rm -rf /var/lib/apt/lists/*

USER ${UID}:${GID}
