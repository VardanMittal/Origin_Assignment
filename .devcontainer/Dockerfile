FROM osrf/ros:humble-desktop

# -----------------------------
# User setup
# -----------------------------
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

RUN sudo usermod --append --groups video $USERNAME

# -----------------------------
# System updates & tools
# -----------------------------
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install -y git mesa-utils

# -----------------------------
# ROS environment
# -----------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# -----------------------------
# Gazebo Classic (11)
# -----------------------------
RUN sudo apt install -y gazebo

# -----------------------------
# ROS ↔ Gazebo bridge
# -----------------------------
RUN sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

# -----------------------------
# TurtleBot3
# -----------------------------
RUN sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-teleop

# -----------------------------
# TurtleBot3 environment
# -----------------------------
ENV TURTLEBOT3_MODEL=burger

RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# -----------------------------
# rosdep (optional but clean)
# -----------------------------
RUN rosdep update

# -----------------------------
# Working directory
# -----------------------------
WORKDIR /workspace
