# Use ROS Noetic full desktop image as base
FROM osrf/ros:noetic-desktop-full

# Set the environment variable to avoid interaction during installation
ARG DEBIAN_FRONTEND=noninteractive

# Label for the Docker image
LABEL maintainer="Karlym Nam karlym5@gmail.com"

# Set to root user to install dependencies
USER root

# Update and install required dependencies
RUN apt-get update \
    && apt-get upgrade --assume-yes --no-install-recommends \
    && apt-get install --assume-yes --no-install-recommends \
    python3-pip \
    wget \
    git \
    python3-rosdep \
    python3-catkin-tools \
    build-essential \
    cmake \
    libopencv-dev \
    ros-noetic-catkin


# Install Python dependencies (optional, based on your requirements.txt)
COPY requirements.txt ./
RUN pip3 install --no-cache-dir -r requirements.txt

# Set up the user that will work in the container
ARG USERNAME=vscode
ARG USER_UID=1001
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -s /usr/bin/bash --groups video,audio,dialout -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


# Create the catkin workspace directory (will be mounted by devcontainer.json)
RUN mkdir -p /home/$USERNAME/stickynav-dev/catkin_ws/src/StickyNAV

# Set working directory to the catkin workspace
WORKDIR /home/$USERNAME/stickynav-dev/catkin_ws/src

# Clone the repositories you need inside the catkin_ws/src
RUN git clone https://github.com/KXRLYM/fridgeback_base_simulator.git \
    && git clone https://github.com/KXRLYM/fridgeback_arm.git \
    && git clone https://github.com/KXRLYM/fridgeback_base.git \
    && git clone https://github.com/KXRLYM/fridgeback_config.git

RUN chown -R vscode:vscode /home/$USERNAME/stickynav-dev/catkin_ws


# Set up NVIDIA GPU support and other ROS-related environment variables
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Ensure ROS environment is sourced when user starts a new shell
RUN echo '. /opt/ros/noetic/setup.bash' >> ~/.bashrc

WORKDIR /home/$USERNAME/stickynav-dev/catkin_ws
RUN exec bash


RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && rosdep update

RUN rosdep install --from-paths src --ignore-src -r -y 
RUN catkin init && catkin config --extend /opt/ros/noetic && catkin build
RUN apt-get install --assume-yes --no-install-recommends \
    ros-noetic-octomap-ros \
    ros-noetic-moveit-visual-tools \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-realsense2-description \
    ros-noetic-teb-local-planner \
    ros-noetic-octomap-rviz-plugins

# Clean up APT lists to reduce image size
RUN rm -rf /var/lib/apt/lists/*

# Switch back to vscode for any further actions
USER $USER_UID

# This will be the default command to start a ROS workspace shell when the container starts
CMD ["/bin/bash"]
