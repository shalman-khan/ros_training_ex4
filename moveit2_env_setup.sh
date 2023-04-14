#!/bin/bash
# This script sets up a workspace for MoveIt2 tutorials on ROS Humble,
# installs necessary dependencies, and builds the workspace.
# Change directory to home directory
cd $HOME

# Source ROS Humble setup.bash file

source /opt/ros/humble/setup.bash

# Install python3-rosdep package
sudo apt install python3-rosdep -y

# Initialize rosdep
sudo rosdep init

# Update rosdep
rosdep update

# Update packages
sudo apt update

# Upgrade packages
sudo apt dist-upgrade -y

# Install python3-colcon-common-extensions package
sudo apt install python3-colcon-common-extensions -y

# Add default mixin to colcon
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml

# Update default mixin
colcon mixin update default

# Install python3-vcstool package
sudo apt install python3-vcstool -y

# Create a new directory for MoveIt2 tutorials workspace and change directory to src folder
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src

# Clone the MoveIt2 tutorials repository
git clone -b humble-training https://github.com/shalman-khan/moveit2_tutorials.git

# Import the repository configuration
vcs import < moveit2_tutorials/moveit2_tutorials.repos

# Update packages and install necessary dependencies
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Change directory to the workspace folder and build it
cd ~/ws_moveit2
colcon build --mixin release

# Source the setup.bash file for the workspace
source ~/ws_moveit2/install/setup.bash

# Add the setup.bash file sourcing command to .bashrc file
echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc

# Install rmw_cyclonedds_cpp package
sudo apt install ros-humble-rmw-cyclonedds-cpp -y

# Set environment variable for RMW implementation to rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add RMW implementation export command to .bashrc file
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
