#!/bin/bash
set -e

# Setup development environment
source /opt/ros/foxy/setup.bash
source /home/autodrive_devkit/install/setup.bash

# AutoDRIVE Devkit Workspace
cd /home/autodrive_devkit

# Launch AutoDRIVE Devkit with GUI
# ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py

# Launch AutoDRIVE Devkit Headless
# ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py