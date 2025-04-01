#!/bin/bash
set -e

# AutoDRIVE Simulator Executable
cd /home/autodrive_simulator

# Launch AutoDRIVE Simulator with GUI
# ./AutoDRIVE\ Simulator.x86_64

# Launch AutoDRIVE Simulator Headless
# xvfb-run ./AutoDRIVE\ Simulator.x86_64 -ip 127.0.0.1 -port 4567

# Launch AutoDRIVE Simulator without Graphics
# ./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567
