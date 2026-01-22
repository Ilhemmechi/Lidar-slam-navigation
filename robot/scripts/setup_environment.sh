#!/bin/bash

# Setup environment script

echo "Setting up environment..."

# Source ROS
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip install -r requirements.txt

echo "Environment setup complete."