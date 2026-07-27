#!/bin/bash
# Raspberry Pi 5 Automated Dependencies & Environment Setup Script for PiSim Platform

echo "=========================================================="
echo "    PiSim Platform - Raspberry Pi 5 Automated Setup       "
echo "=========================================================="

# Update package lists and install system OpenCV / NumPy libraries
sudo apt update && sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    libopencv-dev

# Install Python requirements
pip3 install --break-system-packages -r requirements.txt || pip3 install -r requirements.txt

echo ""
echo "=========================================================="
echo " [+] Setup completed successfully!"
echo " [+] To run Standalone UDP Mode : python3 pisim_core.py"
echo " [+] To run Native ROS 2 Mode   : python3 pisim_ros2_node.py"
echo "=========================================================="
