#!/bin/bash

# Note it is assumed that this file is at the root dir of the package, that is its next to package.xml file

# Source ROS env
source /opt/ros/humble/setup.bash

# Build package
colcon build

# Issues with Python 
# https://stackoverflow.com/questions/36296134/attributeerror-install-layout-when-attempting-to-install-a-package-in-a-virtual
export SETUPTOOLS_USE_DISTUTILS=stdlib

# Setup .deb file rules
rosdep init
rosdep update
bloom-generate rosdebian

# Build deb file
fakeroot debian/rules binary
