#!/bin/bash
cd /home/whi/catkin_workspace/
source /opt/ros/melodic/setup.bash
source /home/whi/catkin_workspace/devel/setup.bash
echo "launching application, please wait..."
roslaunch whi_motion_teleop whi_motion_teleop.launch
