#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# PURPOSE: Make it easier to run YuMi scripts - Run server to send trajectories to YuMi

if [ -z "$1" ]; then
	source ~/yumi_ws/devel/setup.bash # source the catkin workspace
	roslaunch yumi_support robot_interface.launch # run YuMi server to send files to real YuMi robot using the default IP address
else
	source ~/yumi_ws/devel/setup.bash # source the catkin workspace
	roslaunch yumi_support robot_interface.launch robot_ip:=$1 # run YuMi server to send files to real YuMi robot at the specified IP address
fi


