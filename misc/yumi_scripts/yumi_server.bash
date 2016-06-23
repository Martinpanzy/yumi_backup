#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# PURPOSE: Make it easier to run YuMi scripts - Run server to send trajectories to YuMi

if [ -z "$1" ]; then
	source ~/yumi_ws/devel/setup.bash # source the catkin workspace
	roslaunch yumi_support robot_interface.launch # run YuMi server to send files to real YuMi robot using the default IP address
else
	if [ "$1" == "lead_through" ]; then
		source ~/yumi_ws/devel/setup.bash # source the catkin workspace
		roslaunch yumi_support robot_interface.launch lead_through:=true # run YuMi server to send files to real YuMi robot at the specified IP address
	elif [ "$1" == "two_grippers"]; then
		source ~/yumi_ws/devel/setup.bash # source the catkin workspace
		roslaunch yumi_support robot_interface.launch two_grippers:=true # run YuMi server to send files to real YuMi robot at the specified IP address
	else
		# NOTE: Need to check if IP was entered or provide error that input is not acceptable | DATE: 2016-06-23
		source ~/yumi_ws/devel/setup.bash # source the catkin workspace
		roslaunch yumi_support robot_interface.launch robot_ip:=$1 # run YuMi server to send files to real YuMi robot at the specified IP address
	fi
fi


