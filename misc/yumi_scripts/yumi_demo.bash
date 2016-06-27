#!/bin/bash
# PROGRAMMER: Frederick Wachter
# DATE CREATED: 2016-05-20
# PURPOSE: Make it easier to run YuMi scripts - Run YuMi demo in RViz

if [[ -n "$1" ]] && [[ "$1" == "two_grippers" ]]; then
	source ~/yumi_ws/devel/setup.bash # source the catkin workspace
	roslaunch yumi_moveit_config demo.launch two_grippers:=true # launch YuMi in RViz with two grippers
else
	source ~/yumi_ws/devel/setup.bash # source the catkin workspace
	roslaunch yumi_moveit_config demo.launch # launch YuMi in RViz
fi


