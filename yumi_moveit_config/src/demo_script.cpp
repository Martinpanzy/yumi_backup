#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

using namespace ros;
namespace planningInterface = moveit::planning_interface;

int main(int argc,char **argv) {
	init(argc,argv,"demo_script");

	// Start a background "spinner" for node to process ROS messages
	AsyncSpinner spinner(1);
	spinner.start();

	planningInterface::MoveGroup right_arm("right_arm");
	right_arm.setNamedTarget("calc");
	right_arm.move();

	right_arm.setNamedTarget("home");
	right_arm.move();
	// spin();
}


