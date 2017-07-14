#include <ros/ros.h>
// MoveIt blocks
#include <block_pick_place.h>


int main(int argc, char **argv)
{
  ROS_INFO_STREAM_NAMED("temp","Starting Yumi Block Pick Place");

  ros::init (argc, argv, "yumi_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the pick place node
  moveit_simple_grasps::MoveItBlocks();

  ros::shutdown();

  return 0;
}
