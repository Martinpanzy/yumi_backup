/*
  Move group interface for basic pick-place actions.
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp generation and visualization
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yumi_basic_grasp");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "left_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // robot-specific data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
/*  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("yumi_body");
  visual_tools.deleteAllMarkers();
*/

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("yumi_body"));

/*
  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
*/
  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools_.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Load grasp data specific to our robot
  ros::NodeHandle nh("~");
  if (!grasp_data_.loadRobotGraspData(nh, "left_gripper"))
    ros::shutdown();

  // Load grasp generator
  simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );

  geometry_msgs::PoseStamped curr_pose = move_group.getCurrentPose();

  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0.4;
  object_pose.position.y = -0.2;
  object_pose.position.z = 0.0;

  // Orientation
  double angle = 0; //M_PI / 1.5;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  object_pose.orientation.x = quat.x();
  object_pose.orientation.y = quat.y();
  object_pose.orientation.z = quat.z();
  object_pose.orientation.w = quat.w();

  // Visualize object pose as a block
  visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, 0.04);

  // Actually generate grasps
  std::vector<moveit_msgs::Grasp> possible_grasps;
  simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);

  ROS_INFO_NAMED("tutorial", "Reference frame: ", possible_grasps);

  // Visualize
  //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
/*
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation = curr_pose.pose.orientation;
  start_pose2.position = curr_pose.pose.position;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up

  target_pose3.position.y -= 0.1;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z = 0.1;
  waypoints.push_back(target_pose3);  // down and right

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("next step");
/*
  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z = 1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the collision object message
  ros::Duration(1.0).sleep();

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(target_pose1);

  success = move_group.plan(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
*/
  /* Sleep to allow MoveGroup to recieve and process the attached collision object message */
//  ros::Duration(1.0).sleep();

  // Now, let's detach the collision object from the robot
//  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//  move_group.detachObject(collision_object.id);

  // Show text in Rviz of status
//  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();

  /* Sleep to allow MoveGroup to recieve and process the detach collision object message */
//  ros::Duration(1.0).sleep();
/*
  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
*/
  /* Sleep to give Rviz time to show the object is no longer there.*/
//  ros::Duration(1.0).sleep();
/*
  // Dual-arm pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms.
  static const std::string PLANNING_GROUP2 = "arms";
  moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);

  // Define two separate pose goals, one for each end-effector. Note that
  // we are reusing the goal for the right arm above
  two_arms_move_group.setPoseTarget(target_pose1, "r_wrist_roll_link"); //yumi_link_7_r_joint

  geometry_msgs::Pose target_pose4;
  target_pose4.orientation.w = 1.0;
  target_pose4.position.x = 0.7;
  target_pose4.position.y = 0.15;
  target_pose4.position.z = 1.0;

  two_arms_move_group.setPoseTarget(target_pose4, "l_wrist_roll_link"); //yumi_link_7_r_joint

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroupInterface::Plan two_arms_plan;

  success = two_arms_move_group.plan(two_arms_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (dual arm plan) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal1");
  visual_tools.publishAxisLabeled(target_pose4, "goal2");
  visual_tools.publishText(text_pose, "Two Arm Goal", rvt::WHITE, rvt::XLARGE);
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
  visual_tools.publishTrajectoryLine(two_arms_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // END_TUTORIAL
*/
  ros::shutdown();
  return 0;
}
