// ROS
#include <ros/ros.h>

// MoveIt!
// https://github.com/ros-planning/moveit/issues/37
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/macros/deprecation.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing graspsp
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_simple_grasps/custom_environment2.h>

namespace moveit_simple_grasps
{

static const double BLOCK_SIZE = 0.04;

struct MetaBlock
{
  std::string name;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
};

class MoveItBlocks
{
public:

  // grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // which arm are we using
  std::string ee_group_name_;
  std::string planning_group_name_;

  ros::NodeHandle nh_;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group;
  const robot_state::JointModelGroup *eef_joint_model_group;

  // settings
  bool auto_reset_;
  int auto_reset_sec_;
  int pick_place_count_; // tracks how many pick_places have run

  MoveItBlocks()
    : auto_reset_(false),
      auto_reset_sec_(4),
      pick_place_count_(0),
      nh_("~")
  {
    ROS_INFO_STREAM_NAMED("moveit_blocks","Starting MoveIt Blocks");

    // Get arm info from param server
    nh_.param("ee_group_name", ee_group_name_, std::string("unknown"));
    nh_.param("planning_group_name", planning_group_name_, std::string("unknown"));

    planning_group_name_ = "left_arm";

    ROS_INFO_STREAM_NAMED("moveit_blocks","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("moveit_blocks","Planning Group: " << planning_group_name_);

    // Create MoveGroup for one of the planning groups
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
    move_group_->setPlanningTime(30.0);

    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
      ros::shutdown();

    joint_model_group = (move_group_->getRobotModel())->getJointModelGroup(planning_group_name_);
    eef_joint_model_group = (move_group_->getRobotModel())->getJointModelGroup(ee_group_name_);

    /** \brief First: name of the group that is parent to this end-effector group;
    Second: the link this in the parent group that this group attaches to */
    const std::string& ee_parent_link_name = joint_model_group->getEndEffectorParentGroup().second;
    const std::vector<std::string>& ee_attached_name = joint_model_group->getAttachedEndEffectorNames();

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_, "/rviz_visual_tools"));
    //visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->loadEEMarker(eef_joint_model_group); //, planning_group_name_);**

    simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_,true));

    // Let everything load
    ros::Duration(1.0).sleep();

    // Do it.
    startRoutine();
  }

  bool startRoutine()
  {
    // Debug - calculate and output table surface dimensions
    /*
    if( false )
    {
      double y_min, y_max, x_min, x_max;
      getTableWidthRange(y_min, y_max);
      getTableDepthRange(x_min, x_max);
      ROS_INFO_STREAM_NAMED("table","Blocks width range: " << y_min << " <= y <= " << y_max);
      ROS_INFO_STREAM_NAMED("table","Blocks depth range: " << x_min << " <= x <= " << x_max);
    }
    */

    // Create start block positions (hard coded)
    std::vector<MetaBlock> blocks;
    double block_y = 0.1;
    double block_x = 0.45; //0.35
    // Flip the side of the table the blocks are on depending on which arm we are using
    //if( arm_.compare("right") == 0 )
    //  block_y *= -1;
    blocks.push_back( createStartBlock(block_x, block_y, "Block1") );
    //blocks.push_back( createStartBlock(block_x+0.1,   block_y, "Block2") );
    //blocks.push_back( createStartBlock(block_x+0.2,   block_y, "Block3") );

    // The goal for each block is simply translating them on the x axis
    for (std::size_t i = 0; i < blocks.size(); ++i)
    {
      blocks[i].goal_pose = blocks[i].start_pose;
      blocks[i].goal_pose.position.x += 0.2;
    }

    // --------------------------------------------------------------------------------------------------------
    // Repeat pick and place forever
    while(ros::ok())
    {
      // Avoid dirty link transforms
      ros::spinOnce();
      // --------------------------------------------------------------------------------------------
      // Re-add all blocks
      for (std::size_t i = 0; i < blocks.size(); ++i)
      {
        resetBlock(blocks[i]);
      }

        // Do for all blocks
        for (std::size_t block_id = 0; block_id < blocks.size(); ++block_id)
        {
          // Pick -------------------------------------------------------------------------------------
          while(ros::ok())
          {
            ros::spinOnce();

            ROS_INFO_STREAM_NAMED("pick_place","Picking '" << blocks[block_id].name << "'");
            ROS_INFO_STREAM_NAMED("pick_place","Picking '" << blocks[block_id].start_pose.position.x);
            ROS_INFO_STREAM_NAMED("pick_place","Picking '" << blocks[block_id].start_pose.position.y);
            ROS_INFO_STREAM_NAMED("pick_place","Picking '" << blocks[block_id].start_pose.position.z);
            ROS_INFO_STREAM_NAMED("pick_place","Picking '" << blocks[block_id].start_pose.orientation.w);

            // Visualize the block we are about to pick
            visual_tools_->publishCuboid(blocks[block_id].start_pose, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE, rviz_visual_tools::BLUE);

            if( !pick(blocks[block_id].start_pose, blocks[block_id].name) )
            {
              ROS_ERROR_STREAM_NAMED("pick_place","Pick failed.");

              // Ask user if we should try again
              if( !promptUser() )
                exit(0);

              // Retry
              resetBlock(blocks[block_id]);
            }
            else
            {
              ROS_INFO_STREAM_NAMED("pick_place","Done with pick ---------------------------");
              break;
            }
          }
          // Place -------------------------------------------------------------------------------------
          while(ros::ok())
          {
            ros::spinOnce();

            ROS_INFO_STREAM_NAMED("pick_place","Placing '" << blocks[block_id].name << "'");

            // Publish goal block location
            visual_tools_->publishCuboid(blocks[block_id].goal_pose, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE, rviz_visual_tools::BLUE);

            if( !place(blocks[block_id].goal_pose, blocks[block_id].name) )
            {
              ROS_ERROR_STREAM_NAMED("pick_place","Place failed.");

              // Determine if the attached collision body as already been removed, in which case
              // we can ignore the failure and just resume picking
              /*
                if( !move_group_->hasAttachedObject(blocks[block_id].name) )
                {
                ROS_WARN_STREAM_NAMED("pick_place","Collision object already detached, so auto resuming pick place.");

                // Ask user if we should try again
                if( !promptUser() )
                break; // resume picking
                }
              */

              // Ask user if we should try again
              if( !promptUser() )
                exit(0);
            }
            else
            {
              ROS_INFO_STREAM_NAMED("pick_place","Done with place ----------------------------");
              break;
            }
          }
        } // loop through 3 blocks

        ROS_INFO_STREAM_NAMED("pick_place","Finished picking and out " << blocks.size()
          << " blocks. Total pick_places: " << pick_place_count_);

      // Ask user if we should repeat
      //if( !promptUser() )
      //  break;
      ros::Duration(1.0).sleep();
      break; //try pick only once for testing
    }

    // Everything worked!
    return true;
  }

  void resetBlock(MetaBlock block)
  {
    // Remove attached object
    visual_tools_->cleanupACO(block.name);

    // Remove collision object
    visual_tools_->cleanupCO(block.name);

    // Add the collision block
    visual_tools_->publishCollisionBlock(block.start_pose, block.name, BLOCK_SIZE);
    ROS_INFO_STREAM_NAMED("block_pick_place", "After block publish ");
  }

  MetaBlock createStartBlock(double x, double y, const std::string name)
  {
    MetaBlock start_block;
    start_block.name = name;

    // Position
    start_block.start_pose.position.x = x;
    start_block.start_pose.position.y = y;
    start_block.start_pose.position.z = BLOCK_SIZE / 2.0; //getTableHeight(-0.9);

    // Orientation
    double angle = 0; // M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_block.start_pose.orientation.x = quat.x();
    start_block.start_pose.orientation.y = quat.y();
    start_block.start_pose.orientation.z = quat.z();
    start_block.start_pose.orientation.w = quat.w();

    return start_block;
  }

  bool pick(const geometry_msgs::Pose& block_pose, std::string block_name)
  {
    std::vector<moveit_msgs::Grasp> possible_grasps;

    ROS_INFO_STREAM_NAMED("pick_place","Before generation of block grasps");

    // Pick grasp
    simple_grasps_->generateBlockGrasps( block_pose, grasp_data_, possible_grasps );

    ROS_INFO_STREAM_NAMED("pick_place","After generation of block grasps");

    double animate_speed = 0.1;
    // Visualize them
    //visual_tools_->publishAnimatedGrasps(possible_grasps, joint_model_group, animate_speed*0.1);
    visual_tools_->publishGrasps( possible_grasps, eef_joint_model_group, animate_speed );

    ROS_INFO_STREAM_NAMED("pick_place","After visual tools publishing");

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);
/*
    // Allow blocks to be touched by end effector
    {
      // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
      std::vector<std::string> allowed_touch_objects;
      allowed_touch_objects.push_back("Block1");
      allowed_touch_objects.push_back("Block2");
      allowed_touch_objects.push_back("Block3");
      allowed_touch_objects.push_back("Block4");

      // Add this list to all grasps
      for (std::size_t i = 0; i < possible_grasps.size(); ++i)
      {
        possible_grasps[i].allowed_touch_objects = allowed_touch_objects;
      }
    }
*/
    return move_group_->pick(block_name, possible_grasps);
  }

  bool place(const geometry_msgs::Pose& goal_block_pose, std::string block_name)
  {
    ROS_WARN_STREAM_NAMED("place","Placing '"<< block_name << "'");

    std::vector<moveit_msgs::PlaceLocation> place_locations;

    // Re-usable datastruct
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = grasp_data_.base_link_;
    pose_stamped.header.stamp = ros::Time::now();

    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
    {
      pose_stamped.pose = goal_block_pose;

      // Orientation
      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
      pose_stamped.pose.orientation.x = quat.x();
      pose_stamped.pose.orientation.y = quat.y();
      pose_stamped.pose.orientation.z = quat.z();
      pose_stamped.pose.orientation.w = quat.w();

      // Create new place location
      moveit_msgs::PlaceLocation place_loc;

      place_loc.place_pose = pose_stamped;

      visual_tools_->publishCuboid(place_loc.place_pose.pose, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE, rviz_visual_tools::BLUE);

      // Approach
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      post_place_retreat.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      post_place_retreat.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = grasp_data_.base_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
    }

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    move_group_->setPlannerId("RRTConnectkConfigDefault");

    return move_group_->place(block_name, place_locations);
  }

  bool promptUser()
  {
    // Make sure ROS is still with us
    if( !ros::ok() )
      return false;

    if( auto_reset_ )
    {
      ROS_INFO_STREAM_NAMED("pick_place","Auto-retrying in " << auto_reset_sec_ << " seconds");
      ros::Duration(auto_reset_sec_).sleep();
    }
    else
    {
      ROS_INFO_STREAM_NAMED("pick_place","Retry? (y/n)");
      char input; // used for prompting yes/no
      std::cin >> input;
      if( input == 'n' )
        return false;
    }
    return true;
  }

};

} //namespace
