#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <fstream>
#include <string>

// Define Global Constants
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)
const std::string pathsDirectory = "/home/yumi/yumi_ws/src/yumi/yumi_moveit_config/paths/";

// Namespace Commands and Variables
using namespace ros; // use namespace of ros
namespace planningInterface = moveit::planning_interface; // define commonly used namespace for planning interface
namespace moveitCore = moveit::core; // define commonly used namespace for planning interface

// Function Prototypes
bool gotoGroupState(planningInterface::MoveGroup&, std::string);
bool gotoPose(planningInterface::MoveGroup&, geometry_msgs::Pose&);
bool gotoJoints(planningInterface::MoveGroup&, std::vector<double>&);
bool closeHand(planningInterface::MoveGroup&);
bool openHand(planningInterface::MoveGroup&);
bool executePlanner(planningInterface::MoveGroup&);

void displayJointValues(planningInterface::MoveGroup&);
geometry_msgs::Pose createPoseXYZ(double, double, double);

/* -----------------------------------------------
   ---------------- MAIN FUNCTION ----------------
   ----------------------------------------------- */
int main(int argc,char **argv) {

	// Get Desired Input File
	if (argv[1] == NULL) { // check if an output file name was supplied
		ROS_INFO("An input file name needs to be executed as an additional argument."); // notify user that no output file name was supplied 
		shutdown(); // showdown the node
		return 1; // exit due to error
	}
	std::stringstream fullPath; // initialize variable to concatenate the full path
	fullPath << pathsDirectory << argv[1] << ".txt"; // concatenate full path
	std::string inputFile = fullPath.str(); // store the full path

	init(argc,argv,"run_lead_through"); // initialize ROS node

	// Start a background "spinner" for node to process ROS messages
	AsyncSpinner spinner(1);
	spinner.start();

	// Define Move Groups
	planningInterface::MoveGroup right_arm("right_arm");
	planningInterface::MoveGroup left_arm("left_arm");

	right_arm.setNumPlanningAttempts(3);

	gotoGroupState(left_arm,"calc"); // go to calc position with the left arm
	gotoGroupState(right_arm,"home"); // go to home position with the right arm

	std::vector<planningInterface::MoveGroup::Plan> plan = generatePaths(inputFile); // generate paths from input file

	displayJointValues(right_arm); // display joint positions

	return 0;
}
/* -----------------------------------------------
   ---------- PATH GENERATOR FUNCTIONS -----------
   ----------------------------------------------- */
std::vector<planningInterface::MoveGroup::Plan> generatePaths(std:string inputFile) {
/*
*/	
	// Initialize Variables for Pulling From Input File
	std::string line;
	std::string command;
	std::string groupName_1;
	std::string groupName_2;
	std::vector<double> jointValues_right(8);
	std::vector<double> jointValues_left(7);
	std::vector<double>::size_type totalJoints_right;
	std::vector<double>::size_type totalJoints_left;

	std::vector<planningInterface::MoveGroup::Plan> plan;
	int planIndex = 0;

	// Get Joint Values and Execute Trajectories
	ROS_INFO("Opening file and starting execution.");
	std::ifstream leadThrough(inputFile.c_str());
	if (leadThrough.is_open()) {
		while(std::getline(leadThrough,line)) {
			std::istringstream currentLine(line);
			currentLine >> command >> groupName_1 >> totalJoints_right >> jointValues_right[0] >> jointValues_right[1] >> jointValues_right[2] >> jointValues_right[3] >> jointValues_right[4] >> jointValues_right[5] >> jointValues_right[6] >> jointValues_right[7] >> groupName_2 >> totalJoints_left >> jointValues_left[0] >> jointValues_left[1] >> jointValues_left[2] >> jointValues_left[3] >> jointValues_left[4] >> jointValues_left[5] >> jointValues_left[6];
		
			if (planIndex == 0) {
				right_arm.setStartStateToCurrentState();
			} else {
				moveitCore::RobotState state(right_arm.getRobotModel());
				moveitCore::jointTrajPointToRobotState(plan[planIndex-1].trajectory_.joint_trajectory, (plan[planIndex-1].trajectory_.joint_trajectory.points.size()-1), state);
				right_arm.setStartState(state);
			}

			right_arm.setJointValueTarget(jointValues_right);
			planningInterface::MoveGroup::Plan currentPlan;
			bool success = right_arm.plan(currentPlan);
			planIndex++;

			if ((success == 1) && (planIndex > 1)) {
				plan.push_back(currentPlan);
			} else if ((success == 1) && (planIndex == 1)) {
				plan.push_back(currentPlan);
			} else {
				ROS_WARN("Plan failed for trajectory point: %d",planIndex);
				return 1;
			}

		}

		leadThrough.close();
	}
	ROS_INFO("Finished planning.");

	// Execute Planned Movements
	ROS_INFO("Executing trajectories.");
	for (int i = 0; i < plan.size(); i++) {
		right_arm.execute(plan[i]);
		ROS_INFO("Trajectory %d executed",i);
	}
	ROS_INFO("Finished executing trajectories.");

	return plan;
}

std::vector<planningInterface::MoveGroup::Plan> generatePaths(std:string inputFile) {
/* 
   NOTE: Currently broken, need to be update later on
*/
	// Initialize Variables for Pulling From Input File
	std::string line;
	std::string command;
	std::string groupName_1;
	std::string groupName_2;
	std::vector<double> jointValues_right(8);
	std::vector<double> jointValues_left(7);
	std::vector<double>::size_type totalJoints_right;
	std::vector<double>::size_type totalJoints_left;

	planningInterface::MoveGroup::Plan plan;
	int planIndex = 0;

	// Get Joint Values and Execute Planner
	ROS_INFO("Opening file and starting execution.");
	std::ifstream textFile(inputFile.c_str());
	if (textFile.is_open()) {
		while(std::getline(textFile,line)) {
			std::istringstream currentLine(line);
			currentLine >> command >> groupName_1 >> totalJoints_right >> jointValues_right[0] >> jointValues_right[1] >> jointValues_right[2] >> jointValues_right[3] >> jointValues_right[4] >> jointValues_right[5] >> jointValues_right[6] >> jointValues_right[7] >> groupName_2 >> totalJoints_left >> jointValues_left[0] >> jointValues_left[1] >> jointValues_left[2] >> jointValues_left[3] >> jointValues_left[4] >> jointValues_left[5] >> jointValues_left[6];
		
			if (planIndex == 0) {
				right_arm.setStartStateToCurrentState();
			} else {
				moveitCore::RobotState state(right_arm.getRobotModel());
				moveitCore::jointTrajPointToRobotState(plan.trajectory_.joint_trajectory, (plan.trajectory_.joint_trajectory.points.size()-1), state);
				right_arm.setStartState(state);
			}

			right_arm.setJointValueTarget(jointValues_right);
			planningInterface::MoveGroup::Plan currentPlan;
			bool success = right_arm.plan(currentPlan);
			planIndex++;

			if ((success == 1) && (planIndex > 1)) {
				std::size_t trajectorySize = currentPlan.trajectory_.joint_trajectory.points.size();
				trajectory_msgs::JointTrajectoryPoint point;
				for (int i = 0; i < trajectorySize; i++) {
					point = currentPlan.trajectory_.joint_trajectory.points[i];
					plan.trajectory_.joint_trajectory.points.push_back(point);
				}
				ROS_INFO("Plan created for trajectory point: %d",planIndex);
			} else if ((success == 1) && (planIndex == 1)) {
				plan = currentPlan;
			} else {
				ROS_WARN("Plan failed for trajectory point: %d",planIndex);
				return 1;
			}

		}

		textFile.close();
	}
	ROS_INFO("Finished planning.");

	int totalTrajectoryPoints = plan.trajectory_.joint_trajectory.points.size();
	ROS_INFO("Total trajectory points: %lu",totalTrajectoryPoints);

	// Executed Planned Movements
	ROS_INFO("Executing trajectories.");
	planningInterface::MoveGroup::Plan newPlan;
	newPlan = plan;

	int trajSize = 20;
	int sendTrajSize = ceil(plan.trajectory_.joint_trajectory.points.size() / trajSize);
	ROS_INFO("Total trajectories to send: %d",sendTrajSize);
	for (int i = 0; i < sendTrajSize; i++) {
		newPlan.trajectory_.joint_trajectory.points = {}; // NOTE: Generates a warning DATE: 2016-06-20
		ROS_INFO("Creating trajectory.");
		for (int j = 0; j < trajSize; j++) {
			newPlan.trajectory_.joint_trajectory.points[j] = plan.trajectory_.joint_trajectory.points[j+(i*trajSize)];
		}
		right_arm.execute(newPlan);
		ROS_INFO("Trajectory %d executed",i);
	}
	ROS_INFO("Finished executing trajectories.");

	return plan;
}

/* -----------------------------------------------
   ------------- MOVEMENT FUNCTIONS --------------
   ----------------------------------------------- */
bool gotoGroupState(planningInterface::MoveGroup& group, std::string group_state) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Move given group to specified group state

    INPUT(S):
      > group - move group that the user desires to display the joint values of
      > group_state - group state previously defined in the SRDf file that the user would like to move the specified group to
    OUTPUT(S):
      < success - indicates if the movement execution was successful
*/
	group.setNamedTarget(group_state); // set next target as given group state
	bool success = executePlanner(group); // execute planner and movement

	return success; // return whether the path planner was able to create a path or not
}

bool gotoPose(planningInterface::MoveGroup& group, geometry_msgs::Pose& pose) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Set next target for given group using given pose and execute the path planner

    INPUT(S):
      > group - move group that the user desires to display the joint values of
      > pose - pose the user wants to move the given group to
    OUTPUT(S):
      < success - indicates if the path planner was successful
*/
	group.setPoseTarget(pose); // set next target for given group using given pose
	bool success = executePlanner(group); // execute planner and movement

	return success; // return whether the path planner was able to create a path or not
}

bool gotoJoints(planningInterface::MoveGroup& group, std::vector<double>& jointValues) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Set next target for given group using given pose and execute the path planner

    INPUT(S):
      > group - move group that the user desires to display the joint values of
      > jointValues - 1D array of joint values that the user wants to move the given group to
    OUTPUT(S):
      < success - indicates if the path planner was successful
*/
	group.setJointValueTarget(jointValues); // set next target for given group using given joint positions
	bool success = executePlanner(group); // execute planner and movement

	return success; // return whether the path planner was able to create a path or not
}

bool openHand(planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Open the gripper for given move group

    INPUT(S):
      > group - move group that the user desires to close the gripper of
    OUTPUT(S):
      < success - indicates if the movement execution was successful
*/
	// Initialize Variables
	std::vector<double> jointValues; // array to contain joint values
	std::vector<double>::size_type totalJoints; // unsigned long int signifying total joints in move group

	// Get Joint Values and Total Joints
	jointValues = group.getCurrentJointValues(); // get joint values
	totalJoints = jointValues.size(); // get amount of joints within move group

	// Adjust Gripper Joint to Open Position 
	jointValues[totalJoints-1] = gripper_open_position; // overwrite gripper position to open gripper
	group.setJointValueTarget(jointValues); // set open gripper position as the next target
	bool success = executePlanner(group); // execute planner and movement

	return success; // return whether the path planner was able to create a path or not
}

bool closeHand(planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Close the gripper for given move group

    INPUT(S):
      > group - move group that the user desires to close the gripper of
    OUTPUT(S):
      < success - indicates if the movement execution was successful
*/
	// Initialize Variables
	std::vector<double> jointValues; // array to contain joint values
	std::vector<double>::size_type totalJoints; // unsigned long int signifying total joints in move group

	// Get Joint Values and Total Joints
	jointValues = group.getCurrentJointValues(); // get joint values
	totalJoints = jointValues.size(); // get amount of joints within move group

	// Adjust Gripper Joint to Open Position 
	jointValues[totalJoints-1] = gripper_closed_position; // overwrite gripper position to open gripper
	group.setJointValueTarget(jointValues); // set open gripper position as the next target
	bool success = executePlanner(group); // execute planner and movement

	return success; // return whether the path planner was able to create a path or not
}

bool executePlanner(planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Close gripper for given move group
    NOTE: A new target must be set before running this function. Otherwise the path planner will
          not do anything since it is being asked to move to the position that it is already in.
    
    INPUT(S):
      > group - move group that the user wants to execute the path planner for
    OUTPUT(S):
      < success - indicates if the movement execution was successful
*/
	// Construct and Execute Path Planner
	planningInterface::MoveGroup::Plan plan;
	bool success = group.plan(plan);

	ROS_INFO("Attempting Path Planning. Status: %s",success?"Success":"Failed"); // display result to user

	if (success == 1) { // if the path planning was successful
		group.execute(plan); // move to the next target
	}

	return success; // return whether the path planner was able to create a path or not
}

/* -----------------------------------------------
   -------------- GENERAL FUNCTIONS --------------
   ----------------------------------------------- */
void displayJointValues(planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Display all the joint values of a give move group

    INPUT(S):
      > group - move group that the user desires to display the joint values of
    OUTPUT(S): None
*/
	// Initialize Variables
	std::vector<double> jointValues; // array to contain joint values
	std::vector<double>::size_type totalJoints; // unsigned long int signifying total joints in move group

	// Get Joint Values and Total Joints
	jointValues = group.getCurrentJointValues(); // get joint values
	totalJoints = jointValues.size(); // get amount of joints within move group

	// Iterate Through Joints and Display Positions
	ROS_INFO("Total joints: %lu",totalJoints);
	for (int i = 0; i < totalJoints; i++) {
		ROS_INFO("Joint %d Position: %f",i+1,jointValues[i]);
	}
}

geometry_msgs::Pose createPoseXYZ(double x, double y, double z) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Create a pose, in repesect to the world frame, using given (x,y,z) values

    INPUT(S):
      > x - X-coordinate of the design position for the end effector of a given group
      > y - Y-coordinate of the design position for the end effector of a given group
      > z - Z-coordinate of the design position for the end effector of a given group
    OUTPUT(S):
      < pose - constructed pose with user (x,y,z) values
*/
	geometry_msgs::Pose pose; // construct an empty pose

	// Assign Values
	pose.orientation.w = 1; // NOTE: What does this do? DATE: 2016-06-16
	pose.position.x = x; // store desired x location into pose structure
	pose.position.y = y; // store desired y location into pose structure
	pose.position.z = z; // store desired z location into pose structure

	return pose; // return generated pose
}


