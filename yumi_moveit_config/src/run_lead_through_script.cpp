#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <fstream>
#include <string>

// Define Global Constants
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)

// Namespace Commands and Variables
using namespace ros; // use namespace of ros
namespace planningInterface = moveit::planning_interface; // define commonly used namespace

// Function Prototypes
bool gotoGroupState(planningInterface::MoveGroup&, std::string);
bool gotoPose(planningInterface::MoveGroup&, geometry_msgs::Pose&);
bool gotoJoints(planningInterface::MoveGroup&, std::vector<double>& );
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
	std::string inputFile = argv[1];

	init(argc,argv,"run_lead_through"); // initialize ROS node

	// Start a background "spinner" for node to process ROS messages
	AsyncSpinner spinner(1);
	spinner.start();

	// Define Move Groups
	planningInterface::MoveGroup right_arm("right_arm");
	planningInterface::MoveGroup left_arm("left_arm");

	gotoGroupState(right_arm,"home"); // go to home position

	// Initialize Variables for Pulling From Input File
	std::string line;
	std::string command;
	std::string groupName_1;
	std::string groupName_2;
	std::vector<double> jointValues_right(8);
	std::vector<double> jointValues_left(1);
	std::vector<double>::size_type totalJoints_right;
	std::vector<double>::size_type totalJoints_left;

	// Get Joint Values and Execute Trajectories
	ROS_INFO("Opening file and starting execution.");
	std::ifstream leadThrough(inputFile.c_str());
	while  (std::getline(leadThrough,line)) {
		std::istringstream currentLine(line);
		if (!(currentLine >> command >> groupName_1 >> totalJoints_right >> jointValues_right[0] >> jointValues_right[1] >> jointValues_right[2] >> jointValues_right[3] >> jointValues_right[4] >> jointValues_right[5] >> jointValues_right[6] >> jointValues_right[7] >> groupName_2 >> totalJoints_left >> jointValues_left[0] >> jointValues_left[1] >> jointValues_left[2] >> jointValues_left[3] >> jointValues_left[4] >> jointValues_left[5] >> jointValues_left[6])) {
			break;
		} else {
			gotoJoints(right_arm,jointValues_right);
		}
	}
	ROS_INFO("Finished execution.");

	displayJointValues(right_arm); // display joint positions

	return 0;
}

/* -----------------------------------------------
   ------------- MOVEMENT FUNCTIONS --------------
   ----------------------------------------------- */
bool gotoGroupState(planningInterface::MoveGroup& group, std::string group_state) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-16
    PURPOSE: Move given group to specified group state
    FUTURE WORK: Need to add error checking to determine if the give group state is a valid group state

    INPUT(S):
      > group - move group that the user desires to display the joint values of
      > group_state - group state previously defined in the SRDf file that the user would like to move the specified group to
    OUTPUT(S):
      < success - indicates if the movement execution was successful
*/
	group.setNamedTarget(group_state); // set next target as given group state
	group.move(); // move to the group state

	// NOTE: Need to add error checking to make sure the given group_state exists DATE: 2016-06-16
	return 1;
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
      > pose - pose the user wants to move the given group to
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
		group.move(); // move to the next target
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


