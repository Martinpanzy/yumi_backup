#include <ros/ros.h> // include ROS commands
#include <moveit/move_group_interface/move_group.h> // include MoveIt! commands
#include <std_msgs/String.h> // include string messages
#include <sstream> // include string streams
#include <fstream> // allow file streaming

// Namespace Commands and Variables
using namespace ros; // using ROS namespace
namespace planningInterface = moveit::planning_interface; // define commonly used namespace

// Initialize Global Variables
bool writeToFile = 0;
std::string command; // initialize variable for storing the type of point being stored

std::string outputFile; // initialize variable to store the full path for the output file
std::string outputFilePath = "/home/yumi/yumi_ws/src/yumi/yumi_moveit_config/paths/"; // initializing an output file with default name

// Function Prototypes
void commandCallback(std_msgs::String);
void writePoint_BothArms(planningInterface::MoveGroup&, planningInterface::MoveGroup&, std::string);
void writePoint_OneArm(planningInterface::MoveGroup&, std::string, std::string);

/* -----------------------------------------------
   --------------- MAIN FUNCTION -----------------
   ----------------------------------------------- */
int main(int argc,char **argv) {

	// ========== FUTURE WORK ==========
	// Need to also check if the input is a valid file name DATE: 2016-06-17
	// Need to ensure YuMi server and YuMi quick commands have been called in order for this script to work properly
	// Need to check potential issue of gettin all zero joint value for the first write to file
	// =================================

	// Get Desired Output File
	if (argv[1] == NULL) { // check if an output file name was supplied
		ROS_INFO("A output file name needs to be executed as an additional argument."); // notify user that no output file name was supplied 
		shutdown(); // showdown the node
		return 1; // exit due to error
	}

	init(argc,argv,"lead_through"); // initialize ROS node

	// Define Move Groups
	planningInterface::MoveGroup right_arm("right_arm"); // initialize structure for right arm move group
	planningInterface::MoveGroup left_arm("left_arm"); // initialize structure for left arm move group

	// Construct File Location for Output File
	std::stringstream fullPath; // initialize variable to concatenate the full path
	fullPath << outputFilePath << argv[1] << ".txt"; // concatenate full path
	outputFile = fullPath.str(); // store the full path
	ROS_INFO("Output file at: yumi_moveit_config/path/%s.txt",argv[1]); // notify the user of the chosen output file name

	// Setup Subscriber
	NodeHandle nh; // initialize a node handle
	Subscriber sub = nh.subscribe("lead_through_commands",1000,commandCallback); // subscribe to "lead_through_comamnds" topic
	
	// Wait For Published Commands
	while(ok()) {
		if (writeToFile == 1) { // if the user would like to store the current joint positions to the file
			writePoint_BothArms(right_arm,left_arm,command); // write the current joint positions to the file
			writeToFile = 0; // reset flag variable to indicate that the point has been written to file
		}
		spinOnce(); // allow time to check callbacks
	}

	return 0; // indicate successful main function execution
}


/* -----------------------------------------------
   ------------ SUBSCRIBER FUNCTIONS -------------
   ----------------------------------------------- */
void commandCallback(std_msgs::String msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-17
    PURPOSE: Store point type from user and indicate that the user would like to store the current point

    INPUTS:
      > msg - string message from "lead_through_commands" topic
    OUTPUT(S): None
*/
	command = msg.data; // take command form topic into local variable
	writeToFile = 1; // indicate that the user would like to write the current point to the file
}

/* -----------------------------------------------
   -------------- GENERAL FUNCTIONS --------------
   ----------------------------------------------- */
void writePoint_BothArms(planningInterface::MoveGroup& right_arm, planningInterface::MoveGroup& left_arm, std::string command) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-17
    PURPOSE: Execute logging for joint locations for two arms

    INPUTS:
      > right_arm - right arm move group
      > left_arm - left arm move group
      > command - string signifying type of point
    OUTPUT(S): None
*/
	// Intiialize Variables
	std::stringstream trajectory_point; // create a string stream for concatenation
	std::ofstream leadThrough; // create file stream variable

	// Get Joint Values and Total Joints
	std::vector<double> jointValues_right = right_arm.getCurrentJointValues(); // get joint values for right arm
	std::vector<double> jointValues_left  = left_arm.getCurrentJointValues(); // get joint values for left arm
	std::vector<double>::size_type totalJoints_right = jointValues_right.size(); // get total amount of joints in right arm
	std::vector<double>::size_type totalJoints_left  = jointValues_left.size(); // get total amount of joints in right arm

	// Build Trajectory Point As String
	trajectory_point << command << " " << "right_arm" << " " << totalJoints_right << " "; // add command and header for right arm joints
	for (int i = 0; i < totalJoints_right; i++) trajectory_point << jointValues_right[i] << " "; // add right joint values
	trajectory_point << "left_arm" << " " <<  totalJoints_left << " "; // add header for left arm joints
	for (int i = 0; i < totalJoints_left; i++) trajectory_point << jointValues_left[i] << " "; // add left joint values

	std::string outputLine = trajectory_point.str(); // get stringstream as a string

	// Open File and Add Trajectory Point
	leadThrough.open(outputFile.c_str(), std::ofstream::out | std::ofstream::app); // open file in writing and append mode
	leadThrough << outputLine << "\r\n"; // write to file
	leadThrough.close(); // close the file

	ROS_INFO("Position stored with command: %s",command.c_str()); // notify user the command was stored
}

void writePoint_OneArm(planningInterface::MoveGroup& group, std::string group_name, std::string command) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-17
    PURPOSE: Execute logging for joint locations for one arm

    INPUTS:
      > group - desired move group to store value for
      > group_name - name of the move group
      > command - string signifying type of point
    OUTPUT(S): None
*/
	// Intiialize Variables
	std::stringstream trajectory_point; // initialize a string stream for concatenation
	std::ofstream leadThrough; // initialize file stream variable
	std::string outputLine; // initialize variable to store the output line

	// Get Joint Values and Total Joints
	std::vector<double> jointValues = group.getCurrentJointValues(); // get joint values for given group
	std::vector<double>::size_type totalJoints = jointValues.size(); // get total amount of joints in given group

	// Build Trajectory Point As String
	trajectory_point << command << " " << group_name << " " << totalJoints << " "; // add command and header for right arm joints
	for (int i = 0; i < totalJoints; i++) trajectory_point << jointValues[i] << " "; // add right joint values
	outputLine = trajectory_point.str(); // get stringstream as a string

	// Open File and Add Trajectory Point
	leadThrough.open(outputFile.c_str(), std::ofstream::out | std::ofstream::app); // open file in writing and append mode
	leadThrough << outputLine << "\r\n"; // write to file
	leadThrough.close(); // close the file

	ROS_INFO("Position stored with command: %s",command.c_str()); // notify user the command was stored
}