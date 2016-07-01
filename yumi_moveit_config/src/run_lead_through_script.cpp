#include <ros/ros.h> // include node functionality
#include <moveit/move_group_interface/move_group.h> // include move group functionality

#include <moveit_msgs/GetPositionIK.h>

#include <moveit/robot_model_loader/robot_model_loader.h> // include robot model loader functionality for IK
#include <moveit/robot_model/robot_model.h> // include robot model capabilities for IK
#include <moveit/robot_state/robot_state.h> // include robot state capabiliteis for IK

#include <moveit/robot_state/conversions.h> // include robot state conversion functionality
#include <moveit_msgs/RobotTrajectory.h> // include robot trajectory
#include <geometry_msgs/Pose.h> // include pose for move group functionality
#include <sstream> // include string stream functionality
#include <fstream> // include file stream functionality
#include <string> // include string functionality

// Define Global Constants
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)
const std::string pathsDirectory = "/home/yumi/yumi_ws/src/yumi/yumi_moveit_config/paths/"; // full path to folder where trajectory text files should be stored

// Namespace Commands and Variables
using namespace ros; // use namespace of ros
namespace planningInterface = moveit::planning_interface; // define commonly used namespace for planning interface
namespace moveitCore = moveit::core; // define commonly used namespace for planning interface

// Define Structures
struct planner {
	std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to retrieve planned paths for trajectory points
	std::string groupName; // include group name
	bool success; // include boolean to retrieve whether the planner was successful or not
	int totalPlans;
};

struct trajectoryJoints {
	std::string groupName; // include group name
	std::string intendedGroup;
	std::vector<std::vector<double>> joints;
	int totalJoints;
	int totalPoints;
};

struct trajectoryPoses {
	std::string groupName;
	std::string intendedGroup;
	std::vector<geometry_msgs::Pose> pose_left;
	std::vector<geometry_msgs::Pose> pose_right;
	int totalPoints;
};

// Function Prototypes
std::string getFileDataType(std::string);
trajectoryJoints getTrajectoryJoints(planningInterface::MoveGroup&, std::string);
trajectoryPoses getTrajectoryPoses(planningInterface::MoveGroup&, std::string);

planner generatePlans(planningInterface::MoveGroup&, trajectoryJoints&);
planner generatePlans(planningInterface::MoveGroup&, trajectoryPoses&);
planner generatePlans(planningInterface::MoveGroup&, planningInterface::MoveGroup&, planningInterface::MoveGroup&, trajectoryPoses&) ;
bool executePlans(planningInterface::MoveGroup&, planner&);

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
	std::string inputFile = fullPath.str(); // retrieve the full path

	init(argc,argv,"run_lead_through"); // initialize ROS node
	NodeHandle nodeHandle; // initialize node handle

	ServiceClient serviceIK = nodeHandle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik"); // create client for IK service

	// Start a background "spinner" for node to process ROS messages
	AsyncSpinner spinner(1);
	spinner.start();

	// Define Move Groups
	planningInterface::MoveGroup left_arm("left_arm");
	planningInterface::MoveGroup right_arm("right_arm");
	planningInterface::MoveGroup both_arms("both_arms");

	// Set Planning Parameters
	left_arm.setNumPlanningAttempts(5); // set number of planning attempts for left arm
	right_arm.setNumPlanningAttempts(5); // set number of planning attempts for right arm
	both_arms.setNumPlanningAttempts(5); // set number of planning attempts for both arms

	// Get Trajectory From File
	planner plans;

	std::string dataType = getFileDataType(inputFile);
	if (dataType.compare("joints") == 0) {
		trajectoryJoints jointTrajectory = getTrajectoryJoints(both_arms,inputFile);
		if (jointTrajectory.groupName.compare("") != 0) {
			plans = generatePlans(both_arms,jointTrajectory);
		}
	} else if (dataType.compare("poses") == 0) {
		trajectoryPoses poseTrajectory = getTrajectoryPoses(both_arms,inputFile);
		if (poseTrajectory.groupName.compare("") != 0) {
			plans = generatePlans(left_arm,right_arm,both_arms,poseTrajectory);
		}
	}

	bool success = executePlans(both_arms,plans);




	return 0;
}

/* -----------------------------------------------
   ----------- FILE READNG FUNCTIONS -------------
   ----------------------------------------------- */
std::string getFileDataType(std::string inputFile) {

	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string dataType;
	bool success = true;

	ROS_INFO("Getting file data type.");

	// Get Provided File Data Type
	std::ifstream textFile(inputFile.c_str()); // open provided text file
	if (textFile.is_open()) { // if the file was able to successfully open
		// Get Data Type for Provided File
		if (std::getline(textFile,line)) { // if the file exists and contains a first line
			std::istringstream firstLine(line); // create a string steam element for delimiting data by spaces
			firstLine >> dataType; // get the data type from the provided file
			if (!((dataType.compare("joints") == 0)  || (dataType.compare("poses") == 0))) { // if the provided indended group name is not recognized
				ROS_ERROR("Provided file is malformatted or the header contains a non-recognized data type."); // notify the user the indended group name from the input file is not recognized
				ROS_WARN("The first line needs to indicate what type of data is in the provided file."); // notify the user the purpose of the indended group name
				ROS_WARN("Recognized data types: joints, poses"); // notify the user of recognized group names
				ROS_WARN("Provided group: %s",dataType.c_str()); // notify the user of the intended group retrieved from file
				success = false;
			}
		} else { // if the provided file is empty
			ROS_ERROR("The provided file is empty."); // notify the user that the provided file is empty
			success = false;
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s",inputFile.c_str()); // notify user of the supplied file
		success = false;
	}

	// Return Data Type if Retrieved
	if (success) {
		ROS_INFO("File contains data of type: %s",dataType.c_str());
		return dataType;
	} else {
		ROS_WARN("File data type was not able to be retrived.");
		return "";
	}
}

trajectoryJoints getTrajectoryJoints(planningInterface::MoveGroup& group, std::string inputFile) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-28
	PURPOSE: Retrive joint values from a provided file for the provided group

	INPUT(S):
		> group - group data to retrieve from file
		> inputFile - file name that contains the desired point
	OUTPUT(S):
		< trajectoryJoints - joint trajectory structure containing all joint values retrived from provided file

	DEPENDENCIES: None

	INPUT FILE CONVENTION: The input file must have the following convention for each trajectory point:
		- $(Command Name) $(Group 1 Name) $(Group 1 Total Joints (N_1)) $(Joint 1 Value) ... $(Joint N_1 Value) $(Group 2 Name) $(Group 2 Total Joints (N_2)) $(Joint 1 Value) ... $(Joint N_2 Value)

	NOTE: Currently not doing anything with the joint command from provided input file | DATE: 2016-06-22
*/
	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string command; // variable to retrieve the command for each trajectory point from the input file
	std::string dataType; // variable to retrieve the data type from provided file
	std::string intendedGroup; // variable to retrieve the intended arm for the data in the provided file
	int lineIndex = 0; // counter to indicate the current line

	std::string groupName_1; // variable to retrieve the first group name
	std::string groupName_2; // variable to retrieve the second group name
	std::vector<double>::size_type totalJoints_1; // variable to retrieve the joint values for the first group
	std::vector<double>::size_type totalJoints_2; // variable to retrieve the joint values for the second group

	std::string groupName = group.getName(); // get name of the provided group
	trajectoryJoints trajectory_joints; // create structure to retrieve joint values from file
	trajectory_joints.groupName = groupName; // set the group name for the joint values structure
	int groupName_index; // variable to indicate which group the user would like to retrive data for

	// Get Group Name
	if (groupName.compare("left_arm") == 0) { // if the user would like to only retrieve data for the left arm
		groupName_index = 1; // indicate that the user would like to only retrieve data for the left arm
	} else if (groupName.compare("right_arm") == 0) { // if the user would like to only retrieve data for the right arm
		groupName_index = 2; // indicate that the user would like to only retrieve data for the right arm
	} else if (groupName.compare("both_arms") == 0) { //if the user would like to retrieve data for both arms
		groupName_index = 3; // indicate that the user would like to retrieve data for both arms
	} else { // if the provided group is not what was expected
		groupName_index = -1; // indicate that the provided group is not recognized
		ROS_ERROR("Provided group name is not recognized."); // notify the user of the issue
		ROS_WARN("Recognized group names list: left_arm, right_arm, both_arms"); // notify the user of the allowed groups
		ROS_WARN("Provided group: %s",groupName.c_str()); // notify the user of the provided group name
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Getting stored trajectory joints from file."); // notify user that the planner is about to start
	ROS_INFO("File location: %s",inputFile.c_str()); // notify user of the full path of the input file

	// Open File and Generate Plans for Trajectories
	std::ifstream textFile(inputFile.c_str()); // open provided text file
	if (textFile.is_open()) { // if the file was able to successfully open
		// Get Intended Arm for Data in Provided File
		if (std::getline(textFile,line)) { // if the file exists and contains a first line
			std::istringstream firstLine(line); // create a string steam element for delimiting data by spaces
			firstLine >> dataType >> intendedGroup; // get the intended arm for the data in the provided file
			if (!((intendedGroup.compare("left_arm") == 0)  || (intendedGroup.compare("right_arm") == 0) || (intendedGroup.compare("both_arms") == 0))) { // if the indended group name from the provided is not recognized
				ROS_WARN("Provided file is malformatted or the header contains a non-recognized group name."); // notify the user the indended group name from the input file is not recognized
				ROS_WARN("The first line needs to indicate which group the data in the provided file is inteded for."); // notify the user the purpose of the indended group name
				ROS_WARN("Recognized groups: left_arm, right_arm, both_arms"); // notify the user of recognized group names
				ROS_WARN("Provided group: %s",intendedGroup.c_str()); // notify the user of the intended group retrieved from file
			} else { // if the intended group name from the provided file was recognized
				trajectory_joints.intendedGroup = intendedGroup; // retrieve the inteded group name
				if (trajectory_joints.groupName.compare(trajectory_joints.intendedGroup) != 0) { // if the group name and intended group name do not match
					ROS_WARN("The provided group is not the same as the group that the provided file was inteded for."); // notify the user that the group name and intended group name do not match
					ROS_WARN("The path planner and execution may not work as intended."); // notify the user that the planner and exeution of the plans may not work as intended
					ROS_WARN("Provided group: %s",trajectory_joints.groupName.c_str()); // notify the user of the provided group name
					ROS_WARN("Intended group: %s",trajectory_joints.intendedGroup.c_str()); // notify the user of the indeded group name
				}
			}
		} else { // if the provided file is empty
			ROS_ERROR("The provided file is empty."); // notify the user that the provided file is empty
			groupName_index = -1; // indicate that an error occurred when opening up the file
		}

		// Get Remaining Data from Provided File
		while (std::getline(textFile,line)) { // while there is still a new line to take from
			// Create String Steamer and Get Command for Current Trajectory Point
			std::istringstream currentLine(line); // create a string steam element for delimiting data by spaces
			currentLine >> command; // get command name for current trajectory point

			// Get Group Names and Joint Values for Current Trajectory Point
			currentLine >> groupName_1 >> totalJoints_1; //  get the command, group name for the first group, and total joints
			std::vector<double> jointValues_1(totalJoints_1); // create vector of doubles to retrieve joint values for the first group
			for (int joint = 0; joint < totalJoints_1; joint++) { // iterate through joint values
				currentLine >> jointValues_1[joint]; // retrieve the current joint value
			}

			currentLine >> groupName_2 >> totalJoints_2; // get the group name for the second group and the total joints
			std::vector<double> jointValues_2(totalJoints_2); // create vector of doubles to retrieve join values for the second group
			for (int joint = 0; joint < totalJoints_2; joint++) { // iterate through joint values
				currentLine >> jointValues_2[joint]; // retrieve the current joint value
			}

			/* ======================================================================================== */
			/* NEED TO ADD CHECK TO MAKE SURE THE INPUT FILE HAS CORRECT AMOUNT OF INPUTS FOR EACH LINE */
			/* ======================================================================================== */

			if (lineIndex == 0) { // if this is the first time running through the while loop
				if (groupName_index == 3) { // if the user would like to retrive data for both arms
					if (((groupName_1.compare("right_arm") == 0) || (groupName_2.compare("right_arm") == 0)) && ((groupName_1.compare("left_arm") == 0) || (groupName_2.compare("left_arm") == 0))) { // if both group names from the provided file are what was expected
						trajectory_joints.totalJoints = totalJoints_1 + totalJoints_2; // store the total joints for both arms
					} else { // if one or both group names from the provided file were not expected
						ROS_ERROR("One of the group names within the provided input file is not recognized."); // notify the user that one of the group names from the provided file were not expected
						ROS_WARN("Recognized groups: left_arm, right_arm"); // notify the user of the expected group names
						ROS_WARN("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str()); // notify the user of the group names from the provided file
						groupName_index = -1; // indicate that an error occurred
						break; // break from the loop due to error
					}
				} else if (groupName_index == 2) { // if the user would like to only retrieve data for the right arm
					if (groupName_1.compare("right_arm") == 0) { // if the first group contains data for the right arm
						trajectory_joints.totalJoints = totalJoints_1; // store the total joints for the right arm
					} else if (groupName_2.compare("right_arm") == 0) { // if the second group contains data for the right arm
						trajectory_joints.totalJoints = totalJoints_2; // store the total joints for the right arm
					} else { // if none of the groups contained data for the right arm
						ROS_ERROR("One of the group names within the provided input file is not recognized."); // notify the user that one of the group names from the provided file were not expected
						ROS_WARN("Recognized groups: left_arm, right_arm"); // notify the user of the expected group names
						ROS_WARN("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str()); // notify the user of the group names from the provided file
						groupName_index = -1; // indicate that an error occurred
						break; // break from the loop due to error
					}
				} else if (groupName_index == 1) { // if the user would like to only retrive data for the left arm
					if (groupName_1.compare("left_arm") == 0) { // if the first group contains data for the left arm
						trajectory_joints.totalJoints = totalJoints_1; // store the total joints for the left arm
					} else if (groupName_2.compare("left_arm") == 0) { // if the second group contains data for the left arm
						trajectory_joints.totalJoints = totalJoints_2; // store the total joints for the left arm
					} else { // if none of the groups contained data for the left arm
						ROS_ERROR("One of the group names within the provided input file is not recognized."); // notify the user that one of the group names from the provided file were not expected
						ROS_WARN("Recognized groups: left_arm, right_arm"); // notify the user of the expected group names
						ROS_WARN("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str()); // notify the user of the group names from the provided file
						groupName_index = -1; // indicate that an error occurred
						break; // break from the loop due to error
					}
				} else if (groupName_index == -1) { // if an error previously occurred with the group name
					break; // break from the loop due to error
				}
			}

			// Make Sure Total Joints From Provided File Matches Total Joints of Provided Group
			std::vector<std::string> jointNames = group.getActiveJoints(); // get the names of all the active joints in the provided group
			if (trajectory_joints.totalJoints != jointNames.size()) { // if the total active joints in the provided group does not match the total joints retrieved from the current line of the provided file
				ROS_ERROR("The total joints in the provided group does not match the total joints retrieved from the provided file."); // notfiy the user of the total joint mismatch from the provided group and current line of the provided file
				ROS_WARN("Check the total joints in the provided group and ensure that each line within the provided file matches the total joints for the same group."); // notify user of a way to check how to fix this issue
				groupName_index = -1; // indicate that an error occurred
				break; // break from the loop due to error
			}

			if (groupName_index == 1) { // if the user would like to only retrieve data for the left arm
				if (groupName_1.compare("left_arm") == 0) { // if the first group contains data for the left arm
					trajectory_joints.joints.push_back(jointValues_1); // add the joint values for the left arm into the joint trajectory structure
				} else { // if the second group contains data for the left arm
					trajectory_joints.joints.push_back(jointValues_2); // add the joint values for the left arm into the joint trajectory structure
				}
			} else if (groupName_index == 2) { // if the user would like to only retrieve data for the right arm
				if (groupName_1.compare("right_arm") == 0) { // if the first group contains data for the right arm
					trajectory_joints.joints.push_back(jointValues_1); // add the joint values for the left arm into the joint trajectory structure
				} else { // if the second group contains data for the right arm
					trajectory_joints.joints.push_back(jointValues_2); // add the joint values for the left arm into the joint trajectory structure
				}
			} else if (groupName_index == 3) { // if the user would like to retrieve data for both arms
				std::vector<double> jointValues; // initialize vector to store combined joint values for both arms
				if (groupName_1.compare("left_arm") == 0) { // if the first group contains data for the left arm
					jointValues = jointValues_1; // store the first set joint values as the joint values from the first group
					jointValues.insert(jointValues.end(),jointValues_2.begin(),jointValues_2.end()); // store the second set of joint values for the right arm into the array
				} else { // if the second group contains data for the left arm
					jointValues = jointValues_2; // store the first set joint values as the joint values from the second group
					jointValues.insert(jointValues.end(),jointValues_1.begin(),jointValues_1.end()); // store the second set of joint values for the right arm into the array
				}
				trajectory_joints.joints.push_back(jointValues); // add the joint values for both arms into the joint trajectory structure
			}

			lineIndex++; // increment current line counter
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s",inputFile.c_str()); // notify user of the supplied file
	}

	if (groupName_index == -1) { // if an error occured with the group name or retrieving data from the provided file
		trajectory_joints.totalJoints = 0; // reset the total joints in the joint trajectory structure
		trajectory_joints.groupName = ""; // reset the group name in the joint trajectory structure
		ROS_ERROR("There was an error retriving file data. Please fix any issues stated above."); // notify the user an error occurred with the file reading or with the provided group
	} else { // if no errors occurred with the group name or retrieving data from the file
		trajectory_joints.totalPoints = trajectory_joints.joints.size(); // store the total trajectory points to the joint trajectory structure
		ROS_INFO("Retrieved %d points from input file.",trajectory_joints.totalPoints); // notify the user of the total trajectory points
	}

	return trajectory_joints; // return the join trajectory structure
}

trajectoryPoses getTrajectoryPoses(planningInterface::MoveGroup& group, std::string inputFile) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-28
	PURPOSE: Retrive poses from a provided file for the provided group

	INPUT(S):
		> group - group data to retrieve from file
		> inputFile - file name that contains the desired point
	OUTPUT(S):
		< trajectoryPoses - pose trajectory structure containing all poses retrived from provided file

	DEPENDENCIES: None

	INPUT FILE CONVENTION: The input file must have the following convention for each trajectory point:
		- $(Command Name) $(Group 1 Name) $(Group 1 Total Joints (N_1)) $(Joint 1 Value) ... $(Joint N_1 Value) $(Group 2 Name) $(Group 2 Total Joints (N_2)) $(Joint 1 Value) ... $(Joint N_2 Value)

	NOTE: Currently not doing anything with the joint command from provided input file | DATE: 2016-06-22
*/
	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string command; // variable to retrieve the command for each trajectory point from the input file
	std::string dataType; // variable to retrieve the data type from provided file
	std::string intendedGroup; // variable to retrieve the intended arm for the data in the provided file
	int lineIndex = 0; // counter to indicate the current line

	std::string groupName_1; // variable to retrieve the first group name
	std::string groupName_2; // variable to retrieve the second group name
	geometry_msgs::Pose pose_1; // variable to retrieve poses from the first group
	geometry_msgs::Pose pose_2; // variable to retrieve poses from the second group

	std::string groupName = group.getName(); // get name of the provided group
	trajectoryPoses trajectory_poses; // create structure to retrieve poses from file
	trajectory_poses.groupName = groupName; // set the group name for the pose structure
	int groupName_index; // variable to indicate which group the user would like to retrieve data for

	// Get Group Name
	if (groupName.compare("left_arm") == 0) { // if the user would like to only retrieve data for the left arm
		groupName_index = 1; // indicate that the user would like to only retrieve data for the left arm
	} else if (groupName.compare("right_arm") == 0) { // if the user would like to only retrieve data for the right arm
		groupName_index = 2; // indicate that the user would like to only retrieve data for the right arm
	} else if (groupName.compare("both_arms") == 0) { //if the user would like to retrieve data for both arms
		groupName_index = 3; // indicate that the user would like to retrieve data for both arms
	} else { // if the provided group is not what was expected
		groupName_index = -1; // indicate that the provided group is not recognized
		ROS_ERROR("Provided group name is not recognized."); // notify the user of the issue
		ROS_WARN("Recognized group names list: left_arm, right_arm, both_arms"); // notify the user of the allowed groups
		ROS_WARN("Provided group: %s",groupName.c_str()); // notify the user of the provided group name
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Getting stored trajectory poses from file."); // notify user that the planner is about to start
	ROS_INFO("File location: %s",inputFile.c_str()); // notify user of the full path of the input file

	// Open File and Generate Plans for Trajectories
	std::ifstream textFile(inputFile.c_str()); // open provided text file
	if (textFile.is_open()) { // if the file was able to successfully open
		// Get Intended Arm for Data in Provided File
		if (std::getline(textFile,line)) { // if the file exists and contains a first line
			std::istringstream firstLine(line); // create a string steam element for delimiting data by spaces
			firstLine >> dataType >> intendedGroup; // get the intended arm for the data in the provided file
			if (!((intendedGroup.compare("left_arm") == 0)  || (intendedGroup.compare("right_arm") == 0) || (intendedGroup.compare("both_arms") == 0))) { // if the provided indended group name is not recognized
				ROS_WARN("Provided file is malformatted or the header contains a non-recognized group name."); // notify the user the indended group name from the input file is not recognized
				ROS_WARN("The first line needs to indicate which group the data in the provided file is inteded for."); // notify the user the purpose of the indended group name
				ROS_WARN("Recognized groups: left_arm, right_arm, both_arms"); // notify the user of recognized group names
				ROS_WARN("Provided group: %s",intendedGroup.c_str()); // notify the user of the intended group retrieved from file
			} else { // if the intended group name from the provided file was recognized
				trajectory_poses.intendedGroup = intendedGroup; // retrieve the inteded group name
				if (trajectory_poses.groupName.compare(trajectory_poses.intendedGroup) != 0) { // if the group name and intended group name do not match
					ROS_WARN("The provided group is not the same as the group that the provided file was inteded for."); // notify the user that the group name and intended group name do not match
					ROS_WARN("The path planner and execution may not work as intended."); // notify the user that the planner and exeution of the plans may not work as intended
					ROS_WARN("Provided group: %s",trajectory_poses.groupName.c_str()); // notify the user of the provided group name
					ROS_WARN("Intended group: %s",trajectory_poses.intendedGroup.c_str()); // notify the user of the indeded group name
				}
			}
		} else { // if the provided file is empty
			ROS_ERROR("The provided file is empty."); // notify the user that the provided file is empty
			groupName_index = -1; // indicate that an error occurred when opening up the file
		}

		// Get Remaining Data from Provided File
		while (std::getline(textFile,line)) { // while there is still a new line to take from
			// Create String Steamer and Get Command for Current Trajectory Point
			std::istringstream currentLine(line); // create a string steam element for delimiting data by spaces
			currentLine >> command; // get command name for current trajectory point

			// Get Group Names and Poses for Current Trajectory Point
			currentLine >> groupName_1; //  get the command and the group name for the first group
			currentLine >> pose_1.position.x >> pose_1.position.y >> pose_1.position.z; // get position for current pose from first group
			currentLine >> pose_1.orientation.x >> pose_1.orientation.y >> pose_1.orientation.z >> pose_1.orientation.w; // get orientation for current pose from first group

			currentLine >> groupName_2; //  get the group name for the second group
			currentLine >> pose_2.position.x >> pose_2.position.y >> pose_2.position.z; // get position for current pose from second group
			currentLine >> pose_2.orientation.x >> pose_2.orientation.y >> pose_2.orientation.z >> pose_2.orientation.w; // get orientation for current pose from second group

			/* ======================================================================================== */
			/* NEED TO ADD CHECK TO MAKE SURE THE INPUT FILE HAS CORRECT AMOUNT OF INPUTS FOR EACH LINE */
			/* ======================================================================================== */

			// Check Group Names in Input File to Make Sure They Are Expected
			if (lineIndex == 0) { // if this is the first time running through the while loop
				if (groupName_index == 3) { // if the user would like to retrieve data for both arms
					if (!(((groupName_1.compare("right_arm") == 0) || (groupName_2.compare("right_arm") == 0)) && ((groupName_1.compare("left_arm") == 0) || (groupName_2.compare("left_arm") == 0)))) { // if one of the group names from the provided file is not regonized
						ROS_ERROR("One of the group names within the provided input file is not recognized."); // notify the user that one of the group names is not recognized
						ROS_WARN("Recognized groups: left_arm, right_arm"); // notify the user of the allowed group names
						ROS_WARN("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str()); // notify the user of the group names retrieved from the provided file
						groupName_index = -1; // indicate that the provided group is not recognized
						break; // break from the loop due to error
					}
				} else if (groupName_index == 2) { // if the user would like to only retrieve data for the right arm
					if ((groupName_1.compare("right_arm") != 0) && (groupName_2.compare("right_arm") != 0)) { // if none of the group names from the file indicates joints for the right arm
						ROS_ERROR("One of the group names within the provided input file is not recognized."); // notify the user that one of the group names is not recognized
						ROS_WARN("Recognized groups: left_arm, right_arm"); // notify the user of the allowed group names
						ROS_WARN("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str()); // notify the user of the group names retrieved from the provided file
						groupName_index = -1; // indicate that the provided group is not recognized
						break; // break from the loop due to error
					}
				} else if (groupName_index == 1) { // if the user would like to only retrieve data for the left arm
					if ((groupName_1.compare("left_arm") != 0) && (groupName_2.compare("left_arm") != 0)) { // if none of the group names from the file indicates joints for the left arm
						ROS_ERROR("One of the group names within the provided input file is not recognized."); // notify the user that one of the group names is not recognized
						ROS_WARN("Recognized groups: left_arm, right_arm"); // notify the user of the allowed group names
						ROS_WARN("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str()); // notify the user of the group names retrieved from the provided file
						groupName_index = -1; // indicate that the provided group is not recognized
						break; // break from the loop due to error
					}
				} else if (groupName_index == -1) { // if the provided group is already not recognized
					break; // break from the loop due to error
				}
			}

			// Retrieve the Current Pose into the Trajectory Structure
			if (groupName_index == 1) { // if the user would like to only retrieve data for the left arm
				if (groupName_1.compare("left_arm") == 0) { // if the first group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the left arm
				} else { // if the second group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the left arm
				}
			} else if (groupName_index == 2) { // if the user would like to only retrieve data for the right arm
				if (groupName_1.compare("right_arm") == 0) { // if the first group from the file is for the right arm
					trajectory_poses.pose_right.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the right arm
				} else { // if the second group from the file is for the right arm
					trajectory_poses.pose_right.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the right arm
				}
			} else if (groupName_index == 3) { // if the user would like to retrieve data for the both arma
				if (groupName_1.compare("left_arm") == 0) { // if the first group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the left arm
					trajectory_poses.pose_right.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the right arm
				} else { // if the second group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the left arm
					trajectory_poses.pose_right.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the right arm
				}
			}

			lineIndex++; // increment current line counter
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s",inputFile.c_str()); // notify user of the supplied file
	}

	if (groupName_index == -1) { // if the group was not regonized or there was an issue with the group names in the provided file
		trajectory_poses.groupName = ""; // reset the group name to an empty set
		ROS_ERROR("There was an error retriving file data. Please fix any issues stated above."); // notify the user of the error
	} else { // if there were no issues with storing the data from the provided file
		if (groupName_index == 1) { // if the user would like to only retrieve data for the left arm
			trajectory_poses.totalPoints = trajectory_poses.pose_left.size(); // set the size of the points to the size of the left pose within the pose structure
		} else { // if ther user would like to only retrieve data for the right arm or retrieve data for both arms
			trajectory_poses.totalPoints = trajectory_poses.pose_right.size(); // set the size of the points to the size of the right pose within the pose structure
		}
		ROS_INFO("Retrieved %d points from input file.",trajectory_poses.totalPoints); // notify the user of the total points that were retrieved from the provided file
	}

	return trajectory_poses; // return the pose structure
}

/* -----------------------------------------------
   ---------- PATH GENERATOR FUNCTIONS -----------
   ----------------------------------------------- */
planner generatePlans(planningInterface::MoveGroup& group, trajectoryJoints& joint_trajectory) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-22
	PURPOSE: Generate plans from input file for provided group

	INPUT(S):
		> group - group to use for planning trajectories
		> joint_trajectory - structure containing the joint values for a trajectory
	OUTPUT(S):
		< planner - planner structure, created to ensure proper execution is performed

	DEPENDENCIES: None

	NOTE: Should the function be stopped if the planner fails for a given trajectory point? | DATE: 2016-06-28
*/
	// Ensure Projectory is Meant For Provided Group
	if (joint_trajectory.groupName.compare(group.getName()) != 0) { // if the given group does not have the same name as the name within the planner object
		ROS_ERROR("Provided joint trajectory is not meant for the provided group."); // notify the user that the provided planner structure is not for the provided group
		ROS_WARN("Joint trajectory group: %s",joint_trajectory.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_WARN("Provided group: %s",group.getName().c_str()); // notify user of the name of the supplied group
		planner plan;
		return plan;
	}

	// Initialize Variables
	planner plan; // structure of type planner
	plan.groupName = group.getName(); // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories
	int planIndex = 0; // counter to indicate the current plan

	// Notfify User that the Planner is About to Start
	ROS_INFO("Starting planner."); // notify user that the planner is about to start

	for (int plans = 0; plans < joint_trajectory.totalPoints; plans++) {
		// Set Start State Planner Parameter for Provided Group
		if (planIndex == 0) { // if first time running through the while loop
			group.setStartStateToCurrentState(); // set the start location to the current location of the robot
		} else { // if it is not the first time running through the while loop
			moveitCore::RobotState state(group.getRobotModel()); // create state structure using the robots current state
			moveitCore::jointTrajPointToRobotState(plan.plans[planIndex-1].trajectory_.joint_trajectory, (plan.plans[planIndex-1].trajectory_.joint_trajectory.points.size()-1), state); // update the state structure to the given trajectory point
			group.setStartState(state); // set the start state as the newly generated state
		}

		// Plan Trajectory
		group.setJointValueTarget(joint_trajectory.joints[planIndex]); // set the next target for the provided group
		planningInterface::MoveGroup::Plan currentPlan; // initialize the current plan variable
		plan.success = group.plan(currentPlan); // create a path plan for current trajectory point

		// Add Planned Trajectory to Planner Vector if Plan Was Successful
		if (plan.success == true) { // if the planning was successful
			planIndex++; // increment the plan counter
			plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
			ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,joint_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
		} else { // if the planning was not successful
			ROS_WARN("Plan failed for trajectory point: %d of %d",plans+1,joint_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
		}
	}

	// Notfiy User of Planning Success Totals
	plan.success = true; // set plan to success
	plan.totalPlans = plan.plans.size(); // store the total plans within the planner
	ROS_INFO("Finished planning. Total plans created: %d of %d",plan.totalPlans,joint_trajectory.totalPoints); // notify user that the planner has finished
	return plan; // return the planner success flag and the vector of plans
}

planner generatePlans(planningInterface::MoveGroup& group, trajectoryPoses& pose_trajectory) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-07-01
	PURPOSE: Generate plans from input file for provided group

	INPUT(S):
		> group - group to use for planning trajectories
		> pose_trajectory - structure containing the poses for a trajectory
	OUTPUT(S):
		< planner - planner structure, created to ensure proper execution is performed

	DEPENDENCIES: None

	NOTE: Should the function be stopped if the planner fails for a given trajectory point? | DATE: 2016-06-28
*/
	// Ensure Projectory is Meant For Provided Group
	if (pose_trajectory.groupName.compare(group.getName()) != 0) { // if the given group does not have the same name as the name within the planner object
		ROS_ERROR("Provided pose trajectory is not meant for the provided group."); // notify the user that the provided planner structure is not for the provided group
		ROS_WARN("Pose trajectory group: %s",pose_trajectory.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_WARN("Provided group: %s",group.getName().c_str()); // notify user of the name of the supplied group
		planner plan;
		return plan;
	}

	// Initialize Variables
	planner plan; // structure of type planner
	plan.groupName = group.getName(); // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories
	int planIndex = 0; // counter to indicate the current plan

	// Notfify User that the Planner is About to Start
	ROS_INFO("Starting planner."); // notify user that the planner is about to start

	if (plan.groupName.compare("left_arm") == 0) {
		for (int plans = 0; plans < pose_trajectory.totalPoints; plans++) {
			// Set Start State Planner Parameter for Provided Group
			if (planIndex == 0) { // if first time running through the while loop
				group.setStartStateToCurrentState(); // set the start location to the current location of the robot
			} else { // if it is not the first time running through the while loop
				moveitCore::RobotState state(group.getRobotModel()); // create state structure using the robots current state
				moveitCore::jointTrajPointToRobotState(plan.plans[planIndex-1].trajectory_.joint_trajectory, (plan.plans[planIndex-1].trajectory_.joint_trajectory.points.size()-1), state); // update the state structure to the given trajectory point
				group.setStartState(state); // set the start state as the newly generated state
			}

			// Plan Trajectory
			group.setPoseTarget(pose_trajectory.pose_left[planIndex]); // set the next target for the provided group
			planningInterface::MoveGroup::Plan currentPlan; // initialize the current plan variable
			plan.success = group.plan(currentPlan); // create a path plan for current trajectory point

			// Add Planned Trajectory to Planner Vector if Plan Was Successful
			if (plan.success == true) { // if the planning was successful
				planIndex++; // increment the plan counter
				plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
				ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
			} else { // if the planning was not successful
				ROS_WARN("Plan failed for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			}
		}
	} else if (plan.groupName.compare("right_arm") == 0) {
		for (int plans = 0; plans < pose_trajectory.totalPoints; plans++) {
			// Set Start State Planner Parameter for Provided Group
			if (planIndex == 0) { // if first time running through the while loop
				group.setStartStateToCurrentState(); // set the start location to the current location of the robot
			} else { // if it is not the first time running through the while loop
				moveitCore::RobotState state(group.getRobotModel()); // create state structure using the robots current state
				moveitCore::jointTrajPointToRobotState(plan.plans[planIndex-1].trajectory_.joint_trajectory, (plan.plans[planIndex-1].trajectory_.joint_trajectory.points.size()-1), state); // update the state structure to the given trajectory point
				group.setStartState(state); // set the start state as the newly generated state
			}

			// Plan Trajectory
			group.setPoseTarget(pose_trajectory.pose_right[planIndex]); // set the next target for the provided group
			planningInterface::MoveGroup::Plan currentPlan; // initialize the current plan variable
			plan.success = group.plan(currentPlan); // create a path plan for current trajectory point

			// Add Planned Trajectory to Planner Vector if Plan Was Successful
			if (plan.success == true) { // if the planning was successful
				planIndex++; // increment the plan counter
				plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
				ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
			} else { // if the planning was not successful
				ROS_WARN("Plan failed for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			}
		}
	}

	// Notfiy User of Planning Success Totals
	plan.success = true; // set plan to success
	plan.totalPlans = plan.plans.size(); // store the total plans within the planner
	ROS_INFO("Finished planning. Total plans created: %d of %d",plan.totalPlans,pose_trajectory.totalPoints); // notify user that the planner has finished
	return plan; // return the planner success flag and the vector of plans
}

planner generatePlans(planningInterface::MoveGroup& left_arm, planningInterface::MoveGroup& right_arm, planningInterface::MoveGroup& both_arms, trajectoryPoses& pose_trajectory) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-22
	PURPOSE: Generate plans from input file for provided group

	INPUT(S):
		> left_arm - left arm move group
		> right_arm - right arm move group
		> joint_trajectory - structure containing the joint values for a trajectory
	OUTPUT(S):
		< planner - planner structure, created to ensure proper execution is performed

	DEPENDENCIES: None

	NOTE: Should the function be stopped if the planner fails for a given trajectory point? | DATE: 2016-06-28
*/
	// Ensure Projectory is Meant For Provided Group
	if (pose_trajectory.groupName.compare("both_arms") != 0) { // if the given group does not have the same name as the name within the planner object
		ROS_ERROR("Provided joint trajectory is not meant for the provided group."); // notify the user that the provided planner structure is not for the provided group
		ROS_WARN("Intended pose trajectory group: %s",pose_trajectory.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_WARN("This function will use poses for group: both_arms"); // notify user of the name of the group that the trajectory will be used for in this function
		planner plan;
		return plan;
	}

	// Ensure Groups Are Correctly Provided
	if ((left_arm.getName().compare("left_arm") != 0) || (right_arm.getName().compare("right_arm") != 0)) {
		ROS_ERROR("Provided group are either provided in the wrong order or are not expected.");
		ROS_WARN("Expected groups (in order): left_arm, right_arm");
		ROS_WARN("Provided groups (in order): %s, %s",left_arm.getName().c_str(),right_arm.getName().c_str());
	}

	// Initialize Variables
	planner plan; // structure of type planner
	plan.groupName = "both_arms"; // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories
	int planIndex = 0; // counter to indicate the current plan

	planningInterface::MoveGroup::Plan currentPlan;
	planningInterface::MoveGroup::Plan currentPlan_left; // initialize the current plan variable for left arm
	planningInterface::MoveGroup::Plan currentPlan_right; // initialize the current plan variable for right arm
	std::vector<planningInterface::MoveGroup::Plan> previousPlans(3);
	planningInterface::MoveGroup::Plan emptyPlan; // initialize an empty plan
	bool success_left  = false;
	bool success_right = false;

	// Notfify User that the Planner is About to Start
	ROS_INFO("Starting planner."); // notify user that the planner is about to start

	for (int plans = 0; plans < pose_trajectory.totalPoints; plans++) {
		// Set Start State Planner Parameter for Provided Group
		if (planIndex == 0) { // if first time running through the while loop
			left_arm.setStartStateToCurrentState(); // set the start location to the current location of the robot for the left arm
			right_arm.setStartStateToCurrentState(); // set the start location to the current location of the robot for the right arm
			both_arms.setStartStateToCurrentState(); // set the start location to the current location of the robot for both arms
		} else { // if it is not the first time running through the while loop
			if (!(success_left)) { currentPlan_left = previousPlans[0]; }
			if (!(success_right)) { currentPlan_right = previousPlans[1]; }
			if (!(plan.success)) { currentPlan = previousPlans[2]; }

			moveitCore::RobotState state_left(left_arm.getRobotModel()); // create state structure using the robots current state for the left arm
			moveitCore::jointTrajPointToRobotState(currentPlan_left.trajectory_.joint_trajectory, (currentPlan_left.trajectory_.joint_trajectory.points.size()-1), state_left); // update the state structure to the given trajectory point for the left arm
			left_arm.setStartState(state_left); // set the start state as the newly generated state for the left arm

			moveitCore::RobotState state_right(right_arm.getRobotModel()); // create state structure using the robots current state for the left arm
			moveitCore::jointTrajPointToRobotState(currentPlan_right.trajectory_.joint_trajectory, (currentPlan_right.trajectory_.joint_trajectory.points.size()-1), state_right); // update the state structure to the given trajectory point for the left arm
			right_arm.setStartState(state_right); // set the start state as the newly generated state for the left arm

			moveitCore::RobotState state_both(both_arms.getRobotModel()); // create state structure using the robots current state for the left arm
			moveitCore::jointTrajPointToRobotState(currentPlan.trajectory_.joint_trajectory, (currentPlan.trajectory_.joint_trajectory.points.size()-1), state_both); // update the state structure to the given trajectory point for the left arm
			both_arms.setStartState(state_both); // set the start state as the newly generated state for the left arm
		}

		// Plan Trajectories for Both Arms
		left_arm.setPoseTarget(pose_trajectory.pose_left[planIndex]); // set the next target for the left arm
		previousPlans[0] = currentPlan_left;
		currentPlan_left = emptyPlan; // reset the current plan variable for left arm
		success_left = left_arm.plan(currentPlan_left); // create a path plan for current trajectory point for left arm

		right_arm.setPoseTarget(pose_trajectory.pose_right[planIndex]); // set the next target for right arm
		previousPlans[1] = currentPlan_right;
		currentPlan_right = emptyPlan; // reset the current plan variable for right arm
		success_right = right_arm.plan(currentPlan_right); // create a path plan for current trajectory point for right arm

		if ((success_left) && (success_right)) {
			std::vector<double> lastTrajectoryPoint_left  = currentPlan_left.trajectory_.joint_trajectory.points.back().positions;
			std::vector<double> lastTrajectoryPoint_right = currentPlan_right.trajectory_.joint_trajectory.points.back().positions;
			std::vector<double> lastTrajectoryPoint = lastTrajectoryPoint_left;
			lastTrajectoryPoint.insert(lastTrajectoryPoint.end(),lastTrajectoryPoint_right.begin(),lastTrajectoryPoint_right.end());

			both_arms.setJointValueTarget(lastTrajectoryPoint);
			previousPlans[2] = currentPlan;
			currentPlan = emptyPlan;
			plan.success = both_arms.plan(currentPlan);

			if (plan.success) {
				planIndex++; // increment the plan counter
				plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
				ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
			} else { // if the planning was not successful
				ROS_WARN("Plan failed for both arms for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			}
		} else { // if the planning was not successful
			if ((!(success_left)) && (!(success_right))) {
				ROS_WARN("Plan failed for both arms for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			} else if (!(success_left)) {
				ROS_WARN("Plan failed for left arm for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			} else if (!(success_right)) {
				ROS_WARN("Plan failed for right arm for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			}
		}

		// if ((success_left) && (success_right)) {
		// 	int totalJoints_right = currentPlan_right.trajectory_.joint_trajectory.points[0].positions.size();
		// 	int planLeft_size  = currentPlan_left.trajectory_.joint_trajectory.points.size();
		// 	int planRight_size = currentPlan_right.trajectory_.joint_trajectory.points.size();
		// 	planningInterface::MoveGroup::Plan currentPlan; // variable used to combine the left arm and right arm plans
		// 	trajectory_msgs::JointTrajectoryPoint jointValues;
		// 	std::vector<std::string> jointNames;

		// 	jointNames = currentPlan_left.trajectory_.joint_trajectory.joint_names;
		// 	jointNames.insert(jointNames.end(),currentPlan_right.trajectory_.joint_trajectory.joint_names.begin(),currentPlan_right.trajectory_.joint_trajectory.joint_names.end());
		// 	currentPlan.trajectory_.joint_trajectory.joint_names = jointNames;

		// 	int largestPlan = std::max(planLeft_size,planRight_size);
		// 	for (int point = 0; point < largestPlan; point++) {
		// 		if (point < planLeft_size) {
		// 			jointValues = currentPlan_left.trajectory_.joint_trajectory.points[point];
		// 		} else {
		// 			jointValues = currentPlan_left.trajectory_.joint_trajectory.points[planLeft_size-1];
		// 		}

		// 		if (point < planRight_size) {
		// 			for (int joint = 0; joint < totalJoints_right; joint++) { 
		// 				jointValues.positions.push_back(currentPlan_right.trajectory_.joint_trajectory.points[point].positions[joint]);
		// 				jointValues.velocities.push_back(currentPlan_right.trajectory_.joint_trajectory.points[point].velocities[joint]);
		// 				jointValues.accelerations.push_back(currentPlan_right.trajectory_.joint_trajectory.points[point].accelerations[joint]);
		// 				//jointValues.effort.push_back(currentPlan_right.trajectory_.joint_trajectory.points[point].effort[joint]);
		// 				/* NOTE: Effort not added due to recurring seg faults | DATE: 2016-07-01 */
		// 			}
		// 		} else {
		// 			for (int joint = 0; joint < totalJoints_right; joint++) { 
		// 				jointValues.positions.push_back(currentPlan_right.trajectory_.joint_trajectory.points[planRight_size-1].positions[joint]);
		// 				jointValues.velocities.push_back(currentPlan_right.trajectory_.joint_trajectory.points[planRight_size-1].velocities[joint]);
		// 				jointValues.accelerations.push_back(currentPlan_right.trajectory_.joint_trajectory.points[planRight_size-1].accelerations[joint]);
		// 				//jointValues.effort.push_back(currentPlan_right.trajectory_.joint_trajectory.points[planRight_size].effort[joint]);
		// 				/* NOTE: Effort not added due to recurring seg faults | DATE: 2016-07-01 */
		// 			}
		// 		}

		// 		jointValues.time_from_start = currentPlan_left.trajectory_.joint_trajectory.points[point].time_from_start; // use the left arm duration since start trajectory point
		// 		/* NOTE: Should figure out if pose to joint conversion through planning should be replanned to ensure proper velocities and accelerations | DATE: 2016-07-01 */
		// 		currentPlan.trajectory_.joint_trajectory.points.push_back(jointValues);
		// 	}
			
		// 	// Add Newly Create Plan to Planner Structure
		// 	planIndex++;
		// 	plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
		// 	ROS_INFO("Plan created for trajectory point: %d of %d",planIndex,pose_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
		// } else { // if the planning was not successful
		// 	if ((!(success_left)) && (!(success_right))) {
		// 		ROS_WARN("Plan failed for both arms for trajectory point: %d of %d",planIndex+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
		// 	} else if (!(success_left)) {
		// 		ROS_WARN("Plan failed for left arm for trajectory point: %d of %d",planIndex+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
		// 	} else if (!(success_right)) {
		// 		ROS_WARN("Plan failed for right arm for trajectory point: %d of %d",planIndex+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
		// 	}
		// }
	}

	// Notfiy User of Planning Success Totals
	plan.success = true; // set plan to success
	plan.totalPlans = plan.plans.size(); // store the total plans within the planner
	ROS_INFO("Finished planning. Total plans created: %d of %d",plan.totalPlans,pose_trajectory.totalPoints); // notify user that the planner has finished
	return plan; // return the planner success flag and the vector of plans
}

bool executePlans(planningInterface::MoveGroup& group, planner& plan) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-22
	PURPOSE: Execute provided plans for provided group

	INPUT(S):
		> group - group to use to execute planned trajectories
		> plans - vector of plans that have been previously generated
	OUTPUT(S):
		< exectuteSuccess_flag - bool to indicate if the execution of all plans was successful or not successful

	DEPENDENCIES: The generatePaths() function must be run and return succesful before execution this function
*/
	// Initialize Variables
	bool executeSuccess_flag;// flag to indicate the success of the execution for provided plans

	// Ensure Proper Group is Used for Provided Planner
	if (plan.groupName.compare(group.getName()) != 0) { // if the given group does not have the same name as the name within the planner object
		ROS_WARN("Provided planner is not meant for the provided group."); // notify the user that the provided planner structure is not for the provided group
		ROS_INFO("Planner Group: %s",plan.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_INFO("Provided Group: %s",group.getName().c_str()); // notify user of the name of the supplied group
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else if (plan.success == 0) {
		ROS_WARN("Provided planner did not successfully create plans for one or all trajectory points for supplid text file.");
		ROS_INFO("Please (re-)run the generatePaths() function before executing paths again.");
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else {
		// Notify User that the Plan Exection is About to be Started
		ROS_INFO("Executing trajectories."); // notify user that the execution of the plans is about to start

		// Execute Given Plans
		for (int currentPlan = 0; currentPlan < plan.totalPlans; currentPlan++) { // for all plans in plans vector
			executeSuccess_flag = group.execute(plan.plans[currentPlan]); // execute trajectory for the current plan
			if (executeSuccess_flag == true) { // if the execution was successful
				ROS_INFO("Executed trajectory: %d",currentPlan+1); // notify user of the plan number that was executed successfully
			} else { // if the execution was not successful
				ROS_WARN("Failed to execute trajectory for plan: %d",currentPlan+1); // notify user of the plan number that was not executed successfully
				break; // break from the for loop
			}
		}
	}

	// Notify User if the Execition Was Successful
	if (executeSuccess_flag == true) { // if the execution was successful
		ROS_INFO("Finished executing trajectories."); // notify the user that the execution has finished
	} else { // if the execution was not successful
		ROS_ERROR("Execution error occurred, exiting execution function."); // notify the user that the execution failed
	}

	return executeSuccess_flag; // return the execution success flag
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
    group.setStartStateToCurrentState(); // ensure start state is set to the robots current position
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


