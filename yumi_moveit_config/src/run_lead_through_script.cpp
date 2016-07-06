#include <sstream> // include string stream functionality
#include <fstream> // include file stream functionality
#include <string> // include string functionality
#include <stack> // include stack functionality
#include <ctime> // include timing functionality

#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h> // include pose for move group functionality
#include <moveit/move_group_interface/move_group.h> // include move group functionality
#include <moveit/robot_state/conversions.h> // include robot state conversion functionality
#include <moveit_msgs/RobotTrajectory.h> // include robot trajectory
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h> // include node functionality
#include <rosbag/bag.h> // allow rosbag functionality
#include <rosbag/view.h>

#include <yumi_moveit_config/PrePlan.h> // include custom message for planner

// Define Global Constants
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)
const std::string moveitConfigDirectory = "/home/yumi/yumi_ws/src/yumi/yumi_moveit_config/"; // full path to folder where trajectory text files should be stored

// Namespace Commands and Variables
using namespace ros; // use namespace of ros
namespace planningInterface = moveit::planning_interface; // define commonly used namespace for planning interface
namespace moveitCore = moveit::core; // define commonly used namespace for planning interface

// Define Structures
struct planner {
	std::string groupName; // include group name
	std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to retrieve planned paths for trajectory points
	bool success; // include boolean to retrieve whether the planner was successful or not
	int totalPlans; // include count of total plans
};

struct trajectoryJoints {
	std::string groupName; // include group name
	std::string intendedGroup; // include name of the intended group for data within structure
	std::vector<std::vector<double>> joints; // contain trajectory of joints
	int totalJoints; // include count of total joints
	int totalPoints; // include count of total trajectory points
};

struct trajectoryPoses {
	std::string groupName; // include group name
	std::string intendedGroup; // include name of the intended group for data within structure
	std::vector<geometry_msgs::Pose> pose_left; // include trajectory of poses for the left arm
	std::vector<geometry_msgs::Pose> pose_right; // include trajectory of poses for the right arm
	bool usingGripper_left = false; // include boolean to indicate if gripper data was provided for the left arm
	bool usingGripper_right = false; // include bollean to indicate if gripper data was provided for the right arm
	std::vector<double> gripperPos_left; // include gripper values for the trajectory for the left arm
	std::vector<double> gripperPos_right; // include gripper values for the trajectory for the right arm
	int totalPoints; // include count of total trajectory points
};

// Basic Functions
// REFERENCE: stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
std::stack<clock_t> tictoc_stack; // create a stack of type clock

void tic() { // similar to MATLAB version of tic
    tictoc_stack.push(clock()); // add the current clock to the top of the stack
}

double toc() { // similar to MATLAB version of toc
	double totalTime = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC; // get total time elapsed
    tictoc_stack.pop(); // remove top of stack
    return totalTime; // return total time elapsed
}

// Function Prototypes
void createBag(std::string, std::string, planner&);
planner retrieveBag(std::string, std::string);
std::string getFileDataType(std::string);
trajectoryJoints getTrajectoryJoints(planningInterface::MoveGroup&, std::string);
trajectoryPoses getTrajectoryPoses(planningInterface::MoveGroup&, std::string);

planner generatePlans(planningInterface::MoveGroup&, trajectoryJoints&, bool debug = false);
planner generatePlans(planningInterface::MoveGroup&, trajectoryPoses&, bool debug  = false);
planner generatePlans(planningInterface::MoveGroup&, planningInterface::MoveGroup&, planningInterface::MoveGroup&, trajectoryPoses&, bool debug = false);
bool executePlans(planningInterface::MoveGroup&, planner&, bool debug = false);

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
		ROS_ERROR("An input file name needs to be executed as an additional argument."); // notify user that no output file name was supplied 
		shutdown(); // showdown the node
		return 1; // exit due to error
	}
	std::stringstream fullPath; // initialize variable to concatenate the full path
	fullPath << moveitConfigDirectory << "paths/" << argv[1] << ".txt"; // concatenate full path
	std::string inputFile = fullPath.str(); // retrieve the full path

	// Initialize ROS
	init(argc,argv,"run_lead_through"); // initialize ROS node
	NodeHandle nodeHandle; // initialize node handle

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

	// Get Trajectory From File and Create Plans
	planner plans;
	plans = retrieveBag("test", "test");
	// std::string dataType = getFileDataType(inputFile);
	// if (dataType.compare("joints") == 0) {
	// 	trajectoryJoints jointTrajectory = getTrajectoryJoints(both_arms, inputFile);
	// 	if (jointTrajectory.groupName.compare("") != 0) {
	// 		plans = generatePlans(both_arms, jointTrajectory);
	// 		createBag("test","test",plans);
	// 	}
	// } else if (dataType.compare("poses") == 0) {
	// 	trajectoryPoses poseTrajectory = getTrajectoryPoses(both_arms, inputFile);
	// 	if (poseTrajectory.groupName.compare("") != 0) {
	// 		plans = generatePlans(left_arm, right_arm, both_arms, poseTrajectory);
	// 		createBag("test", "test", plans);
	// 	}
	// }

	// Execute Plans
	bool success = executePlans(both_arms, plans);

	return 0;
}

/* -----------------------------------------------
   ------- FILE READNG/WRITING FUNCTIONS ---------
   ----------------------------------------------- */
void createBag(std::string bagName, std::string topicName, planner& plan) {

	/* =============================================== */
	/* NEED TO ENSURE .bag IS NOT INCLUDED IN BAG NAME */
	/* =============================================== */
	// Notify User of Storing a Bag
	ROS_INFO("Creating ROS bag for group: %s", plan.groupName.c_str());
	
	// Create Bag with Provided Name
	bagName = moveitConfigDirectory + "bags/" + bagName + ".bag";
	rosbag::Bag bag(bagName, rosbag::bagmode::Write);

	// Store Planner Data into Bag Message
	yumi_moveit_config::PrePlan prePlan_msg;
	prePlan_msg.group_name = plan.groupName;
	prePlan_msg.start_state = plan.plans[0].start_state_;
	prePlan_msg.total_trajectories = plan.totalPlans;
	for (int plans = 0; plans < plan.totalPlans; plans++) {
		prePlan_msg.trajectory.push_back(plan.plans[plans].trajectory_);
	}

	// Write to Bag
	bag.write(topicName, ros::Time::now(), prePlan_msg);
	bag.close();

	// Notify User of Successful Bag Writing
	ROS_INFO("Successfully created ROS bag.");
	ROS_INFO("Location: %s", bagName.c_str());
	ROS_INFO("Topic name: %s", topicName.c_str());
}

planner retrieveBag(std::string bagName, std::string topicName) {

	// Notify User of Retrieving a Bag
	bagName = moveitConfigDirectory + "bags/" + bagName + ".bag";
	ROS_INFO("Retrieving ROS bag at location: %s", bagName.c_str());

	// Get Bag with Specified Topic
	rosbag::Bag bag(bagName, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topicName));

	// Store Bag Elements
	yumi_moveit_config::PrePlan::ConstPtr prePlan_msg;
	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		prePlan_msg = m.instantiate<yumi_moveit_config::PrePlan>();
	}

	// Get Elements from Retrieved Bag
	planner plan;
	plan.groupName = prePlan_msg->group_name;
	plan.totalPlans = prePlan_msg->total_trajectories;
	std::vector<moveit_msgs::RobotTrajectory> trajectory = prePlan_msg->trajectory;

	// Store Trajectories Into Planner Structure
	for (int plans = 0; plans < plan.totalPlans; plans++) {
		planningInterface::MoveGroup::Plan currentPlan;
		currentPlan.trajectory_ = trajectory[plans];
		plan.plans.push_back(currentPlan);
	}
	plan.plans[0].start_state_ = prePlan_msg->start_state;

	// Close the Bag and Notify the User
	bag.close();
	ROS_INFO("Successfully retrieved ROS bag.");

	return plan; // return the plan
}

std::string getFileDataType(std::string inputFile) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-01
	PURPOSE: Retrive the data type from the provided input file

	INPUT(S):
		> inputFile - file name that contains the desired 
	OUTPUT(S):
		< dataType - Data type retrived from the provided input file

	DEPENDENCIES: None
*/
	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string dataType; // variable to store the data type from the provided input file
	bool success = true; // flag to indicate if any error occurred
 
	ROS_INFO("Getting file data type."); // notify the user that the file data type for the provided input file is being retrived

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
				ROS_WARN("Provided group: %s", dataType.c_str()); // notify the user of the intended group retrieved from file
				success = false; // indicate that there was an error
			}
		} else { // if the provided file is empty
			ROS_ERROR("The provided file is empty."); // notify the user that the provided file is empty
			success = false; // indicate that there was an error
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s",inputFile.c_str()); // notify user of the supplied file
		success = false; // indicate that there was an error
	}

	// Return Data Type if Retrieved
	if (success) { // if there were no errors with getting the file data type variable from the provided file
		ROS_INFO("File contains data of type: %s", dataType.c_str()); // notify the user of the data type from the provided input file
		return dataType; // return the data type from the file
	} else { // if there were erros with getting the file data type variable from the provided file
		ROS_ERROR("File data type was not able to be retrived."); // notify the user that there was an error with getting the data type from the provided input file
		return ""; // return an empty string
	}
}

trajectoryJoints getTrajectoryJoints(planningInterface::MoveGroup& group, std::string inputFile) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-06-28
	PURPOSE: Retrive joint values from a provided file for the provided group

	INPUT(S):
		> group - group data to retrieve from file
		> inputFile - file name that contains the joint data for stored robot states
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

	std::string groupName_1, groupName_2; // variables to retrieve the first and second group name
	std::vector<double>::size_type totalJoints_1, totalJoints_2; // variables to retrieve the joint values for the first and second group

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
	tic(); // start timer

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

			// Check Group Names in Input File to Make Sure They Are Expected
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

			// Retrieve the Current Joint Values into the Trajectory Structure
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

	// Check If There Were Errors During Function Execution and Notify User
	if (groupName_index == -1) { // if an error occured with the group name or retrieving data from the provided file
		trajectory_joints.totalJoints = 0; // reset the total joints in the joint trajectory structure
		trajectory_joints.groupName = ""; // reset the group name in the joint trajectory structure
		ROS_ERROR("There was an error retriving file data. Please fix any issues stated above."); // notify the user an error occurred with the file reading or with the provided group
		toc(); // reset clock variable
	} else { // if no errors occurred with the group name or retrieving data from the file
		trajectory_joints.totalPoints = trajectory_joints.joints.size(); // store the total trajectory points to the joint trajectory structure
		ROS_INFO("Retrieved %d points from input file.",trajectory_joints.totalPoints); // notify the user of the total trajectory points
		ROS_INFO("Processing time: %.4f",toc()); // display processing time
	}

	return trajectory_joints; // return the join trajectory structure
}

trajectoryPoses getTrajectoryPoses(planningInterface::MoveGroup& group, std::string inputFile) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-06-28
	PURPOSE: Retrive poses from a provided file for the provided group

	INPUT(S):
		> group - group data to retrieve from file
		> inputFile - file name that contains pose data for stored robot states
	OUTPUT(S):
		< trajectoryPoses - pose trajectory structure containing all poses retrived from provided file

	DEPENDENCIES: None

	INPUT FILE CONVENTION: The input file must have the following convention for each trajectory point:
		- $(Command Name) $(Group 1 Name) $(gripper_exist (true/false)) $(if gripper_exist, gripper_pos) $(pose.position.x) ... $(pose.orientation.w) $(Group 2 Name) ... $(pose.orientation.w)

	NOTE: Currently not doing anything with the joint command from provided input file | DATE: 2016-06-22
*/
	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string command; // variable to retrieve the command for each trajectory point from the input file
	std::string dataType; // variable to retrieve the data type from provided file
	std::string intendedGroup; // variable to retrieve the intended arm for the data in the provided file
	int lineIndex = 0; // counter to indicate the current line

	std::string groupName_1, groupName_2; // variables to retrieve the first and second group name
	std::string gripperExist_1, gripperExist_2; // variables to retrive if the gripper for the first and second group exist
	double gripperPos_1, gripperPos_2; // variables to retrieve the gripper position if it exists
	geometry_msgs::Pose pose_1, pose_2; // variables to retrieve poses from the first and second group

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
	tic();

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
			currentLine >> groupName_1 >> gripperExist_1; //  get the group name and whether the gripper exists for the first group
			if (gripperExist_1.compare("true") == 0) { currentLine >> gripperPos_1; } // get the gripper position for the first group if it exists
			currentLine >> pose_1.position.x >> pose_1.position.y >> pose_1.position.z; // get position for current pose from first group
			currentLine >> pose_1.orientation.x >> pose_1.orientation.y >> pose_1.orientation.z >> pose_1.orientation.w; // get orientation for current pose from first group

			currentLine >> groupName_2 >> gripperExist_2; //  get the group name and whether the gripper exists for the second group
			if (gripperExist_2.compare("true") == 0) { currentLine >> gripperPos_2; } // get the gripper position for the second group if it exists
			currentLine >> pose_2.position.x >> pose_2.position.y >> pose_2.position.z; // get position for current pose from second group
			currentLine >> pose_2.orientation.x >> pose_2.orientation.y >> pose_2.orientation.z >> pose_2.orientation.w; // get orientation for current pose from second group

			/* ======================================================================================== */
			/* NEED TO ADD CHECK TO MAKE SURE THE INPUT FILE HAS CORRECT AMOUNT OF INPUTS FOR EACH LINE */
			/* ======================================================================================== */

			// Check Group Names in Input File to Make Sure They Are Expected and Check If Gripper Exists for Groups
			if (lineIndex == 0) { // if this is the first time running through the while loop
				// Check Group Names in Input File to Make Sure They Are Expected
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

				// Check If Groups From File Have Grippers
				if (gripperExist_1.compare("true") == 0) { // if the first group contains data for the gripper position
					if (groupName_1.compare("left_arm") == 0) { // if the first group provides data for the left arm
						trajectory_poses.usingGripper_left = true; // indicate that the left arm data from the text file contains gripper position data
					} else { // if the first group provides data for the right arm
						trajectory_poses.usingGripper_right = true; // indicate that the right arm data from the text file contains gripper position data
					}
				}
				if (gripperExist_2.compare("true") == 0) { // if the second group contains data for the gripper position
					if (groupName_2.compare("left_arm") == 0) { // if the second group provides data for the left arm
						trajectory_poses.usingGripper_left = true; // indicate that the left arm data from the text file contains gripper position data
					} else { // if the second group provides data for the right arm
						trajectory_poses.usingGripper_right = true; // indicate that the right arm data from the text file contains gripper position data
					}
				}
			}

			// Retrieve the Current Pose into the Trajectory Structure
			if (groupName_index == 1) { // if the user would like to only retrieve data for the left arm
				if (groupName_1.compare("left_arm") == 0) { // if the first group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the left arm
					if (trajectory_poses.usingGripper_left) { // if the gripper exists for the first group
						trajectory_poses.gripperPos_left.push_back(gripperPos_1); // retrieve the gripper position for the first group into the pose structure for the left arm
					}
				} else { // if the second group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the left arm
					if (trajectory_poses.usingGripper_left) { // if the gripper exists for the second group
						trajectory_poses.gripperPos_left.push_back(gripperPos_2); // retrieve the gripper position for the second group into the pose structure for the left arm
					}
				}
			} else if (groupName_index == 2) { // if the user would like to only retrieve data for the right arm
				if (groupName_1.compare("right_arm") == 0) { // if the first group from the file is for the right arm
					trajectory_poses.pose_right.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the right arm
					if (trajectory_poses.usingGripper_right) { // if the gripper exists for the first group
						trajectory_poses.gripperPos_right.push_back(gripperPos_1); // retrieve the gripper position from the first group into the pose structure for the right arm
					}
				} else { // if the second group from the file is for the right arm
					trajectory_poses.pose_right.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the right arm
					if (trajectory_poses.usingGripper_right) { // if the gripper exists for the second group
						trajectory_poses.gripperPos_right.push_back(gripperPos_2); // retrieve the gripper position from the second group into the pose structure for the right arm
					}
				}
			} else if (groupName_index == 3) { // if the user would like to retrieve data for the both arma
				if (groupName_1.compare("left_arm") == 0) { // if the first group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the left arm
					trajectory_poses.pose_right.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the right arm
					if (trajectory_poses.usingGripper_left) { // if the gripper exists for the first group
						trajectory_poses.gripperPos_left.push_back(gripperPos_1); // retrieve the gripper position from the first group into the pose structure for the left arm
					}
					if (trajectory_poses.usingGripper_right) { // if the gripper exists for the second group
						trajectory_poses.gripperPos_right.push_back(gripperPos_2); // retrieve the gripper position from the second group into the pose structure for the right arm
					}
				} else { // if the second group from the file is for the left arm
					trajectory_poses.pose_left.push_back(pose_2); // retrieve the pose from the second group into the pose structure for the left arm
					trajectory_poses.pose_right.push_back(pose_1); // retrieve the pose from the first group into the pose structure for the right arm
					if (trajectory_poses.usingGripper_left) { // if the gripper exists for the second group
						trajectory_poses.gripperPos_left.push_back(gripperPos_2); // retrieve the gripper position from the first group into the pose structure for the left arm
					}
					if (trajectory_poses.usingGripper_right) { // if the gripper exists for the first group
						trajectory_poses.gripperPos_right.push_back(gripperPos_1); // retrieve the gripper position from the second group into the pose structure for the right arm
					}
				}
			}

			lineIndex++; // increment current line counter
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s", inputFile.c_str()); // notify user of the supplied file
	}

	// Check If There Were Errors During Function Execution and Notify User
	if (groupName_index == -1) { // if the group was not regonized or there was an issue with the group names in the provided file
		trajectory_poses.groupName = ""; // reset the group name to an empty set
		ROS_ERROR("There was an error retriving file data. Please fix any issues stated above."); // notify the user of the error
		toc(); // reset clock variable
	} else { // if there were no issues with storing the data from the provided file
		if (groupName_index == 1) { // if the user would like to only retrieve data for the left arm
			trajectory_poses.totalPoints = trajectory_poses.pose_left.size(); // set the size of the points to the size of the left pose within the pose structure
		} else { // if ther user would like to only retrieve data for the right arm or retrieve data for both arms
			trajectory_poses.totalPoints = trajectory_poses.pose_right.size(); // set the size of the points to the size of the right pose within the pose structure
		}
		ROS_INFO("Retrieved %d points from input file.", trajectory_poses.totalPoints); // notify the user of the total points that were retrieved from the provided file
		ROS_INFO("Processing time: %.4f", toc()); // display processing time
	}

	return trajectory_poses; // return the pose structure
}

/* -----------------------------------------------
   ---------- PATH GENERATOR FUNCTIONS -----------
   ----------------------------------------------- */
planner generatePlans(planningInterface::MoveGroup& group, trajectoryJoints& joint_trajectory, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-06-22
	PURPOSE: Generate plans from input file for provided group

	INPUT(S):
		> group - group to use for planning trajectories
		> joint_trajectory - structure containing the joint values for a trajectory
		> debug - indicate if the function should skip trajectory points if the plan for that point fails. true) skip point, false) stop funciton execution
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
		planner plan; // create a plan variable
		plan.success = false; // indicate the planner was unsuccessful
		return plan; // return empty plan
	}

	// Initialize Variables
	planner plan; // structure of type planner
	plan.groupName = group.getName(); // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories
	int planIndex = 0; // counter to indicate the current plan

	planningInterface::MoveGroup::Plan currentPlan; // variable to store the current plan
	planningInterface::MoveGroup::Plan lastSuccessfulPlan; // variable to store the last successful plan

	// Check Optional Variable(s)
	if (debug) { // if this function is being run in debug mode
		ROS_WARN("Planning is being run in debug mode."); // notify the user that this function is being run in debug mode
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Starting planner."); // notify user that the planner is about to start

	for (int plans = 0; plans < joint_trajectory.totalPoints; plans++) {
		// Set Start State Planner Parameter for Provided Group
		if (planIndex == 0) { // if first time running through the while loop
			group.setStartStateToCurrentState(); // set the start location to the current location of the robot
		} else { // if it is not the first time running through the while loop
			if (!(plan.success)) { // if the previous plan was not successful
				currentPlan = lastSuccessfulPlan; // reset the current plan as the last successful plan
			}
			moveitCore::RobotState state(group.getRobotModel()); // create state structure using the robots current state
			moveitCore::jointTrajPointToRobotState(plan.plans[planIndex-1].trajectory_.joint_trajectory, (plan.plans[planIndex-1].trajectory_.joint_trajectory.points.size()-1), state); // update the state structure to the given trajectory point
			group.setStartState(state); // set the start state as the newly generated state
		}

		// Store Previous Plan if it was Successful
		if (plan.success) { // if the plan was successful on the previous iteration
			lastSuccessfulPlan = currentPlan; // store the previous plan
		}

		// Plan Trajectory
		group.setJointValueTarget(joint_trajectory.joints[plans]); // set the next target for the provided group
		planningInterface::MoveGroup::Plan currentPlan; // initialize the current plan variable
		plan.success = group.plan(currentPlan); // create a path plan for current trajectory point

		// Add Planned Trajectory to Planner Vector if Plan Was Successful
		if (plan.success) { // if the planning was successful
			planIndex++; // increment the plan counter
			plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
			ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,joint_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
		} else { // if the planning was not successful
			ROS_WARN("Plan failed for trajectory point: %d of %d",plans+1,joint_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			if (debug) { // if the function is in debug mode
				plan.success = true; // reset the planner success flag to true since in debug mode
				ROS_WARN("Skipping trajectory point and continuing planner (planner is in debug mode)."); // notify the user that the trajectory point is being skipped due to debug mode
			} else { // if the function is not in debug mode
				ROS_WARN("Stopping planner."); // notify the user that the planner is being stopped
				break; // exit planning loop
			}
		}
	}

	// Notfiy User if Planner Succeessful and/or Success Totals
	if (plan.success) { // if the planning was successful or was in debug mode
		plan.totalPlans = plan.plans.size(); // store the total plans within the planner
		ROS_INFO("Finished planning. Total plans created: %d of %d",plan.totalPlans,joint_trajectory.totalPoints); // notify user that the planner has finished
	} else { // if the planning was not successful
		ROS_INFO("Planning unsuccessful. Planning failed for trajectory point: %d of %d",planIndex+1,joint_trajectory.totalPoints); // notify user that the planner has finished
	}
	
	return plan; // return the planner success flag and the vector of plans
}

planner generatePlans(planningInterface::MoveGroup& group, trajectoryPoses& pose_trajectory, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-01
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
		planner plan; // create a plan variable
		plan.success = false; // indicate the planner was unsuccessful
		return plan; // return empty plan
	}

	// Initialize Variables
	planner plan; // structure of type planner
	plan.groupName = group.getName(); // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories
	int planIndex = 0; // counter to indicate the current plan

	planningInterface::MoveGroup::Plan currentPlan; // variable to store the current plan
	planningInterface::MoveGroup::Plan lastSuccessfulPlan; // variable to store the last successful plan

	std::vector<geometry_msgs::Pose> poses;

	// Check Which Arm was Provided
	std::string arm;
	if (plan.groupName.compare("left_arm") == 0) {
		arm  = "left";
		poses = pose_trajectory.pose_left;
	} else if (plan.groupName.compare("right_arm") == 0) {
		arm  = "right";
		poses = pose_trajectory.pose_right;
	} else {
		ROS_WARN("Provided group was not one of the expected groups.");
		ROS_WARN("Expected groups: left_arm, right_arm");
		ROS_WARN("Provided group: %s",plan.groupName.c_str());
		plan.groupName = "";
		return plan; // return empty plan
	}

	// Check Optional Variable(s)
	if (debug) { // if this function is being run in debug mode
		ROS_WARN("Planning is being run in debug mode."); // notify the user that this function is being run in debug mode
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Starting planner for %s arm.",arm.c_str()); // notify user that the planner is about to start

	for (int plans = 0; plans < pose_trajectory.totalPoints; plans++) {
		// Set Start State Planner Parameter for Provided Group
		if (planIndex == 0) { // if first time running through the while loop
			group.setStartStateToCurrentState(); // set the start location to the current location of the robot
		} else { // if it is not the first time running through the while loop
			if (!(plan.success)) { // if the previous plan was not successful
				currentPlan = lastSuccessfulPlan; // reset the current plan as the last successful plan
			}
			moveitCore::RobotState state(group.getRobotModel()); // create state structure using the robots current state
			moveitCore::jointTrajPointToRobotState(currentPlan.trajectory_.joint_trajectory, (currentPlan.trajectory_.joint_trajectory.points.size()-1), state); // update the state structure to the given trajectory point
			group.setStartState(state); // set the start state as the newly generated state
		}

		// Store Previous Plan if it was Successful
		if (plan.success) { // if the plan was successful on the previous iteration
			lastSuccessfulPlan = currentPlan; // store the previous plan
		}

		// Plan Trajectory
		group.setPoseTarget(poses[plans]); // set the next target for the provided group
		planningInterface::MoveGroup::Plan currentPlan; // initialize the current plan variable
		plan.success = group.plan(currentPlan); // create a path plan for current trajectory point

		// Add Planned Trajectory to Planner Vector if Plan Was Successful
		if (plan.success) { // if the planning was successful
			planIndex++; // increment the plan counter
			plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
			ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
		} else { // if the planning was not successful
			ROS_WARN("Plan failed for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			if (debug) { // if the function is in debug mode
				plan.success = true; // reset the planner success flag to true since in debug mode
				ROS_WARN("Skipping trajectory point and continuing planner (planner is in debug mode)."); // notify the user that the trajectory point is being skipped due to debug mode
			} else { // if the function is not in debug mode
				ROS_WARN("Stopping planner."); // notify the user that the planner is being stopped
				break; // exit planning loop
			}
		}
	}

	// Notfiy User if Planner Succeessful and/or Success Totals
	if (plan.success) { // if the planning was successful or was in debug mode
		plan.totalPlans = plan.plans.size(); // store the total plans within the planner
		ROS_INFO("Finished planning. Total plans created: %d of %d",plan.totalPlans,pose_trajectory.totalPoints); // notify user that the planner has finished
	} else { // if the planning was not successful
		ROS_INFO("Planning unsuccessful. Planning failed for trajectory point: %d of %d",planIndex+1,pose_trajectory.totalPoints); // notify user that the planner has finished
	}
	
	return plan; // return the planner success flag and the vector of plans
}

planner generatePlans(planningInterface::MoveGroup& left_arm, planningInterface::MoveGroup& right_arm, planningInterface::MoveGroup& both_arms, trajectoryPoses& pose_trajectory, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-06-22
	PURPOSE: Generate plans from input file for provided group

	INPUT(S):
		> left_arm - left arm move group
		> right_arm - right arm move group
		> joint_trajectory - structure containing the joint values for a trajectory
	OUTPUT(S):
		< planner - planner structure, created to ensure proper execution is performed

	DEPENDENCIES: Assumed the gripper prismatic joint is the last joint in the SRDF for both the left and right arm
	              SRDF group for both_arms must contain all the joints for the left arm first, then the joints for the right arm

	NOTE: Should the function be stopped if the planner fails for a given trajectory point? | DATE: 2016-06-28
*/
	// Ensure Projectory is Meant For Provided Group
	if (pose_trajectory.groupName.compare("both_arms") != 0) { // if the given group does not have the same name as the name within the planner object
		ROS_ERROR("Provided joint trajectory is not meant for the provided group."); // notify the user that the provided planner structure is not for the provided group
		ROS_WARN("Intended pose trajectory group: %s",pose_trajectory.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_WARN("This function will use poses for group: both_arms"); // notify user of the name of the group that the trajectory will be used for in this function
		planner plan; // create a plan variable
		plan.success = false; // indicate the planner was unsuccessful
		return plan; // return empty plan
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
	std::vector<planningInterface::MoveGroup::Plan> lastSuccessfulPlan(3);
	planningInterface::MoveGroup::Plan emptyPlan; // initialize an empty plan
	bool success_left  = false;
	bool success_right = false;

	bool usingGripper_left  = false;
	bool usingGripper_right = false;
	int totalJoints_left  = left_arm.getCurrentJointValues().size();
	int totalJoints_right = right_arm.getCurrentJointValues().size();
	std::string lastJointName_left  = left_arm.getActiveJoints().back(); // get the last active joint name (exclude fixed joints) for the left arm
	std::string lastJointName_right = right_arm.getActiveJoints().back(); // get the last active joint name (excludes fixed joints) for the right arm


	// Check If Either Group Is Using a Gripper
	if (lastJointName_left.compare(0,7,"gripper") == 0) { // if the last active joint for the left arm is a gripper
		usingGripper_left = true; // indicate the first group has a gripper
	}
	if (lastJointName_right.compare(0,7,"gripper") == 0) { // if the last active joint for the right arm is a gripper
		usingGripper_right = true; // indicate the second group has a gripper
	}

	// Notify the User If There Is Any Mismatch Between Gripper Data and the Gripper Existing
	if ((usingGripper_left) && (!pose_trajectory.usingGripper_left)) { 
		ROS_WARN("The loaded YuMi is using a gripper on the left arm, but the provided pose structure does not contain gripper position data."); 
		ROS_WARN("Result: The gripper position for the left arm will be set to 0.");
	} else if ((!usingGripper_left) && (pose_trajectory.usingGripper_left)) {
		ROS_WARN("The loaded YuMi is not using a gripper on the left arm, but the provided pose structure does contain gripper position data.");
		ROS_WARN("Result: The gripper position data for the left arm will be discarded.");
	}
	if ((usingGripper_right) && (!pose_trajectory.usingGripper_right)) {
		ROS_WARN("The loaded YuMi is using a gripper on the right arm, but the provided pose structure does not contain gripper position data."); 
		ROS_WARN("Result: The gripper position for the right arm will be set to 0.");
	} else if ((!usingGripper_right) && (pose_trajectory.usingGripper_right)) { 
		ROS_WARN("The loaded YuMi is not using a gripper on the right arm, but the provided pose structure does contain gripper position data.");
		ROS_WARN("Result: The gripper position data for the right arm will be discarded.");
	}

	// Check Optional Variable(s)
	if (debug) { // if this function is being run in debug mode
		ROS_WARN("Planning is being run in debug mode."); // notify the user that this function is being run in debug mode
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Starting planner."); // notify user that the planner is about to start

	for (int plans = 0; plans < pose_trajectory.totalPoints; plans++) {
		// Set Start State Planner Parameter for Provided Group
		if (planIndex == 0) { // if first time running through the while loop
			left_arm.setStartStateToCurrentState(); // set the start location for the planner to the current location of the robot for the left arm
			right_arm.setStartStateToCurrentState(); // set the start location for the planner to the current location of the robot for the right arm
			both_arms.setStartStateToCurrentState(); // set the start location for the planner to the current location of the robot for both arms
		} else { // if it is not the first time running through the while loop
			// Set Current Plan to Previous Plan if it Was Not Successful
			if ((!success_left) || (!success_right)) { // if each of the previous plans for the arms were not successful on the last iteration
				currentPlan_left  = lastSuccessfulPlan[0]; // set the previous plan to the last successful plan for the right arm
				currentPlan_right = lastSuccessfulPlan[1]; // set the previous plan to the last successful plan for the right arm
			}
			if (!(plan.success)) { // if the planner for both arms was not successful on the previous iteration
				currentPlan = lastSuccessfulPlan[2]; // set the previous plan to the last sucessful plan for both arms
			}

			// Set the State State of New Plan to Previous Trajectory Target from Planner
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

		// Store Previous Plans if They Were Successful
		if ((success_left) && (success_right)) { // if both plans were successful on the previous iteration
			lastSuccessfulPlan[0] = currentPlan_left; // store the previous plan for the left arm
			lastSuccessfulPlan[1] = currentPlan_right; // store the previous plan for the right arm
		}

		// Plan Trajectories for Each Arm
		left_arm.setPoseTarget(pose_trajectory.pose_left[plans]); // set the next target for the left arm
		currentPlan_left = emptyPlan; // reset the current plan variable for left arm
		success_left = left_arm.plan(currentPlan_left); // create a path plan for current trajectory point for left arm

		right_arm.setPoseTarget(pose_trajectory.pose_right[plans]); // set the next target for right arm
		currentPlan_right = emptyPlan; // reset the current plan variable for right arm
		success_right = right_arm.plan(currentPlan_right); // create a path plan for current trajectory point for right arm

		if ((success_left) && (success_right)) { // if the planner for each arm was successful
			// Build the Next Trajectory Target for Both Arms
			std::vector<double> lastTrajectoryPoint_left  = currentPlan_left.trajectory_.joint_trajectory.points.back().positions; // get the last set of joint values from the left planner for the current plan
			std::vector<double> lastTrajectoryPoint_right = currentPlan_right.trajectory_.joint_trajectory.points.back().positions; // get the last set of joint values from the right planner for the current plan
			std::vector<double> lastTrajectoryPoint = lastTrajectoryPoint_left; // store the joint values for left arm in the both_arms group as the last set of joint values from the left planner
			lastTrajectoryPoint.insert(lastTrajectoryPoint.end(),lastTrajectoryPoint_right.begin(),lastTrajectoryPoint_right.end()); // store the joint values for the right arm in the both_arms group as the last set of joint values from the right planner

			// Force Gripper Position in Trajectory Target if Both the Loaded Arm and Pose Structure Data Exist with a Gripper and Gripper Data Respectively
			if (usingGripper_left) { // if the YuMi model contains a gripper for the left arm
				if (pose_trajectory.usingGripper_left) { // if the provided pose structure contains data for the left gripper position
					lastTrajectoryPoint[totalJoints_left-1] = pose_trajectory.gripperPos_left[plans]; // force the left gripper position to the data from the provided pose structure
				} else { // if the provided pose structure does not contain data for the left gripper
					lastTrajectoryPoint[totalJoints_left-1] = gripper_closed_position; // force the left gripper position to the closed position
				}
			}
			if (usingGripper_right) { // if the YuMi model contains a gripper for the right arm
				if (pose_trajectory.usingGripper_right) { // if the provided pose structure contains data for the right gripper position
					lastTrajectoryPoint[totalJoints_left+totalJoints_right-1] = pose_trajectory.gripperPos_right[plans]; // force the right gripper position to the data from the provided pose structure
				} else { // if the provided pose structure does not contain data for the right gripper
					lastTrajectoryPoint[totalJoints_left+totalJoints_right-1] = gripper_closed_position; // force the right gripper position to the closed position
				}
			}

			// Plan Trajectory for Both Arms
			both_arms.setJointValueTarget(lastTrajectoryPoint); // set the target to the newly created joint value vector
			lastSuccessfulPlan[2] = currentPlan; // store the previous plan for both arms
			currentPlan = emptyPlan; // reset the plan variable for both arms
			plan.success = both_arms.plan(currentPlan); // try to create a plan for both arms

			// Check if Plan Was Successful
			if (plan.success) { // if the plan was successfully created
				planIndex++; // increment the plan counter
				plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
				ROS_INFO("Plan created for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning was executed for
			} else { // if the planning was not successful
				ROS_WARN("Plan failed for both arms for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
				if (debug) { // if the function is in debug mode
					plan.success = true; // reset the planner success flag to true since in debug mode
					ROS_WARN("Skipping trajectory point and continuing planner (planner is in debug mode)."); // notify the user that the trajectory point is being skipped due to debug mode
				} else { // if the function is not in debug mode
					ROS_WARN("Stopping planner."); // notify the user that the planner is being stopped
					break; // exit planning loop
				}
			}
		} else { // if the planning was not successful
			if ((!(success_left)) && (!(success_right))) { // if both the left and right arms planners were unsuccessful
				ROS_WARN("Plan failed for both arms for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			} else if (!(success_left)) { // if only the left arm planner was unsuccessful
				ROS_WARN("Plan failed for left arm for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			} else if (!(success_right)) { // if only the right arm planner was unsuccessful
				ROS_WARN("Plan failed for right arm for trajectory point: %d of %d",plans+1,pose_trajectory.totalPoints); // notify user of the trajectory point which planning failed to execute for
			}

			if (debug) { // if the function is in debug mode
				plan.success = true; // reset the planner success flag to true since in debug mode
				ROS_WARN("Skipping trajectory point and continuing planner (planner is in debug mode)."); // notify the user that the trajectory point is being skipped due to debug mode
			} else { // if the function is not in debug mode
				plan.success = false; // indicate that the planner for failed
				ROS_WARN("Stopping planner."); // notify the user that the planner is being stopped
				break; // exit planning loop
			}
		}
	}

	// Notfiy User if Planner Succeessful and/or Success Totals
	if (plan.success) { // if the planning was successful or was in debug mode
		plan.totalPlans = plan.plans.size(); // store the total plans within the planner
		ROS_INFO("Finished planning. Total plans created: %d of %d",plan.totalPlans,pose_trajectory.totalPoints); // notify user that the planner has finished
	} else { // if the planning was not successful
		ROS_INFO("Planning unsuccessful. Planning failed for trajectory point: %d of %d",planIndex+1,pose_trajectory.totalPoints); // notify user that the planner has finished
	}

	return plan; // return the planner success flag and the vector of plans
}

bool executePlans(planningInterface::MoveGroup& group, planner& plan, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-06-22
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

	// Check Optional Variable(s)
	if (debug) { // if this function is being run in debug mode
		ROS_WARN("Planning is being run in debug mode."); // notify the user that this function is being run in debug mode
	}

	// Ensure Proper Group is Used for Provided Planner
	if (plan.groupName.compare(group.getName()) != 0) { // if the given group does not have the same name as the name within the planner object
		ROS_WARN("Provided planner is not meant for the provided group."); // notify the user that the provided planner structure is not for the provided group
		ROS_INFO("Planner Group: %s",plan.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_INFO("Provided Group: %s",group.getName().c_str()); // notify user of the name of the supplied group
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else if (plan.success == 0) {
		ROS_WARN("Provided planner did not successfully create plans for one or all trajectory points for supplid text file.");
		ROS_INFO("Please (re)run the generatePaths() function before executing paths again.");
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else {
		// Notify User that the Plan Exection is About to be Started
		ROS_INFO("Executing trajectories."); // notify user that the execution of the plans is about to start

		// Go to Trajectory Start State
		group.setStartStateToCurrentState();
		group.setJointValueTarget(plan.plans[0].start_state_.joint_state.position);
		executeSuccess_flag = group.move();
		if (!(executeSuccess_flag)) {
			ROS_WARN("Not able to go to the start state of the trajectory.");
			if (debug) {
				executeSuccess_flag = true;
				ROS_WARN("Continuing execution (debug mode is on).");
			} else {
				ROS_WARN("Stopping execution of trajectories.");
			}
		}

		// Execute Given Plans
		if (executeSuccess_flag) {
			for (int currentPlan = 0; currentPlan < plan.totalPlans; currentPlan++) { // for all plans in plans vector
				executeSuccess_flag = group.execute(plan.plans[currentPlan]); // execute trajectory for the current plan
				if (executeSuccess_flag) { // if the execution was successful
					ROS_INFO("Executed trajectory: %d",currentPlan+1); // notify user of the plan number that was executed successfully
				} else { // if the execution was not successful
					ROS_WARN("Failed to execute trajectory for plan: %d",currentPlan+1); // notify user of the plan number that was not executed successfully
					break; // break from the for loop
				}
			}
		}
	}

	// Notify User if the Execition Was Successful
	if (executeSuccess_flag) { // if the execution was successful
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


