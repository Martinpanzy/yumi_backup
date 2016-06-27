#include <ros/ros.h> // include node functionality
#include <moveit/move_group_interface/move_group.h> // include move group functionality
#include <moveit/robot_state/conversions.h> // include robot state conversion functionality
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
	std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to store planned paths for trajectory points
	std::string groupName; // include group name
	bool success; // include boolean to store whether the planner was successful or not
};

// Function Prototypes
planner generatePathsAndExecute(std::string, planningInterface::MoveGroup&);
planner generatePaths(std::string, planningInterface::MoveGroup&);
bool executePaths(planningInterface::MoveGroup&, planner&);
bool executePaths_TwoArm(planningInterface::MoveGroup&, planner&, planningInterface::MoveGroup&, planner&);

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
	planningInterface::MoveGroup left_arm("left_arm");
	planningInterface::MoveGroup right_arm("right_arm");
	planningInterface::MoveGroup both_arms("both_arms");

	// Set Planning Parameters
	left_arm.setNumPlanningAttempts(5);
	right_arm.setNumPlanningAttempts(5);
	both_arms.setNumPlanningAttempts(10);

	displayJointValues(both_arms); // display joint positions

	// Move Groups to Calc Position
	//gotoGroupState(left_arm,"calc"); // go to calc position with the left arm
	//gotoGroupState(right_arm,"calc"); // go to calc position with the right arm
	//gotoGroupState(both_arms,"calc");
	//gotoGroupState(both_arms,"home");

	// Generate and Execute Paths
	// planner plan_left_arm  = generatePaths(inputFile,left_arm);
	// planner plan_right_arm = generatePaths(inputFile,right_arm);
	// if ((plan_left_arm.success == true) && (plan_right_arm.success == true)) {
	// 	executePaths_TwoArm(left_arm, plan_left_arm, right_arm, plan_right_arm);
	// }

	planner plan_left_arm  = generatePaths(inputFile,both_arms);
	if (plan_left_arm.success == true) {
		executePaths(both_arms, plan_left_arm);
	}

	// Display Joint Values
	displayJointValues(both_arms); // display joint positions

	return 0;
}

/* -----------------------------------------------
   ---------- PATH GENERATOR FUNCTIONS -----------
   ----------------------------------------------- */
planner generatePathsAndExecute(std::string inputFile, planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-23
	PURPOSE: Generate and execute plans from input file for provided group

	INPUT(S):
		> inputFile - name of the file containing desured trajectory points
		> group - group to use for planning trajectories
	OUTPUT(S):
		< planner - planner structure, created to ensure proper execution is performed

	DEPENDENCIES: This function expects for two group joint values sets to be present in the provided file
	              The SRDF group for "both_arms" must first contain the joints for the "left_arm" then the joints for the "right_arm"
	INPUT FILE CONVENTION: The input file must have the following convention for each trajectory point:
		- $(Command Name) $(Group 1 Name) $(Group 1 Total Joints (N_1)) $(Joint 1 Value) ... $(Joint N_1 Value) $(Group 2 Name) $(Group 2 Total Joints (N_2)) $(Joint 1 Value) ... $(Joint N_2 Value)

	NOTE: Currently not doing anything with the joint command from provided input file | DATE: 2016-06-22
*/
	// Initialize Variables
	std::string line; // variable to store each line of text from the input file
	std::string command; // variable to store the command for each trajectory point from the input file

	std::string groupName_1; // variable to store the first group name
	std::string groupName_2; // variable to store the second group name
	std::string groupName_BothArms = "both_arms";
	std::vector<double>::size_type totalJoints_1; // variable to store the joint values for the first group
	std::vector<double>::size_type totalJoints_2; // variable to store the joint values for the second group

	planner plan; // structure of type planner
	plan.groupName = group.getName(); // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories

	planningInterface::MoveGroup::Plan currentPlan; // structure for an individual plan
	std::vector<planningInterface::MoveGroup::Plan> emptyPlan(1); // empty vector of plans with size of 1
	int planIndex = 0; // counter to indicate the current plan

	// Notfify User that the Planner is About to Start
	ROS_INFO("Opening file and starting planner."); // notify user that the planner is about to start
	ROS_INFO("File location: %s",inputFile.c_str()); // notify user of the full path of the input file

	// Open File and Generate Plans for Trajectories
	std::ifstream textFile(inputFile.c_str()); // open provided text file
	if (textFile.is_open()) { // if the file was able to successfully open
		while (std::getline(textFile,line)) { // while there is still a new line to take from
			// Create String Steamer and Get Command for Current Trajectory Point
			std::istringstream currentLine(line); // create a string steam element for delimiting data by spaces
			currentLine >> command; // get command name for current trajectory point

			// Get First Group Name and Joint Values for Current Trajectory Point
			currentLine >> groupName_1 >> totalJoints_1; //  get the command, group name for the first group, and total joints
			std::vector<double> jointValues_1(totalJoints_1); // create vector of doubles to store joint values for the first group
			for (int joint = 0; joint < totalJoints_1; joint++) { // iterate through joint values
				currentLine >> jointValues_1[joint]; // store the current joint value
			}

			// Get Second Group Name and Joint Values for Current Trajectory Point
			currentLine >> groupName_2 >> totalJoints_2; // get the group name for the second group and the total joints
			std::vector<double> jointValues_2(totalJoints_2); // create vector of doubles to store join values for the second group
			for (int joint = 0; joint < totalJoints_2; joint++) { // iterate through joint values
				currentLine >> jointValues_2[joint]; // store the current joint value
			}

			// Set Next Target for Planner
			if (plan.groupName.compare(groupName_1) == 0) { // if the desired move group is the first group from the current line
				group.setJointValueTarget(jointValues_1); // set the next target for the right arm
			} else if (plan.groupName.compare(groupName_2) == 0) { // if the desired move group is the second group from the current line
				group.setJointValueTarget(jointValues_2); // set the next target for the left arm
			} else if (plan.groupName.compare(groupName_BothArms) == 0) { // if the desired move group is both arms
				int totalJoints_DualArm = totalJoints_1 + totalJoints_2; // calculate total joints between both arms
				std::vector<double> jointValues_DualArm(totalJoints_DualArm); // create vector to store joint values for both arms
				for (int joint = 0; joint < totalJoints_DualArm; joint++) { // iterate through joint values for both arms
					if (groupName_1.compare("left_arm")) { // if the first group from the provided file gives joints values for left arm (left arm joints must be assigned first, as defined in the SRDF file)
						if (joint < totalJoints_1) { // if the current joint is less than the total joints in the left arm
							jointValues_DualArm[joint] = jointValues_1[joint]; // assign the current joint to the respective joint in the "both_arms" group
						} else { // if the current joint is supposed to be taken from the right arm joint values
							jointValues_DualArm[joint] = jointValues_2[joint-totalJoints_1]; // assign the current joint from the respective joint in the right arm
						}
					} else if (groupName_2.compare("left_arm")) { // if the second group from the provided file gives joint values for the right arm (right arm joints must be assigned second, as defined in the SRDF file)
						if (joint < totalJoints_2) { // if the current joint is less than the total joint in the left arm
							jointValues_DualArm[joint] = jointValues_2[joint]; // assign the current joint to the respective joint in the "both_arms" group
						} else { // if the current joint is supposed to be taken from the right arm joint values
							jointValues_DualArm[joint] = jointValues_1[joint-totalJoints_2]; // assign the current joint from the rspective joint in the right arm
						}
					}
				}
				group.setJointValueTarget(jointValues_DualArm); // set the next target for the "both_arms" group
			} else { // if the provided group name is not recognized as any of the group names above
				ROS_WARN("Provided group name is not recognized from group names supplied in file"); // notify user that the inputted group is not recognized
				ROS_INFO("Provided group name: %s",plan.groupName.c_str()); // notify user of the input group name
				ROS_INFO("Group 1: %s (from file)",groupName_1.c_str()); // notify user of the first group name pulled from the provided file
				ROS_INFO("Group 2: %s (from file)",groupName_2.c_str()); // notfiy user of the second group name pulled from the provided file
				ROS_INFO("Other Possible Group(s): %s (from SRDF)",groupName_BothArms.c_str());
				plan.success = false; // set the planner success flag as failed
				break; // break from the while loop
			}

			// Plan Trajectory
			currentPlan = emptyPlan[0]; // reset the current plan variable
			group.setStartStateToCurrentState(); // set the start location to the current location of the robot
			plan.success = group.plan(currentPlan); // create a path plan for current trajectory point
			planIndex++; // increment plan index

			// Add Planned Trajectory to Planner Vector if Plan Was Successful
			if (plan.success == true) { // if the planning was successful
				plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
				ROS_INFO("Plan create for trajectory point: %d",planIndex); // notify user of the trajectory point which planning was executed for
			} else { // if the planning was not successful
				ROS_WARN("Plan failed for trajectory point: %d",planIndex); // notify user of the trajectory point which planning failed to execute for
				break; // break from the while loop
			}

			// Execute Newly Create Plan
			ROS_INFO("Executing trajectory."); // notify user that the execution of the new plan is about to start
			plan.success = group.execute(currentPlan); // execute trajectory for the current plan
			if (plan.success == true) { // if the execution was successful
				ROS_INFO("Trajectory %d executed",planIndex+1); // notify user of the plan number that was executed successfully
			} else { // if the execution was not successful
				ROS_WARN("Failed to execute trajectory for plan: %d",planIndex+1); // notify user of the plan number that was not executed successfully
				break; // break from the for loop
			}
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_WARN("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_INFO("File: %s",inputFile.c_str()); // notify user of the supplied file
	}

	// Notify User if the Planner Was Successful
	if (plan.success == true) { // if the planner was successful
		ROS_INFO("Finished planning."); // notify user that the planner has finished
		return plan; // return the planner success flag and the vector of plans
	} else { // if the planner was not successful
		ROS_ERROR("Planning or executing error occurred, exiting planner function."); // notify user that the planner failed
		return plan; // return the planner success flag and the empty vector of plans
	}
}

planner generatePaths(std::string inputFile, planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-22
	PURPOSE: Generate plans from input file for provided group

	INPUT(S):
		> inputFile - name of the file containing desured trajectory points
		> group - group to use for planning trajectories
	OUTPUT(S):
		< planner - planner structure, created to ensure proper execution is performed

	DEPENDENCIES: This function expects for two group joint values sets to be present in the provided file
	              The SRDF group for "both_arms" must first contain the joints for the "left_arm" then the joints for the "right_arm"
	INPUT FILE CONVENTION: The input file must have the following convention for each trajectory point:
		- $(Command Name) $(Group 1 Name) $(Group 1 Total Joints (N_1)) $(Joint 1 Value) ... $(Joint N_1 Value) $(Group 2 Name) $(Group 2 Total Joints (N_2)) $(Joint 1 Value) ... $(Joint N_2 Value)

	NOTE: Currently not doing anything with the joint command from provided input file | DATE: 2016-06-22
*/
	// Initialize Variables
	std::string line; // variable to store each line of text from the input file
	std::string command; // variable to store the command for each trajectory point from the input file

	std::string groupName_1; // variable to store the first group name
	std::string groupName_2; // variable to store the second group name
	std::string groupName_BothArms = "both_arms";
	std::vector<double>::size_type totalJoints_1; // variable to store the joint values for the first group
	std::vector<double>::size_type totalJoints_2; // variable to store the joint values for the second group

	planner plan; // structure of type planner
	plan.groupName = group.getName(); // get the group name of the provided group
	plan.success = false; // flag to indicate the success of the planner for provided trajectories

	planningInterface::MoveGroup::Plan currentPlan; // structure for an individual plan
	std::vector<planningInterface::MoveGroup::Plan> emptyPlan(1); // empty vector of plans with size of 1
	int planIndex = 0; // counter to indicate the current plan

	// Notfify User that the Planner is About to Start
	ROS_INFO("Opening file and starting planner."); // notify user that the planner is about to start
	ROS_INFO("File location: %s",inputFile.c_str()); // notify user of the full path of the input file

	// Open File and Generate Plans for Trajectories
	std::ifstream textFile(inputFile.c_str()); // open provided text file
	if (textFile.is_open()) { // if the file was able to successfully open
		while (std::getline(textFile,line)) { // while there is still a new line to take from
			// Set Start State Planner Parameter for Provided Group
			if (planIndex == 0) { // if first time running through the while loop
				group.setStartStateToCurrentState(); // set the start location to the current location of the robot
			} else { // if it is not the first time running through the while loop
				moveitCore::RobotState state(group.getRobotModel()); // create state structure using the robots current state
				moveitCore::jointTrajPointToRobotState(currentPlan.trajectory_.joint_trajectory, (currentPlan.trajectory_.joint_trajectory.points.size()-1), state); // update the state structure to the given trajectory point
				group.setStartState(state); // set the start state as the newly generated state
			}

			// Create String Steamer and Get Command for Current Trajectory Point
			std::istringstream currentLine(line); // create a string steam element for delimiting data by spaces
			currentLine >> command; // get command name for current trajectory point

			// Get First Group Name and Joint Values for Current Trajectory Point
			currentLine >> groupName_1 >> totalJoints_1; //  get the command, group name for the first group, and total joints
			std::vector<double> jointValues_1(totalJoints_1); // create vector of doubles to store joint values for the first group
			for (int joint = 0; joint < totalJoints_1; joint++) { // iterate through joint values
				currentLine >> jointValues_1[joint]; // store the current joint value
			}

			// Get Second Group Name and Joint Values for Current Trajectory Point
			currentLine >> groupName_2 >> totalJoints_2; // get the group name for the second group and the total joints
			std::vector<double> jointValues_2(totalJoints_2); // create vector of doubles to store join values for the second group
			for (int joint = 0; joint < totalJoints_2; joint++) { // iterate through joint values
				currentLine >> jointValues_2[joint]; // store the current joint value
			}

			// Set Next Target for Planner
			if (plan.groupName.compare(groupName_1) == 0) { // if the desired move group is the first group from the current line
				group.setJointValueTarget(jointValues_1); // set the next target for the right arm
			} else if (plan.groupName.compare(groupName_2) == 0) { // if the desired move group is the second group from the current line
				group.setJointValueTarget(jointValues_2); // set the next target for the left arm
			} else if (plan.groupName.compare(groupName_BothArms) == 0) { // if the desired move group is both arms
				int totalJoints_DualArm = totalJoints_1 + totalJoints_2; // calculate total joints between both arms
				std::vector<double> jointValues_DualArm(totalJoints_DualArm); // create vector to store joint values for both arms
				for (int joint = 0; joint < totalJoints_DualArm; joint++) { // iterate through joint values for both arms
					if (groupName_1.compare("left_arm") == 0) { // if the first group from the provided file gives joints values for left arm (left arm joints must be assigned first, as defined in the SRDF file)
						if (joint < totalJoints_1) { // if the current joint is less than the total joints in the left arm
							jointValues_DualArm[joint] = jointValues_1[joint]; // assign the current joint to the respective joint in the "both_arms" group
						} else { // if the current joint is supposed to be taken from the right arm joint values
							jointValues_DualArm[joint] = jointValues_2[joint-totalJoints_1]; // assign the current joint from the respective joint in the right arm
						}
					} else if (groupName_2.compare("left_arm") == 0) { // if the second group from the provided file gives joint values for the right arm (right arm joints must be assigned second, as defined in the SRDF file)
						if (joint < totalJoints_2) { // if the current joint is less than the total joint in the left arm
							jointValues_DualArm[joint] = jointValues_2[joint]; // assign the current joint to the respective joint in the "both_arms" group
						} else { // if the current joint is supposed to be taken from the right arm joint values
							jointValues_DualArm[joint] = jointValues_1[joint-totalJoints_2]; // assign the current joint from the rspective joint in the right arm
						}
					}
				}
				group.setJointValueTarget(jointValues_DualArm); // set the next target for the "both_arms" group
			} else { // if the provided group name is not recognized as any of the group names above
				ROS_WARN("Provided group name is not recognized from group names supplied in file"); // notify user that the inputted group is not recognized
				ROS_INFO("Provided group name: %s",plan.groupName.c_str()); // notify user of the input group name
				ROS_INFO("Group 1: %s (from file)",groupName_1.c_str()); // notify user of the first group name pulled from the provided file
				ROS_INFO("Group 2: %s (from file)",groupName_2.c_str()); // notfiy user of the second group name pulled from the provided file
				ROS_INFO("Other Possible Group(s): %s (from SRDF)",groupName_BothArms.c_str());
				plan.success = false; // set the planner success flag as failed
				break; // break from the while loop
			}

			// Plan Trajectory
			currentPlan = emptyPlan[0]; // reset the current plan variable
			plan.success = group.plan(currentPlan); // create a path plan for current trajectory point
			planIndex++; // increment plan index

			// Add Planned Trajectory to Planner Vector if Plan Was Successful
			if (plan.success == true) { // if the planning was successful
				plan.plans.push_back(currentPlan); // add the newly create plan to the plan vector
				ROS_INFO("Plan create for trajectory point: %d",planIndex); // notify user of the trajectory point which planning was executed for
			} else { // if the planning was not successful
				ROS_WARN("Plan failed for trajectory point: %d",planIndex); // notify user of the trajectory point which planning failed to execute for
				break; // break from the while loop
			}
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_WARN("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_INFO("File: %s",inputFile.c_str()); // notify user of the supplied file
	}

	// Notify User if the Planner Was Successful
	if (plan.success == true) { // if the planner was successful
		ROS_INFO("Finished planning."); // notify user that the planner has finished
		return plan; // return the planner success flag and the vector of plans
	} else { // if the planner was not successful
		ROS_ERROR("Planning error occurred, exiting planner function."); // notify user that the planner failed
		return plan; // return the planner success flag and the empty vector of plans
	}

}

bool executePaths(planningInterface::MoveGroup& group, planner& plan) {
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
		ROS_WARN("Provided planner is not meant for the provided group"); // notify the user that the provided planner structure is not for the provided group
		ROS_INFO("Planner Group: %s",plan.groupName.c_str()); // notify user of the group the planner is intended to be used for
		ROS_INFO("Provided Group: %s",group.getName().c_str()); // notify user of the name of the supplied group
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else if (plan.success == 0) {
		ROS_WARN("Provided planner did not successfully create plans for one or all trajectory points for supplid text file.");
		ROS_INFO("Please (re-)run the generatePaths() function before executing paths again");
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else {
		// Notify User that the Plan Exection is About to be Started
		ROS_INFO("Executing trajectories."); // notify user that the execution of the plans is about to start

		// Execute Given Plans
		for (int currentPlan = 0; currentPlan < plan.plans.size(); currentPlan++) { // for all plans in plans vector
			executeSuccess_flag = group.execute(plan.plans[currentPlan]); // execute trajectory for the current plan
			if (executeSuccess_flag == true) { // if the execution was successful
				ROS_INFO("Trajectory %d executed",currentPlan+1); // notify user of the plan number that was executed successfully
				sleep(0.5); // wait for 0.5 seconds
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

bool executePaths_TwoArm(planningInterface::MoveGroup& group1, planner& plan1, planningInterface::MoveGroup& group2, planner& plan2) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATE: 2016-06-22
	PURPOSE: Execute provided plans for two groups
	INPUT(S):
		> group - group to use to execute planned trajectories
		> plans - vector of plans that have been previously generated
	OUTPUT(S):
		< exectuteSuccess_flag - bool to indicate if the execution of all plans was successful or not successful

	DEPENDENCIES: The generatePaths() function must be run and return succesful before execution this function

	NOTES: Ensure bothj arms have the same amount of paths | DATE: 2016-06-23
*/
	// Initialize Variables
	bool executeSuccess_flag;// flag to indicate the success of the execution for provided plans

	// Ensure Proper Group is Used for Provided Planner
	if ((plan1.groupName.compare(group1.getName()) != 0) || (plan2.groupName.compare(group2.getName()) != 0)) { // if the given group does not have the same name as the name within the planner object
		ROS_WARN("One or both of the provided planner is not meant for the provided group"); // notify the user that the provided planner structure is not for the provided group
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else if ((plan1.success == 0) || (plan2.success == 0)) {
		ROS_WARN("One or both of the provided planner did not successfully create plans for one or all trajectory points for supplid text file.");
		ROS_INFO("Please (re-)run the generatePaths() function before executing paths again");
		executeSuccess_flag = false; // set success flag to indicate the execution was not successful
	} else {
		// Notify User that the Plan Exection is About to be Started
		ROS_INFO("Executing trajectories."); // notify user that the execution of the plans is about to start

		// Execute Given Plans
		for (int currentPlan = 0; currentPlan < plan1.plans.size(); currentPlan++) { // for all plans in plans vector
			group1.asyncExecute(plan1.plans[currentPlan]); // execute trajectory for the current plan with group 1
			executeSuccess_flag = group2.execute(plan2.plans[currentPlan]); // execute trajectory for the current plan with group 1
			if (executeSuccess_flag == true) { // if the execution was successful
				ROS_INFO("Trajectory %d executed",currentPlan+1); // notify user of the plan number that was executed successfully
				sleep(5); // wait for 0.5 seconds
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


/* -----------------------------------------------
   --------------- COMMENTED CODE ----------------
   ----------------------------------------------- */

//std::vector<planningInterface::MoveGroup::Plan> generatePaths(std::string inputFile) {
/* 
   NOTE: Currently broken, need to be update later on
*/
/*	// Initialize Variables for Pulling From Input File
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
	ROS_INFO("Input file: %s",inputFile.c_str());
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
} */


