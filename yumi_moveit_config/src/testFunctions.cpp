#include <moveit_msgs/GetPositionIK.h>

#include <moveit/robot_model_loader/robot_model_loader.h> // include robot model loader functionality for IK
#include <moveit/robot_model/robot_model.h> // include robot model capabilities for IK
#include <moveit/robot_state/robot_state.h> // include robot state capabiliteis for IK

struct planner {
	std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to store planned paths for trajectory points
	std::string groupName; // include group name
	bool success; // include boolean to store whether the planner was successful or not
};

struct trajectoryJoints_point {
	std::vector<double> joint;
};

struct trajectoryJoints {
	std::string groupName; // include group name
	std::vector<trajectoryJoints_point> joints;
	int totalJoints;
};

struct trajectoryPoses {
	std::string groupName;
	std::vector<geometry_msgs::Pose> pose_left;
	std::vector<geometry_msgs::Pose> pose_right;
};

planner generatePathsAndExecute(std::string, planningInterface::MoveGroup&);
planner generatePaths(std::string, planningInterface::MoveGroup&);
bool executePaths(planningInterface::MoveGroup&, planner&);

planningInterface::MoveGroup::Plan getTrajectory(planningInterface::MoveGroup&, std::string);
trajectoryJoints getTrajectoryJoints(planningInterface::MoveGroup&, std::string);
trajectoryPoses getTrajectoryPoses(planningInterface::MoveGroup&, std::string);


ServiceClient serviceIK = nodeHandle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik"); // create client for IK service


planningInterface::MoveGroup::Plan getTrajectory(planningInterface::MoveGroup& group, std::string inputFile) {
/*
	NOTE: Need to figure out how to create joint trajectory? Maybe conver to pose first? | DATE: 2016-06-28
*/
	trajectoryPoses poseTrajectory = getTrajectoryPoses(group,inputFile);
	moveit_msgs::RobotTrajectory trajectory;
	ROS_INFO("Computing cartesian path.");
	double fraction = group.computeCartesianPath(poseTrajectory,0.01,0.0,trajectory); // what is fraction?
	ROS_INFO("Successful computation of cartesian path.");

	ROS_INFO("Creating plan.");
	planningInterface::MoveGroup::Plan plan;
	plan.trajectory_ = trajectory;
	ROS_INFO("Plan successfully created. Total points: %lu",plan.trajectory_.joint_trajectory.points.size());

	return plan;
}

trajectoryJoints getTrajectoryJoints(planningInterface::MoveGroup& group, std::string inputFile) {
/*
	NOTE: Should push back be used below for storing joint values? | DATE 2016-06-28
*/
	// Initialize Variables
	std::string line; // variable to store each line of text from the input file
	std::string command; // variable to store the command for each trajectory point from the input file
	int currentLine = 0;

	std::string groupName_1; // variable to store the first group name
	std::string groupName_2; // variable to store the second group name
	std::vector<double>::size_type totalJoints_1; // variable to store the joint values for the first group
	std::vector<double>::size_type totalJoints_2; // variable to store the joint values for the second group

	std::string groupName = group.getName();
	trajectoryJoints trajectory_joints;
	trajectory_joints.groupName = groupName;

	// Get Group Name
	int groupName_index;
	if (groupName.compare("left_arm") == 0) {
		groupName_index = 1;
	} else if (groupName.comapre("right_arm") == 0) {
		groupName_index = 2;
	} else if (groupName.comapre("both_arms") == 0) {
		groupName_index = 3;
	} else {
		groupName_index = -1;
		ROS_ERROR("Provided group name is not recognized.");
		ROS_INFO("Recognized group names list: left_arm, right_arm, both_arms");
		ROS_INFO("Provided group: %s",groupName.c_str());
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Getting stored trajectory joints from file."); // notify user that the planner is about to start
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

			if (currentLine == 0) {
				if (groupName_index == 3) {
					if (((groupName_1.compare("right_arm") == 0) || (groupName_2.compare("right_arm") == 0)) && ((groupName_1.compare("left_arm") == 0) || (groupName_2.compare("left_arm") == 0)) {
						trajectory_joints.totalJoints = totalJoints_1 + totalJoints_2;
					} else {
						ROS_WARN("One of the group names within the provided input file is not recognized.");
						ROS_INFO("Recognized groups: left_arm, right_arm");
						ROS_INFO("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str());
						groupName_index = -1;
						break;
					}
				} else if (groupName_index == 2) {
					if (groupName_1.compare("right_arm") == 0) {
						trajectory_joints.totalJoints = totalJoints_1;
					} else if (groupName_2.compare("right_arm") == 0) {
						trajectory_joints.totalJoints = totalJoints_2;
					} else {
						ROS_WARN("One of the group names within the provided input file is not recognized.");
						ROS_INFO("Recognized groups: left_arm, right_arm");
						ROS_INFO("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str());
						groupName_index = -1;
						break;
					}
				} else if (groupName_index == 1) {
					if (groupName_1.compare("left_arm") == 0) {
						trajectory_joints.totalJoints = totalJoints_1;
					} else if (groupName_2.compare("left_arm") == 0) {
						trajectory_joints.totalJoints = totalJoints_2;
					} else {
						ROS_WARN("One of the group names within the provided input file is not recognized.");
						ROS_INFO("Recognized groups: left_arm, right_arm");
						ROS_INFO("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str());
						groupName_index = -1;
						break;
					}
				} else if (groupName_index == -1) {
					break;
				}
			}

			if (groupName_index == 1) {
				if (groupName_1.compare("left_arm") == 0) {
					trajectory_joints.joints[currentLine].joint.push_back(jointValues_1);
				} else if (groupName_2.compare("left_arm") == 0) {
					trajectory_joints.joints[currentLine].joint.push_back(jointValues_2);
				}
			} else if (groupName_index == 2) {
				if (groupName_1.compare("right_arm") == 0) {
					trajectory_joints.joints[currentLine].joint.push_back(jointValues_1);
				} else if (groupName_2.compare("right_arm") == 0) {
					trajectory_joints.joints[currentLine].joint.push_back(jointValues_2);
				}
			} else if (groupName_index == 3) {
				if (groupName_1.compare("left_arm") == 0) {
					for (int joint = 0; joint < trajectory_joints.totalJoints; joint++) {
						if (joint < totalJoints_1) { trajectory_joints.joints[currentLine].joint[joint] = jointValues_1[joint]; }
						else { trajectory_joints.joints[currentLine].joint[joint] = jointValues_2[joint-totalJoints_1]; }
					}
				} else
					for (int joint = 0; joint < trajectory_joints.totalJoints; joint++) {
						if (joint < totalJoints_2) { trajectory_joints.joints[currentLine].joint[joint] = jointValues_2[joint]; }
						else { trajectory_joints.joints[currentLine].joint[joint] = jointValues_1[joint-totalJoints_2]; }
					}
				}
			}

			currentLine++; // increment current line counter
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_WARN("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_INFO("File: %s",inputFile.c_str()); // notify user of the supplied file
	}

	if (groupName_index == -1) {
		trajectory_joints.totalJoints = 0;
	}

	return trajectory_joints;
}

trajectoryPoses getTrajectoryPoses(planningInterface::MoveGroup& group, std::string inputFile) {
	
	// Initialize Variables
	std::string line; // variable to store each line of text from the input file
	std::string command; // variable to store the command for each trajectory point from the input file
	int currentLine = 0;

	std::string groupName_1; // variable to store the first group name
	std::string groupName_2; // variable to store the second group name
	geometry_msgs::Pose pose_1;
	geometry_msgs::Pose pose_2;

	std::string groupName = group.getName();
	trajectoryPoses trajectory_poses;
	trajectory_poses.groupName = groupName;

	// Get Group Name
	int groupName_index;
	if (groupName.compare("left_arm") == 0) {
		groupName_index = 1;
	} else if (groupName.comapre("right_arm") == 0) {
		groupName_index = 2;
	} else if (groupName.comapre("both_arms") == 0) {
		groupName_index = 3;
	} else {
		groupName_index = -1;
		ROS_ERROR("Provided group name is not recognized.");
		ROS_INFO("Recognized group names list: left_arm, right_arm, both_arms");
		ROS_INFO("Provided group: %s",groupName.c_str());
	}

	// Notfify User that the Planner is About to Start
	ROS_INFO("Getting stored trajectory poses from file."); // notify user that the planner is about to start
	ROS_INFO("File location: %s",inputFile.c_str()); // notify user of the full path of the input file

	// Open File and Generate Plans for Trajectories
	std::ifstream textFile(inputFile.c_str()); // open provided text file
	if (textFile.is_open()) { // if the file was able to successfully open
		while (std::getline(textFile,line)) { // while there is still a new line to take from
			// Create String Steamer and Get Command for Current Trajectory Point
			std::istringstream currentLine(line); // create a string steam element for delimiting data by spaces
			currentLine >> command; // get command name for current trajectory point

			// Get First Group Names and Poses for Current Trajectory Point
			currentLine >> groupName_1; //  get the command and the group name for the first group
			currentLine >> pose_1.position.x >> pose_1.position.y >> pose_1.position.z; // get position for current pose from first group
			currentLine >> pose_1.orientation.x >> pose_1.orientation.y >> pose_1.orientation.z >> pose_1.orientation.w; // get orientation for current pose from first group

			currentLine >> groupName_2; //  get the group name for the second group
			currentLine >> pose_2.position.x >> pose_2.position.y >> pose_2.position.z; // get position for current pose from second group
			currentLine >> pose_2.orientation.x >> pose_2.orientation.y >> pose_2.orientation.z >> pose_2.orientation.w; // get orientation for current pose from second group

			// Check Group Names in Input File to Make Sure They Are Expected
			if (currentLine == 0) {
				if (groupName_index == 3) {
					if (!(((groupName_1.compare("right_arm") == 0) || (groupName_2.compare("right_arm") == 0)) && ((groupName_1.compare("left_arm") == 0) || (groupName_2.compare("left_arm") == 0))) {
						ROS_WARN("One of the group names within the provided input file is not recognized.");
						ROS_INFO("Recognized groups: left_arm, right_arm");
						ROS_INFO("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str());
						groupName_index = -1;
						break;
					}
				} else if (groupName_index == 2) {
					if ((groupName_1.compare("right_arm") != 0) && (groupName_2.compare("right_arm") != 0)) {
						ROS_WARN("One of the group names within the provided input file is not recognized.");
						ROS_INFO("Recognized groups: left_arm, right_arm");
						ROS_INFO("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str());
						groupName_index = -1;
						break;
					}
				} else if (groupName_index == 1) {
					if ((groupName_1.compare("left_arm") != 0) && (groupName_2.compare("left_arm") != 0)) {
						ROS_WARN("One of the group names within the provided input file is not recognized.");
						ROS_INFO("Recognized groups: left_arm, right_arm");
						ROS_INFO("Group names form file: %s, %s",groupName_1.c_str(),groupName_2.c_str());
						groupName_index = -1;
						break;
					}
				} else if (groupName_index == -1) {
					break;
				}
			}

			// Store the Current Pose into the Trajectory Structure
			if (groupName_index == 1) {
				if (groupName_1.compare("left_arm") == 0) {
					trajectory_poses.pose_left.push_back(pose_1);
				} else if (groupName_2.compare("left_arm") == 0) {
					trajectory_poses.pose_left.push_back(pose_2);
				}
			} else if (groupName_index == 2) {
				if (groupName_1.compare("right_arm") == 0) {
					trajectory_poses.pose_right.push_back(pose_1);
				} else if (groupName_2.compare("right_arm") == 0) {
					trajectory_poses.pose_right.push_back(pose_2);
				}
			} else if (groupName_index == 3) {
				if (groupName_1.compare("left_arm") == 0) {
					trajectory_poses.pose_left.push_back(pose_1);
					trajectory_poses.pose_right.push_back(pose_2);
				} else if (groupName_2.compare("left_arm") == 0) {
					trajectory_poses.pose_left.push_back(pose_2);
					trajectory_poses.pose_right.push_back(pose_1);
				}
			}

			currentLine++; // increment current line counter
		}
		textFile.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_WARN("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_INFO("File: %s",inputFile.c_str()); // notify user of the supplied file
	}

	return trajectory_poses;
}



planner generatePathsAndExecute(std::string inputFile, planningInterface::MoveGroup& group) {
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
			if (executeSuccess_flag == true) { // if the execution was successful
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

/* ==================== */
/* UNFINISHED FUNCTIONS */
/* ==================== */

bool getTrajectory(planningInterface::MoveGroup&, std::string);

trajectoryJoints convertPoseToJoints(planningInterface::MoveGroup&, trajectoryPoses&);
std::vector<double> computeIK(ServiceClient, geometry_msgs::Pose);

/* ---------------------- */
/* Main function comments */
/* ---------------------- */
	//left_arm.setPlannerId("RRTstarkConfigDefault"); // set planner for left arm
	//right_arm.setPlannerId("RRTstarkConfigDefault"); // set planner for right arm
	//both_arms.setPlannerId("RRTstarkConfigDefault"); // set planner for both arms

	// ROS_INFO("Total joints: %d",jointTrajectory.totalJoints);
	// for (int joint = 0; joint < jointTrajectory.totalJoints; joint++) {
	// 	ROS_INFO("Joint %d: %.4f",joint+1,jointTrajectory.joints[0][joint]);
	// }		

	// ROS_INFO("Position: %.4f %.4f %.4f Orientation: %.4f %.4f %.4f %.4f",poseTrajectory.pose_left[0].position.x,poseTrajectory.pose_left[0].position.y,poseTrajectory.pose_left[0].position.z,poseTrajectory.pose_left[0].orientation.x,poseTrajectory.pose_left[0].orientation.y,poseTrajectory.pose_left[0].orientation.z,poseTrajectory.pose_left[0].orientation.w);

	// if (poseTrajectory.groupName.compare("") != 0) {
	// 	jointTrajectory = convertPoseToJoints(left_arm,poseTrajectory);
	// }
	// for (int point = 0; point < poseTrajectory.totalPoints; point++) {
	//	std::vector<double> jointValues = computeIK(serviceIK,poseTrajectory.pose_left[0]);
	// 	sleep(2);
	// }

	// ROS_INFO("");

	// for (int joint = 0; joint < jointTrajectory.totalJoints; joint++) {
	// 	ROS_INFO("Joint %d: %.4f",joint+1,jointTrajectory.joints[1][joint]);
	// }

	// ROS_INFO("");

	// for (int joint = 0; joint < jointTrajectory.totalJoints; joint++) {
	// 	ROS_INFO("Joint %d: %.4f",joint+1,jointTrajectory.joints[2][joint]);
	// }

	// Plan and Execute Trajectory
	// if (jointTrajectory.groupName.compare("") != 0) {
	// 	planner plans = generatePlans(both_arms,jointTrajectory);
	// 	bool success = executePlans(both_arms,plans);
	// }

	//displayJointValues(both_arms); // display joint positions

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

	// planner plan_left_arm  = generatePaths(inputFile,both_arms);
	// if (plan_left_arm.success == true) {
	// 	executePaths(both_arms, plan_left_arm);
	// }


bool getTrajectory(planningInterface::MoveGroup& group, std::string inputFile) {
/*
	STILL WORKING ON THIS
*/

	bool success = true;



	// if (pointType.compare("poses") == 0) { // still working on this
	// 	trajectoryPoses poseTrajectory = getTrajectoryPoses(group,inputFile);
	// 	for (int poses = 0; poses < poseTrajectory.totalPoints; poses++) {
	// 		group.setPoseTarget(poseTrajectory.pose_right[poses]); // set next target for given group using given pose
	// 		success = executePlanner(group); // execute planner and movement
	// 	}
	// }

	// if (pointType.compare("joints") == 0) { // still working on this
	// 	trajectoryJoints jointTrajectory = getTrajectoryJoints(group,inputFile);
	// 	for (int joints = 0; joints < jointTrajectory.totalPoints; joints++) {
	// 		std::vector<double> jointValues = jointTrajectory.joints[joints];
	// 		group.setJointValueTarget(jointValues); // set next target for given group using given pose
	// 		success = executePlanner(group); // execute planner and movement
	// 	}
	// }

	// if (pointType.compare("cartesian") == 0) { // still working on this
	// 	planningInterface::MoveGroup::Plan plan;
	// 	moveit_msgs::RobotTrajectory trajectory;

	// 	for (int pose = 0; pose < poseTrajectory.pose_right.size(); pose++) {
	// 		ROS_INFO("Target %d: x: %.4f y: %.4f z: %.4f",pose+1,poseTrajectory.pose_right[pose].position.x,poseTrajectory.pose_right[pose].position.y,poseTrajectory.pose_right[pose].position.z);
	// 	}

	// 	ROS_INFO("Computing cartesian path.");
	// 	double fraction = group.computeCartesianPath(poseTrajectory.pose_right,0.01,0.0,trajectory,true);
	// 	ROS_INFO("Successful computation of cartesian path. Fraction: %.4f",fraction);

	// 	ROS_INFO("Creating plan.");
	// 	plan.trajectory_ = trajectory;
	// 	ROS_INFO("Plan successfully created. Total points: %lu",plan.trajectory_.joint_trajectory.points.size());

	// 	ROS_INFO("Executing Plan");
	// 	bool success = group.execute(plan);
	// }

	return success;
}

/* -----------------------------------------------
   ------------ KINEMATICS FUNCTIONS -------------
   ----------------------------------------------- */
trajectoryJoints convertPoseToJoints(planningInterface::MoveGroup& group, trajectoryPoses& trajectory_poses) {

	// Initialize Variables
	trajectoryJoints trajectory_joints;
	std::string groupName = group.getName();
	bool conversionSuccess = true;
	int totalJointss;

	int attempts = 10;
	double timeout = 0.1;

	ROS_INFO("Converting poses to joint values for group: %s",groupName.c_str());

	// Get Kinematic Model and Kinematic State
	robot_model_loader::RobotModelLoader modelLoader("robot_description"); // load robot model
	robot_model::RobotModelPtr kinematicModel = modelLoader.getModel(); // get kinematic model
	robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(kinematicModel));
	kinematicState->setToDefaultValues();

	// Get Joint Model(s) for Provided Group
	if (groupName.compare("both_arms") == 0) {
		const robot_state::JointModelGroup* jointModel_left = kinematicModel->getJointModelGroup("left_joints"); // get joint model for provided group
		const robot_state::JointModelGroup* jointModel_right = kinematicModel->getJointModelGroup("right_joints"); // get joint model for provided group
		std::vector<double> jointValues_left, jointValues_right;
		geometry_msgs::Pose pose_left, pose_right;
		bool success_left, success_right;

		// Convert Poses for Both Arm to Joint Values
		for (int pose = 0; pose < trajectory_poses.totalPoints; pose++) {
			std::vector<double> jointValues;
			pose_left  = trajectory_poses.pose_left[pose];
			pose_right = trajectory_poses.pose_right[pose];

			success_left  = kinematicState->setFromIK(jointModel_left, pose_left, attempts, timeout);
			if (success_left) {
				kinematicState->copyJointGroupPositions(jointModel_left, jointValues_right);
				jointValues = jointValues_left;
			}
			else {
				ROS_ERROR("Left arm pose to joints IK conversion for trajectory point %d failed.",pose);
				conversionSuccess = false;
				break;
			}

			success_right = kinematicState->setFromIK(jointModel_right, pose_right, attempts, timeout);
			if (success_right) {
				kinematicState->copyJointGroupPositions(jointModel_right, jointValues_left);
				jointValues.insert(jointValues.end(),jointValues_right.begin(),jointValues_right.end());
				trajectory_joints.joints.push_back(jointValues);
			}
			else {
				ROS_ERROR("Right arm pose to joints IK conversion for trajectory point %d failed.",pose);
				conversionSuccess = false;
				break;
			}
			totalJointss = jointValues.size();
		}
	} else if (groupName.compare("left_arm") == 0) {
		const robot_state::JointModelGroup* jointModel = kinematicModel->getJointModelGroup("left_joints"); // get joint model for provided group
		geometry_msgs::Pose currentPose;
		bool success;

		for (int pose = 0; pose < trajectory_poses.totalPoints; pose++) {
			std::vector<double> jointValues;
			currentPose = trajectory_poses.pose_left[pose];
			success = kinematicState->setFromIK(jointModel, currentPose, attempts, timeout);
			
			if (success) {
				kinematicState->copyJointGroupPositions(jointModel, jointValues);
				trajectory_joints.joints.push_back(jointValues);
			}
			else {
				ROS_ERROR("Left arm pose to joints IK conversion for trajectory point %d failed.",pose);
				conversionSuccess = false;
				break;
			}
			totalJointss = jointValues.size();
		}
	} else if (groupName.compare("right_arm") == 0) {
		const robot_state::JointModelGroup* jointModel = kinematicModel->getJointModelGroup("right_joints"); // get joint model for provided group
		geometry_msgs::Pose currentPose;
		bool success;

		for (int pose = 0; pose < trajectory_poses.totalPoints; pose++) {
			std::vector<double> jointValues;
			currentPose = trajectory_poses.pose_right[pose];
			success = kinematicState->setFromIK(jointModel, currentPose, attempts, timeout);
			
			if (success) {
				kinematicState->copyJointGroupPositions(jointModel, jointValues);
				trajectory_joints.joints.push_back(jointValues);
			}
			else {
				ROS_ERROR("Right arm pose to joints IK conversion for trajectory point %d failed.",pose);
				conversionSuccess = false;
				break;
			}
			totalJointss = jointValues.size();
		}
	}

	if (conversionSuccess == false) {
		ROS_WARN("Failed to convert poses to joint values.");
		trajectoryJoints emptyTrajectory;
		return emptyTrajectory;
	} else {
		ROS_INFO("Successfully converted poses to joints.");
		trajectory_joints.groupName     = trajectory_poses.groupName;
		trajectory_joints.intendedGroup = trajectory_poses.intendedGroup;
		trajectory_joints.totalJoints   = totalJointss;
		trajectory_joints.totalPoints   = trajectory_poses.totalPoints;
		return trajectory_joints;
	}
}

std::vector<double> computeIK(ServiceClient serviceIK, geometry_msgs::Pose pose) {
	moveit_msgs::GetPositionIK::Request serviceRequest;
	moveit_msgs::GetPositionIK::Response serviceResponse;

	geometry_msgs::PoseStamped poseStamped;
	poseStamped.pose = pose;
	//poseStamped.header.frame_id = "yumi_link_7_l";

	std::vector<std::string> link_names;
	link_names.push_back("yumi_body");
	link_names.push_back("yumi_link_1_l");
	link_names.push_back("yumi_link_2_l");
	link_names.push_back("yumi_link_3_l");
	link_names.push_back("yumi_link_4_l");
	link_names.push_back("yumi_link_5_l");
	link_names.push_back("yumi_link_6_l");
	link_names.push_back("yumi_link_7_l");

	std::vector<std::string> joint_names;
	joint_names.push_back("yumi_joint_1_l");
	joint_names.push_back("yumi_joint_2_l");
	joint_names.push_back("yumi_joint_7_l");
	joint_names.push_back("yumi_joint_3_l");
	joint_names.push_back("yumi_joint_4_l");
	joint_names.push_back("yumi_joint_5_l");
	joint_names.push_back("yumi_joint_6_l");

	std::vector<double> currentJointValues = {0,0,0,0,0,0,0};

	//robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
	//robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	//robot_state::JointModelGroup* jointModel_left = kinematic_model->getJointModelGroup("left_arm");

	ROS_INFO("Working 1");
	
	serviceRequest.ik_request.group_name       = "left_joints";
	//serviceRequest.ik_request.avoid_collisions = true;
	serviceRequest.ik_request.ik_link_name     = "yumi_link_7_l" ; // user get end effector call for this
	serviceRequest.ik_request.pose_stamped     = poseStamped;
	serviceRequest.ik_request.ik_link_names    = link_names;
	serviceRequest.ik_request.attempts         = 10;
	serviceRequest.ik_request.robot_state.joint_state.name = joint_names;
	serviceRequest.ik_request.robot_state.joint_state.position = currentJointValues;

	ROS_INFO("Working 2");

	serviceIK.call(serviceRequest,serviceResponse);

	ROS_INFO("Working 3");

	std::vector<double> jointValues = serviceResponse.solution.joint_state.position;
	std::vector<std::string> jointNames = serviceResponse.solution.joint_state.name;
	for (int joint = 0; joint < jointValues.size(); joint++) {
		ROS_INFO("Joint %s: %.4f",jointNames[joint].c_str(),jointValues[joint]);
	}

	ROS_INFO("Joint value size %lu",jointValues.size());
	ROS_INFO("Error code: %d",serviceResponse.error_code);

	return jointValues;
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