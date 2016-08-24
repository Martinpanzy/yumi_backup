// INCLUDES
#include <fstream>
#include <regex>
#include <sstream>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// NAMESPACE DECLARATIONS
using namespace ros;
namespace planningInterface = moveit::planning_interface;

// GLOBAL VARIABLES
const std::string yumi_scripts_directory = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/";
std::string command;
bool new_command = false;

// STRUCTURES
struct poseConfig {
    geometry_msgs::Pose pose;
    double gripper_position;
    std::vector<int> confdata;
    double external_axis_position;
};

struct leadThroughPoses {
	std::string arm;
	std::vector<poseConfig> pose_configs;
	std::vector<std::string> pose_names;
	bool gripper_attached = false;
};

// FUNCTION PROTOTYPES
poseConfig getAxisConfigurations(planningInterface::MoveGroup&, bool debug = false);

void writeToFile(std::string, leadThroughPoses&, bool debug = false);
void writeToFile(std::string, leadThroughPoses&, leadThroughPoses&, bool debug = false);
std::string getRobtargetOutputLine(std::string, poseConfig&, bool debug = false);

// MAIN FUNCTION
int main(int argc,char **argv) {

	// INITIALIZE VARIABLES
	std::string output_file_name;

	leadThroughPoses poses_left;
	leadThroughPoses poses_right;

	int point_left_move  = 1;
	int point_right_move = 1;
	int point_left_movesync  = 1;
	int point_right_movesync = 1;
	int desired_group = -1;
	int pose_index = 1;

	bool debug = false;
	bool left_arm_only  = false;
	bool right_arm_only = false;
	bool skip_command     = false;
	bool gripper_movement = false;

	// CHECK INPUTS
	if (argv[1] == NULL) {
		ROS_ERROR("An output file name needs to be executed as an additional argument.");
		return 1;
	} else {
		output_file_name = argv[1];
	}

	if (argv[2] == NULL) {
		ROS_ERROR("Need to provide argument to indicate which arm to store values for (left/right/both).");
		return 1;
	} else {
		if (strcmp(argv[2],"left") == 0) { desired_group = 1; }
		else if (strcmp(argv[2],"right") == 0) { desired_group = 2; }
		else if (strcmp(argv[2],"both") == 0) { desired_group = 3; }
		else { // if the desired arm argument is not recognized
			ROS_ERROR("Argument for desired arm(s) is not recognized.");
			ROS_WARN("Recognized argument(s): left, right, both");
			ROS_WARN("Provided argument: %s", argv[2]);
			return 1;
		}
	}

	if (argv[3] != NULL) {
		if (strcmp(argv[3],"debug") == 0) {
			debug = true;
			ROS_INFO("Running in debug mode.");
		} else {
			ROS_ERROR("Debug argument not recognized.");
			ROS_WARN("Recognized argument(s): debug");
			ROS_WARN("Provided argument: %s", argv[3]);
			return 1;
		}
	}

	if (debug) {
		ROS_INFO("Provdied file name: %s", output_file_name.c_str());
		if (desired_group == 3) { ROS_INFO("Storing data for both arms."); }
		else { ROS_INFO("Storing data only for the %s arm.", argv[2]); }
		ROS_INFO(">--------------------");
	}

	init(argc,argv,"lead_through");
	NodeHandle node_handle;

	// DEFINE MOVE GROUPS
	planningInterface::MoveGroup left_arm("left_arm_ik");
    left_arm.startStateMonitor();
    planningInterface::MoveGroup right_arm("right_arm_ik");
    right_arm.startStateMonitor();
    planningInterface::MoveGroup left("left_arm"); // used strictly to determine if a gripper was loaded on the left arm
    planningInterface::MoveGroup right("right_arm"); // user strictly to determine if a gripper was loaded on the right arm

    // SET MOVE GROUP REFRENCE FRAMES AND END EFFECTORS
	std::string end_effector_left    = "yumi_link_7_l";
	std::string end_effector_right   = "yumi_link_7_r";
	std::string pose_reference_frame = "yumi_body";

	left_arm.setPoseReferenceFrame(pose_reference_frame);
	right_arm.setPoseReferenceFrame(pose_reference_frame);

	left_arm.setEndEffectorLink(end_effector_left);
	right_arm.setEndEffectorLink(end_effector_right);

	if (debug) {
		ROS_INFO("Left Arm  - Pose reference frame: %s | End effector link: %s", left_arm.getPoseReferenceFrame().c_str(), left_arm.getEndEffectorLink().c_str());
		ROS_INFO("Right Arm - Pose reference frame: %s | End effector link: %s", right_arm.getPoseReferenceFrame().c_str(), right_arm.getEndEffectorLink().c_str());
	}

	// ADD leadThroughPoses DATA
	poses_left.arm  = "left";
	poses_right.arm = "right";
	if (left.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
		poses_left.gripper_attached = true;
		if (debug) { ROS_INFO("A gripper is attached to the left arm."); }
	} else if (debug) {
		ROS_INFO("A gripper is not attached to the left arm.");
	}
	if (right.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
		poses_right.gripper_attached = true;
		if (debug) { ROS_INFO("A gripper is attached to the right arm."); }
	} else if (debug) {
		ROS_INFO("A gripper is not attached to the right arm.");
	}

	// CHECK IF DESIRED OUTPUT FILE ALREADY EXISTS
	if (desired_group == 3) {
		std::string file_left_path  = yumi_scripts_directory + "modules/" + output_file_name + "_left" + ".mod";
		std::string file_right_path = yumi_scripts_directory + "modules/" + output_file_name + "_right" + ".mod";
		/* When sotring data for both arms, the files for each arm will have the same name expect with a "_left" or "_right" at the end of the name */
		std::ifstream file_left(file_left_path);
		std::ifstream file_right(file_right_path);
		if ((file_left.good()) || (file_right.good())) {
			ROS_ERROR("One of the output file names already exists. Please use another name or delete the existing file.");
			ROS_WARN("Left arm output file:  %s", file_left_path.c_str());
			ROS_WARN("Right arm output file: %s", file_right_path.c_str());
			return 1;
		}
	} else {
		std::string file_path = yumi_scripts_directory + "modules/" + output_file_name + ".mod";
		std::ifstream file(file_path);
		if (file.good()) {
			ROS_ERROR("Output file name already exists. Please use another name or delete the existing file.");
			ROS_WARN("Output file path: %s", file_path.c_str());
			return 1;
		}
	}
	
	// INITIATE TERMINAL INPUT FROM USER
	const int MAX_ARGUMENTS = 1;

	std::vector<std::string> previous_point_names;
	std::string previous_point_arm;
	int system_return;

	system_return = std::system("clear"); // clear the screen
	ROS_INFO("Ready to take arguments. For a list of recognized arguments, enter \"help\"");

	while (ok()) {	
	/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
        DATE CREATED: 2016-08-04

        DESCRIPTION: This while loop waits for a command line input from the user and determines if the inputted argument is on
        			 of the regonized arguments. The recognized arguments include: exit, clear, state, debug, help, left_only, 
        			 right_only, close_left, close_right, open_left, open_right, and store. If the provided argument is not
        			 recognized, then the user will be notified and the script will wait for another input from the user. For
        			 more information on the recognized commands, please refer to the wiki page shown below.

       	WIKI: github.com/ethz-asl/yumi/wiki/YuMi-Lead-Through
    */
		std::vector<std::string> inputs(MAX_ARGUMENTS,"");
        int argument = 0;

        getline(std::cin, command);

        if (command.compare("exit") == 0) {
        /* If the user would like to exit the program*/
            return 1;

        } else if (command.compare("clear") == 0) {
        /* If the user would like to clear the terminal window screen */
            system_return = std::system("clear");
            continue;

        } else if (command.compare("state") == 0) {
        /* If the user would like to know the state variables of the function (debug, bounds, etc.) */
            ROS_INFO("Debug mode: %s", debug?"on":"off");
            continue;

        } else if (command.compare(0, 5, "debug") == 0) {
        /* If the user would like to change whether the script is running in debug mode */
			if (command.length() == 5) {
        	/* If the command just is "debug" */
            	debug = !debug;
            } else if (command.compare(6, 2, "on") == 0) {
            /* If the user indicated if debug should be set to on */
            	debug = true;
            } else if (command.compare(6, 3, "off") == 0) {
            /* If the user indicated if debug should be set to off */
            	debug = false;
            } else {
            	ROS_ERROR("Argument not recognized.");
            	continue;
            }

            ROS_INFO("Debug mode: %s", debug?"on":"off");
            continue;

		} else if (command.compare("help") == 0) {
        /* If the user would like a list of allowed arguments*/
			ROS_INFO("_____ List of Allowed Arguments _____");
            ROS_INFO("                store: Store the position of the arm(s)");
            ROS_INFO("           store left: Only store the position of the left arm");
            ROS_INFO("          store right: Only store the position of the right arm");
            ROS_INFO("           close left: Gripper is to be in the closed position for the left arm");
            ROS_INFO("          close right: Gripper is to be in the closed position for the right arm");
            ROS_INFO("            open left: Gripper is to be in the open position for the left arm");
            ROS_INFO("           open right: Gripper is to be in the open position for the right arm");
            ROS_INFO("               finish: Finished storing points, write to file");
            ROS_INFO("          show points: Show all stored point names with lined up synchronized movements");
            ROS_INFO("      delete previous: Delete the previously stored point");
            ROS_INFO(" delete previous left: Delete the previously stored point for the left arm");
            ROS_INFO("delete previous right: Delete the previously stored point for the right arm");
            ROS_INFO("           delete all: Delete all points stored");
            ROS_INFO(" ");
            ROS_INFO("                debug: Change whether script is run in debug mode or not");
            ROS_INFO("                state: Check if the script is in debug mode or not");
            ROS_INFO("                clear: Clear the terminal window");
            ROS_INFO("                 exit: Exit the program without writing to file");
            ROS_INFO("----------------------------------------");
            ROS_INFO("For more information, please refer to the Wiki: github.com/ethz-asl/yumi/wiki/YuMi-Lead-Through");
            continue;

		} else if (command.compare("store left") == 0) {
		/* If the user would like to only store the pose for the left arm */
			if (desired_group == 1) {
				ROS_WARN("Program was already set to only store the pose for the left arm.");
			} else if (desired_group == 2) {
				ROS_ERROR("The program was set to only store the positions for the right arm.");
				skip_command = true;
			} else {
				left_arm_only  = true;
				right_arm_only = false;
			}

		} else if (command.compare("store right") == 0) {
		/* If the user would like to only store the pose for the right arm */
			if (desired_group == 2) {
				ROS_WARN("Program was already set to only store the pose for the right arm.");
			} else if (desired_group == 1) {
				ROS_ERROR("The program was set to only store the positions for the left arm.");
				skip_command = true;
			} else {
				right_arm_only = true;
				left_arm_only  = false;
			}

		} else if (command.compare("open left") == 0) {
		/* If the user would like to indicate that the left gripper is to open */
			if (desired_group == 2) {
				ROS_ERROR("The program was set to only store the pose for the right arm.");
				skip_command = true;
			} else {
				if (poses_left.gripper_attached) {
					poses_left.pose_names.push_back("OpenHand");

					poseConfig empty_pose;
					poses_left.pose_configs.push_back(empty_pose);
					/* Add an empty pose to have the pose names and poses have the same index */

					gripper_movement = true;
				} else {
					ROS_ERROR("A gripper is not attached to the left arm.");
					skip_command = true;
				}
			}

		} else if (command.compare("open right") == 0) {
		/* If the user would like to indicate that the right gripper is to open */
			if (desired_group == 1) {
				ROS_ERROR("The program was set to only store the pose for the left arm.");
				skip_command = true;
			} else {
				if (poses_right.gripper_attached) {
					poses_right.pose_names.push_back("OpenHand");

					poseConfig empty_pose;
					poses_right.pose_configs.push_back(empty_pose);
					/* Add an empty pose to have the pose names and poses have the same index */

					gripper_movement = true;
				} else {
					ROS_ERROR("A gripper is not attached to the right arm.");
					skip_command = true;
				}
			}

		} else if (command.compare("close left") == 0) {
		/* If the user would like to indicate that the left gripper is to close */
			if (desired_group == 2) {
				ROS_ERROR("The program was set to only store the pose for the right arm.");
				skip_command = true;
			} else {
				if (poses_left.gripper_attached) {
					poses_left.pose_names.push_back("CloseHand");

					poseConfig empty_pose;
					poses_left.pose_configs.push_back(empty_pose);
					/* Add an empty pose to have the pose names and poses have the same index */

					gripper_movement = true;
				} else {
					ROS_ERROR("A gripper is not attached to the left arm.");
					skip_command = true;
				}
			}

		} else if (command.compare("close right") == 0) {
		/* If the user would like to indicate that the right gripper is to close */
			if (desired_group == 1) {
				ROS_ERROR("The program was set to only store the pose for the left arm.");
				skip_command = true;
			} else {
				if (poses_right.gripper_attached) {
					poses_right.pose_names.push_back("CloseHand");

					poseConfig empty_pose;
					poses_right.pose_configs.push_back(empty_pose);
					/* Add an empty pose to have the pose names and poses have the same index */

					gripper_movement = true;
				} else {
					ROS_ERROR("A gripper is not attached to the right arm.");
					skip_command = true;
				}
			}

		} else if (command.compare("store") == 0) {
		/* If the user would like to store the current position of YuMi */
			left_arm_only    = false;
			right_arm_only   = false;

		} else if (command.compare(0, 15, "delete previous") == 0) {
		/* If the user would like to delete the previous point stored or the previous point stored for a specified arm */
			if (desired_group == 1) {
			/* If this script is storing positions for the left arm */
				if ((previous_point_names.size() > 0) && (poses_left.pose_names.size() > 0)) {
				/* If there are points to delete for the left arm*/ 
					if (command.compare("delete previous right") == 0) {
					/* If the user would like to delete a previous point for an arm that was not set to store points */
						ROS_ERROR("The program was set to only store the positions for the left arm.");
					}

					poses_left.pose_names.pop_back();
					poses_left.pose_configs.pop_back();

					point_left_move--;
					previous_point_names.pop_back();

					ROS_INFO("Deleted the last point for the left arm.");
				} else {
				/* If there are no points to delete*/
					ROS_ERROR("There are not points to delete.");
					continue;
				}
			} else if (desired_group == 2) {
			/* If this script is storing positions for the right arm */
				if ((previous_point_names.size() > 0) && (poses_right.pose_names.size() > 0)) {
				/* If there are points to delete for the right arm*/
					if (command.compare("delete previous left") == 0) {
					/* If the user would like to delete a previous point for an arm that was not set to store points */
						ROS_ERROR("The program was set to only store the positions for the right arm.");
					}

					poses_right.pose_names.pop_back();
					poses_right.pose_configs.pop_back();

					point_right_move--;
					previous_point_names.pop_back();

					ROS_INFO("Deleted the last point for the right arm.");
				} else {
				/* If there are no points to delete*/
					ROS_ERROR("There are not points to delete.");
					continue;
				}
			} else if (desired_group == 3) {
			/* If this script is storing positions for both arms */
				if (command.compare("delete previous left") == 0) {
				/* If the user would like to delete the last point stored for the left arm */
					if ((previous_point_names.size() > 0) && (poses_left.pose_names.size() > 0)) {
					/* If there are points to delete for the left arm*/
						if (poses_left.pose_names.back().compare(0, 1, "s") == 0) {
						/* If the previous position stored for the left arm was supposed to be a synchronized movement with the right arm */
							std::string response;
							ROS_WARN("Previous position was meant to be a synchronized movement with the right arm");
							ROS_INFO("Would you like to keep the position on the right arm? (yes/no/skip)");
							getline(std::cin, response);

							if (response.compare("yes") == 0) {
							/* If the user would like to keep the position on the right arm that was synchronized with the last point name on the left arm */
								std::string new_point_name;
								std::string last_point_name_left = poses_left.pose_names.back();
								for (int index = (poses_right.pose_names.size()-1); index >= 0; index--) {
								/* Search through the poses names for the right arm to find the one that matches with the last point name on the left arm */
									if (poses_right.pose_names[index].compare(last_point_name_left) == 0) {
									/* If the current index of the point name for the right arm matches the last point name on the left arm, 
									   then change the point name to indicate that the point is not a synchronized movement with the left arm
									   anymore but rather is an independent movement */
										new_point_name = "p" + std::to_string(point_right_move);
										point_right_movesync--;
										point_right_move++;

										poses_right.pose_names[index] = new_point_name;
										break;
									}
								}
								
								for (int index = (previous_point_names.size()-1); index >= 0; index--) {
								/* Search through the previous point names list to find the one that matched with the last point name on the left arm */
									if (previous_point_names[index].compare(last_point_name_left) == 0) {
									/* If the current index of the previous point names matches the last point name on the left arm, then
									   update the previous point name with the new point name indicating the right arm is an independent
									   movement instead of a synchronized movement with the left arm */
										previous_point_names[index] = new_point_name;
										break;
									}
								}

								poses_left.pose_names.pop_back();
								poses_left.pose_configs.pop_back();
								point_left_movesync--;

								ROS_INFO("Deleted the last point for the left arm and renamed the synchronized point for the right arm.");
							} else if (response.compare("no") == 0) {
							/* If the user would not like to keep the position on the right arm that was synchronized with the last point name on the left arm */
								std::string new_point_name;
								std::string last_point_name_left = poses_left.pose_names.back();
								for (int index = (poses_right.pose_names.size()-1); index >= 0; index--) {
								/* Search through the poses names for the right arm to find the one that matches with the last point name on the left arm */
									if (poses_right.pose_names[index].compare(last_point_name_left) == 0) {
									/* If the current index of the point name for the right arm matches the last point name on the left arm */
										if (index != (poses_right.pose_names.size()-1)) {
										/* If the index is not the last point name for the right arm */
											for (int position = index+1; position < poses_right.pose_names.size(); position++) {
											/* Cycle through the point names for the right arm shifting the point names and pose configs that 
											   come after the point to be removed point to overwrite the point name and config being removed */
												poses_right.pose_names[position-1]   = poses_right.pose_names[position];
												poses_right.pose_configs[position-1] = poses_right.pose_configs[position];
											}
										}
										poses_right.pose_names.pop_back();
										poses_right.pose_configs.pop_back();
										break;
									}
								}

								for (int index = (previous_point_names.size()-1); index >= 0; index--) {
								/* Search through the previous point names list to find the one that matched with the last point name on the left arm */
									if (previous_point_names[index].compare(last_point_name_left) == 0) {
									/* If the current index of the previous point names matches the last point name on the left arm */
										if (index != (previous_point_names.size()-1)) {
										/* If the index is not the last point name for the previous point names list */
											for (int position = index+1; position < previous_point_names.size(); position++) {
											/* Cycle through the previous point names list shifting the previous point names that come after 
											   the point to be removed point to overwrite the point name being removed */
												previous_point_names[position-1] = previous_point_names[position];
											}
											
										}
										previous_point_names.pop_back();
										break;
									}
								}

								poses_left.pose_names.pop_back();
								poses_left.pose_configs.pop_back();
								point_left_movesync--;
								point_right_movesync--;

								ROS_INFO("Deleted the last point for the left arm and the synchronized point for the right arm.");
							} else if (response.compare("skip") == 0) {
							/* If the user would not like to delete the previous position */
								ROS_INFO("Not deleting any positions.");
								continue;
							} else {
							/* If the provided argument is not recognized */
								ROS_ERROR("Argument not recognized. Not deleting any positions.");
								continue;
							}
						} else {
						/* If the previous position for the left arm is independent from the right arm (non-synchronized movement) */
							std::string last_point_name_left = poses_left.pose_names.back();
							for (int index = (previous_point_names.size()-1); index >= 0; index--) {
							/* Search through the previous point names list to find the one that matched with the lat point name on the left arm */
								if (previous_point_names[index].compare(last_point_name_left) == 0) {
								/* If the current index of the previous point names matches the last point name on the left arm */
									if (index != (previous_point_names.size()-1)) {
									/* If the index is not the last point name for the previous point names list */
										for (int position = index+1; position < previous_point_names.size(); position++) {
										/* Cycle through the previous point names list shifting the previous point names that come after 
										   the point to be removed point to overwrite the point name being removed */
											previous_point_names[position-1] = previous_point_names[position];
										}
										
									}
									previous_point_names.pop_back();
									break;
								}
							}

							poses_left.pose_names.pop_back();
							poses_left.pose_configs.pop_back();
							point_left_move--;

							ROS_INFO("Deleted the last point for the left arm.");
						}
					} else {
					/* If there are no points to delete*/
						ROS_ERROR("There are not points to delete.");
					}
				} else if (command.compare("delete previous right") == 0) {
				/* If the user would like to delete the last point stored for the right arm */
					if ((previous_point_names.size() > 0) && (poses_right.pose_names.size() > 0)) {
					/* If there are points to delete for the right arm*/
						if (poses_right.pose_names.back().compare(0, 1, "s") == 0) {
						/* If the previous position stored for the right arm was supposed to be a synchronized movement with the left arm */
							std::string response;
							ROS_WARN("Previous position was meant to be a synchronized movement with the left arm");
							ROS_INFO("Would you like to keep the position on the left arm? (yes/no/skip)");
							getline(std::cin, response);

							if (response.compare("yes") == 0) {
							/* If the user would like to keep the position on the left arm that was synchronized with the last point name on the right arm */
								std::string new_point_name;
								std::string last_point_name_right = poses_right.pose_names.back();
								for (int index = (poses_left.pose_names.size()-1); index >= 0; index--) {
								/* Search through the poses names for the left arm to find the one that matches with the last point name on the right arm */
									if (poses_left.pose_names[index].compare(last_point_name_right) == 0) {
									/* If the current index of the point name for the left arm matches the last point name on the right arm, 
									   then change the point name to indicate that the point is not a synchronized movement with the right arm
									   anymore but rather is an independent movement */
										new_point_name = "p" + std::to_string(point_left_move);
										point_left_movesync--;
										point_left_move++;

										poses_left.pose_names[index] = new_point_name;
										break;
									}
								}
								
								for (int index = (previous_point_names.size()-1); index >= 0; index--) {
								/* Search through the previous point names list to find the one that matched with the last point name on the right arm */
									if (previous_point_names[index].compare(last_point_name_right) == 0) {
									/* If the current index of the previous point names matches the last point name on the right arm, then
									   update the previous point name with the new point name indicating the left arm is an independent
									   movement instead of a synchronized movement with the right arm */
										previous_point_names[index] = new_point_name;
										break;
									}
								}

								poses_right.pose_names.pop_back();
								poses_right.pose_configs.pop_back();
								point_right_movesync--;

								ROS_INFO("Deleted the last point for the right arm and renamed the synchronized point for the left arm.");
							} else if (response.compare("no") == 0) {
							/* If the user would not like to keep the position on the left arm that was synchronized with the last point name on the right arm */
								std::string new_point_name;
								std::string last_point_name_right = poses_right.pose_names.back();
								for (int index = (poses_left.pose_names.size()-1); index >= 0; index--) {
								/* Search through the poses names for the left arm to find the one that matches with the last point name on the right arm */
									if (poses_left.pose_names[index].compare(last_point_name_right) == 0) {
									/* If the current index of the point name for the left arm matches the last point name on the right arm */
										if (index != (poses_left.pose_names.size()-1)) {
										/* If the index is not the last point name for the left arm */
											for (int position = index+1; position < poses_left.pose_names.size(); position++) {
											/* Cycle through the point names for the left arm shifting the point names and pose configs that 
											   come after the point to be removed point to overwrite the point name and config being removed */
												poses_left.pose_names[position-1]   = poses_left.pose_names[position];
												poses_left.pose_configs[position-1] = poses_left.pose_configs[position];
											}
										}
										poses_left.pose_names.pop_back();
										poses_left.pose_configs.pop_back();
										break;
									}
								}

								for (int index = (previous_point_names.size()-1); index >= 0; index--) {
								/* Search through the previous point names list to find the one that matched with the last point name on the right arm */
									if (previous_point_names[index].compare(last_point_name_right) == 0) {
									/* If the current index of the previous point names matches the last point name on the right arm */
										if (index != (previous_point_names.size()-1)) {
										/* If the index is not the last point name for the previous point names list */
											for (int position = index+1; position < previous_point_names.size(); position++) {
											/* Cycle through the previous point names list shifting the previous point names that come after 
											   the point to be removed point to overwrite the point name being removed */
												previous_point_names[position-1] = previous_point_names[position];
											}
											
										}
										previous_point_names.pop_back();
										break;
									}
								}

								poses_right.pose_names.pop_back();
								poses_right.pose_configs.pop_back();
								point_right_movesync--;
								point_left_movesync--;

								ROS_INFO("Deleted the last point for the right arm and the synchronized point for the left arm.");
							} else if (response.compare("skip") == 0) {
							/* If the user would not like to delete the previous position */
								ROS_INFO("Not deleting any positions.");
								continue;
							} else {
							/* If the provided argument is not recognized */
								ROS_ERROR("Argument not recognized. Not deleting any positions.");
								continue;
							}
						} else {
						/* If the previous position for the left arm is independent from the right arm (non-synchronized movement) */
							std::string last_point_name_left = poses_left.pose_names.back();
							for (int index = (previous_point_names.size()-1); index >= 0; index--) {
							/* Search through the previous point names list to find the one that matched with the lat point name on the left arm */
								if (previous_point_names[index].compare(last_point_name_left) == 0) {
								/* If the current index of the previous point names matches the last point name on the left arm */
									if (index != (previous_point_names.size()-1)) {
									/* If the index is not the last point name for the previous point names list */
										for (int position = index+1; position < previous_point_names.size(); position++) {
										/* Cycle through the previous point names list shifting the previous point names that come after 
										   the point to be removed point to overwrite the point name being removed */
											previous_point_names[position-1] = previous_point_names[position];
										}
										
									}
									previous_point_names.pop_back();
									break;
								}
							}

							poses_left.pose_names.pop_back();
							poses_left.pose_configs.pop_back();
							point_left_move--;

							ROS_INFO("Deleted the last point for the right arm.");
						}
					} else {
					/* If there are no points to delete*/
						ROS_ERROR("There are not points to delete.");
						continue;
					}
				} else {
					if (previous_point_names.size() > 0) {
					/* If there are points to delete based on the list of previously stored positions */
						if (previous_point_names.back().compare(0, 1, "s") == 0) {
						/* If the previously stored position was a synchronized movement between the left and right arm */
							poses_left.pose_names.pop_back();
							poses_left.pose_configs.pop_back();
							poses_right.pose_names.pop_back();
							poses_right.pose_configs.pop_back();

							point_left_movesync--;
							point_right_movesync--;
							previous_point_names.pop_back();

							ROS_INFO("Deleted the last point for both arms.");
						} else if ((previous_point_names.back().compare(poses_left.pose_names.back()) == 0) && (previous_point_arm.compare("left") == 0)) {
						/* If the previously stored position was only meant for the left arm */
							poses_left.pose_names.pop_back();
							poses_left.pose_configs.pop_back();

							point_left_movesync--;
							previous_point_names.pop_back();

							ROS_INFO("Deleted the last point for the left arm.");
						} else if ((previous_point_names.back().compare(poses_right.pose_names.back()) == 0) && (previous_point_arm.compare("right") == 0)) {
						/* If the previously stored position was only meant for the right arm */
							poses_right.pose_names.pop_back();
							poses_right.pose_configs.pop_back();

							point_right_movesync--;
							previous_point_names.pop_back();

							ROS_INFO("Deleted the last point for the right arm");
						} else {
						/* If there is some error with deleting previous points that occurred since this script was first run */
							ROS_ERROR("Fatal error with deleting previous position.");
							ROS_WARN("An unusual error occurred most likely due to error in code for deleting the previous point.");
						}
					} else {
					/* If there are no points to delete*/
						ROS_ERROR("There are not points to delete.");
						continue;
					}
				}

				std::string previous_point_name = previous_point_names.back();
				if (previous_point_name.compare(poses_left.pose_names.back()) == 0) {
				/* If the new last previous pose name is the same as the last pose name for the left arm */
					if (poses_left.pose_names.back().compare(0, 1, "s") == 0) {
					/* If the last pose name is a syncronous move with the right arm */
						previous_point_arm = "both";
					} else {
					/* If the last pose name is an independent move from the right arm */
						previous_point_arm = "left";
					}
				} else if (previous_point_name.compare(poses_right.pose_names.back()) == 0) {
				/* If the new last previous pose name is the same as the last pose name for the right arm */
					previous_point_arm = "right";
				}
			}

			continue;

		} else if (command.compare("delete all") == 0) {
		/* If the user would like to delete all stored points */
			if (desired_group == 1) {
			/* If this script is storing positions for the left arm */
				poses_left.pose_names.clear();
				poses_left.pose_configs.clear();
				point_left_move = point_left_movesync = 1;

				previous_point_names.clear();

				ROS_INFO("Deleted all positions for the left arm.");
			} else if (desired_group == 2) {
			/* If this script is storing positions for the right arm */
				poses_right.pose_names.clear();
				poses_right.pose_configs.clear();
				point_right_move = point_right_movesync = 1;

				previous_point_names.clear();

				ROS_INFO("Deleted all positions for the right arm.");
			} else {
			/* If this script is storing positions for both arms */
				poses_left.pose_names.clear();
				poses_left.pose_configs.clear();
				point_left_move = point_left_movesync = 1;

				poses_right.pose_names.clear();
				poses_right.pose_configs.clear();
				point_right_move = point_right_movesync = 1;

				previous_point_names.clear();

				ROS_INFO("Deleted all positions for both arms.");
			}

			continue;

		} else if (command.compare("show points") == 0) {
		/* If the user would like to display the current set of points that have been stored in order */

			int index_left = 0, index_right = 0, line = 1;
			int poses_left_size = poses_left.pose_names.size(), poses_right_size = poses_right.pose_names.size();
			std::string point_name_left, point_name_right;
			while ((index_left < poses_left_size) || (index_right < poses_right_size)) {
			/* While there are still point names to be displayed */
				if ((index_left < poses_left_size) && (index_right < poses_right_size)) {
				/* If there points to still be displayed for the left and right arm */
					point_name_left  = poses_left.pose_names[index_left];
					point_name_right = poses_right.pose_names[index_right];

					if ((point_name_left.compare(0, 1, "s") == 0) && (point_name_right.compare(0, 1, "s") == 0)) {
					/* If the current index of point names for the left and right arm is indicating a synchronized movement */
						if (point_name_left.compare(point_name_right) == 0) {
						/* Double checking above statement */
							ROS_INFO("(Line %d)  Left: %s  |  Right: %s", line, point_name_left.c_str(), point_name_right.c_str());
							index_left++;
							index_right++;
						}
					} else if ((point_name_left.compare(0, 1, "p") == 0) || (point_name_left.compare(0, 1, "O") == 0) || (point_name_left.compare(0, 1, "C") == 0)) {
					/* If the current index of point names for the left arm is indicating an independent movement */
						if ((point_name_right.compare(0, 1, "p") == 0) || (point_name_right.compare(0, 1, "O") == 0) || (point_name_right.compare(0, 1, "C") == 0)) {
						/* If the current index of point names for the right arm is also indicating an independent movement */
							ROS_INFO("(Line %d)  Left: %s  |  Right: %s", line, point_name_left.c_str(), point_name_right.c_str());
							index_left++;
							index_right++;
						} else {
							ROS_INFO("(Line %d)  Left: %s  |  Right: --", line, point_name_left.c_str());
							index_left++;
						}
					} else if ((point_name_right.compare(0, 1, "p") == 0) || (point_name_right.compare(0, 1, "O") == 0) || (point_name_right.compare(0, 1, "C") == 0)) {
					/* If the current index of point names for the right arm is indicating an independent movement */
						if ((point_name_left.compare(0, 1, "p") == 0) || (point_name_left.compare(0, 1, "O") == 0) || (point_name_left.compare(0, 1, "C") == 0)) {
						/* If the current index of point names for the left arm is also indicating an independent movement */
							ROS_INFO("(Line %d)  Left: %s  |  Right: %s", line, point_name_left.c_str(), point_name_right.c_str());
							index_left++;
							index_right++;
						} else {
							ROS_INFO("(Line %d)  Left: --  |  Right: %s", line, point_name_right.c_str());
							index_right++;
						}
					}
				} else if (index_left < poses_left_size) {
				/* If only the left arm has point names to display */
					point_name_left  = poses_left.pose_names[index_left];
					ROS_INFO("(Line %d)  Left: %s  |  Right: --", line, point_name_left.c_str());
					index_left++;
				} else if (index_right < poses_right_size) {
				/* If only the right arm has point names to display */
					point_name_right = poses_right.pose_names[index_right];
					ROS_INFO("(Line %d)  Left: --  |  Right: %s", line, point_name_right.c_str());
					index_right++;
				}

				line++;
			}

			if ((poses_left_size == 0) && (poses_right_size == 0)) {
			/* If there are no stored points to show */
				ROS_WARN("There are no stored points to show");
			}

			continue;

		} else if (command.compare("finish") == 0) {
		/* If there are no more trajectory points to store, indicated by the command "finish" */
			ROS_INFO("Finish command received.");
			break;

		} else {
		/* If the agument is not recognized */
			ROS_ERROR("Argument not recognized.");
            continue;
		}

		std::string point_name;
		if (((desired_group == 1) || ((desired_group == 3) && (!right_arm_only))) && (!skip_command) && (!gripper_movement)) {
		/* If the left arm point is to be stored, either the user is only intending to store the
		   left arm values as indicated by the initial input, or the user initial input indicated
		   that both arm positions should be stored, but only wants to store the pose of the right
		   arm for the current arm position. */
			poseConfig pose_config_left = getAxisConfigurations(left_arm, debug);
			pose_config_left.pose = left_arm.getCurrentPose().pose;
			poses_left.pose_configs.push_back(pose_config_left);

			if ((left_arm_only) || (desired_group == 1)) {
				point_name = "p" + std::to_string(point_left_move);
				point_left_move++;
			} else { 
				point_name = "s" + std::to_string(point_left_movesync);
				point_left_movesync++; 
			}
			poses_left.pose_names.push_back(point_name);
			previous_point_names.push_back(point_name);
			previous_point_arm = "left";

			ROS_INFO("(Position index left: %d) Stored point %s for left arm.", (point_left_movesync+point_left_move)-2, point_name.c_str());
		}
		if (((desired_group == 2) || ((desired_group == 3) && (!left_arm_only))) && (!skip_command) && (!gripper_movement)) {
		/* If the left arm point is to be stored, either the user is only intending to store the
		   left arm values as indicated by the initial input, or the user initial input indicated
		   that both arm positions should be stored, but only wants to store the pose of the right
		   arm for the current arm position. */
			poseConfig pose_config_right = getAxisConfigurations(right_arm, debug);
			pose_config_right.pose = right_arm.getCurrentPose().pose;
			poses_right.pose_configs.push_back(pose_config_right);

			if ((right_arm_only) || (desired_group == 2)) {
				point_name = "p" + std::to_string(point_right_move);
				point_right_move++;
			} else { 
				point_name = "s" + std::to_string(point_right_movesync);
				point_right_movesync++; 
			}
			poses_right.pose_names.push_back(point_name);
			if (point_name.compare(0, 1, "p") == 0) {
			/* If only the right arm position is being stored, then add the point name to the list 
			   of previous point names. Otherwise, the point name has already been stored into this 
			   list previously when storing the position for the left arm above */
				previous_point_names.push_back(point_name);
				previous_point_arm = "right";
			} else {
			/* If the current position was stored for both arms, then the previous point name was for both arms */
				previous_point_arm = "both";
			}

			ROS_INFO("(Position index right: %d) Stored point %s for right arm.", (point_right_movesync+point_right_move)-2, point_name.c_str());
		}

		if (debug) { ROS_INFO("...................."); }

		if (skip_command) {
		/* If there was an error with the command supplied by the user */
			ROS_WARN("YuMi position not stored due to error.");
			skip_command = false;
		} else {
			if (gripper_movement) {
			/* If there the command indicated a gripper movement */
				ROS_INFO("Gripper command received and stored. Command: %s", command.c_str());
				gripper_movement = false;
			} else {
				pose_index++;
			}
		}
	}

	// WRITE DATA TO FILE
	if (ok()) {
		ROS_INFO(">--------------------");
		if (desired_group == 1) { writeToFile(output_file_name, poses_left, debug); }
		else if (desired_group == 2) { writeToFile(output_file_name, poses_right, debug); }
		else if (desired_group == 3) { writeToFile(output_file_name, poses_left, poses_right, debug); }
	}

	ROS_INFO("Finished file writing. Terminating program.");
	return 0;
}


poseConfig getAxisConfigurations(planningInterface::MoveGroup& group, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-26

    PURPOSE: The purpose of this function is to get the current axis configuration of the robot based
             on the current joint values of the provided group. The axis configurations are based on 
             the ABB convention, which is shown under the variabe type "confdata" in the ABB RAPID
             reference manual. These configurations are also described below.

    INSTRUCTIONS: The axis configraution is generated for the provided move group. The provided move
                  group can only be one of the following groups: left_arm, right_arm. If debug mode
                  is set to true, then the current joint values with be displayed along with the 
                  confdata calculated from the current joint values.


    -------------------- OTHER INFORMATION --------------------

    ABB YuMi AXIS NAMING CONVENTION (from YuMi base to end effector): [1, 2, 7, 3, 4, 5, 6]

    ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

    CONFIGURATION SETS: -4 -> [-360, -270)   -3 -> [-270, -180)   -2 -> [-180, -90)   -1 -> [-90,   0)
                         0 -> [0,      90)    1 -> [  90,  180)    2 -> [180,  270)    3 -> [270, 360)
        - The configuration values above are for joints 1, 4, and 6

    CFX: ABCD
        - A represents configuration for axis 5 and can be either (1) or (0)
            > (0) if axis 5 position >= 0 degrees, (1) if axis 5 position < 0 degrees
        - B represents configuration for axis 3 and can be either (1) or (0)
            > (0) if axis 3 position >= -90 degrees, (1) if axis 3 position < -90 degrees
        - C represents configuration for axis 2 and can be either (1) or (0)
            > (0) if axis 2 position >= 0 degrees, (1) if axis 2 position < 0 degrees
        - D represents the compatability bit, particulary used for linear movements
            > This value is not used and is always set to (0)
	
    Examples: (Compatibility bit assumed to be 0)
        - Axis 5 = 0 degrees, Axis 3 = -90 degrees, Axis 2 = 0 degrees   | cfx = 0000 or 0
        - Axis 5 = 0 degrees, Axis 3 = -91 degrees, Axis 2 = 0 degrees   | cfx = 0100 or 100
        - Axis 5 = -90 degrees, Axis 3 = 0 degrees, Axis 2 = -90 degrees | cfx = 1010

    EXTERNAL AXIS POSITION CONVENTION: [arm_angle, 9E+09, 9E+09, 9E+09, 9E+09, 9E+09]
*/
    if (debug) { ROS_INFO("...................."); }

    // ERROR CHECKING
    std::string group_name = group.getName();
    if (group_name.compare("both_arms") == 0) {
        if (!debug) { ROS_INFO(">--------------------"); }
        ROS_ERROR("From: getAxisConfigurations(group, debug)");
        ROS_ERROR("Cannot get axis configurations for both arms at the same time. Please run this function for each arm individually.");
        poseConfig empty_pose_config;
        return empty_pose_config;
    } else if ((group_name.compare(0, 8, "left_arm") != 0) && (group_name.compare(0, 9, "right_arm") != 0)) {
        if (!debug) { ROS_INFO(">--------------------"); }
        ROS_ERROR("From: getAxisConfigurations(group, debug)");
        ROS_ERROR("The provided group is not recognized by this function.");
        ROS_WARN("Provided group: %s", group_name.c_str());
        ROS_WARN("Recognized groups: left_arm, right_arm, left_arm_ik, right_arm_ik");
        poseConfig empty_pose_config;
        return empty_pose_config;
    }

    // INITIALIZE VARIABLES
    const double PI = 3.14159;
    const double rad2deg = 180.0/PI;

    poseConfig pose_config;
    std::vector<double> joint_vaues;
    int axis_1_config, axis_4_config, axis_6_config, cfx = 0;

    // GET CURRENT JOINT VALUES
    std::vector<double> joint_values = group.getCurrentJointValues();

    if (debug) {
        ROS_INFO("_____ (debug) Current Joint Positions _____");
        for (int joint = 0; joint < joint_values.size(); joint++) { 
        	ROS_INFO("Joint index %d values: %.5f", joint+1, joint_values[joint]); 
        } 
    }

    // GET AXIS CONFIGURATIONS
    axis_1_config = std::floor(joint_values[0] / (PI/2.0)); // joint 1 config value
    axis_4_config = std::floor(joint_values[4] / (PI/2.0)); // joint 4 config value
    axis_6_config = std::floor(joint_values[6] / (PI/2.0)); // joint 6 config value
    if (joint_values[5] < 0)     { cfx += 1000; } // joint 5 config value
    if (joint_values[3] < -PI/2) { cfx += 100;  } // joint 3 config value
    if (joint_values[1] < 0)     { cfx += 10;   } // joint 2 config value

    pose_config.confdata.push_back(axis_1_config);
    pose_config.confdata.push_back(axis_4_config);
    pose_config.confdata.push_back(axis_6_config);
    pose_config.confdata.push_back(cfx);

    pose_config.external_axis_position = joint_values[2]; // this is wrong, need to update to arm angle calculation

    if (debug) {
        ROS_INFO("_____ (debug) Axis Configuration for Current Joint Position _____");
        ROS_INFO("Configuration: [%d, %d, %d, %d] | External Axis Position: [%.5f]", 
            pose_config.confdata[0], pose_config.confdata[1], pose_config.confdata[2], pose_config.confdata[3], 
            pose_config.external_axis_position);
    }

    return pose_config;
}

void writeToFile(std::string output_file_name, leadThroughPoses& poses, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-27

    PURPOSE: The purpose of this function is to write the set of data points that were stored previously
    		 to the desired file supplied by the user. The user supplied the file name, and this function
    		 will place all files into the "modules" folder in the "yumi_scripts" ROS package. This function
    		 will write a module with the ABB convention for a RAPID Module file for YuMi.

    INSTRUCTIONS: The output_file_name variable should not have any path preceding it (home/folder/etc...)
    			  and show not contain any file extention (*.txt, *.mod, etc.) since this added in this
    			  function. The set of poses that were previously stored from the user also must be
    			  supplied to this function. If debug is set to true, the user will be notified of the
    			  current status of the function writing as well as each line that has been written to the
    			  file. 


    -------------------- OTHER INFORMATION --------------------

	SIMPLE ABB YuMi MODULE EXAMPLE:
        1) MODULE module_name
        2) LOCAL CONST string YuMi_App_Program_Version:="1.0.1";
        3) LOCAL VAR robtarget s1 := [[316.65,45.52,21.47],[0.0502929,0.702983,-0.635561,0.315196],[-1,0,-1,1010],[-169.577,9E+09,9E+09,9E+09,9E+09,9E+09]];
        4) PROC main()
        5) MoveSync s1;
        6) Move p1
        7) OpenHand;
        8) CloseHand;
        9) ENDPROC
       10) ENDMODULE

    ABB YuMi MODULE CONVENTIONS:
    	- First line is always the module name
    	- Second line always defines that this file is to be used by the YuMi App Program
    	- Next set of lines define the robtargets in descending numerical order
    	- Line after the robtarget definitions is the start of the main function
    	- Next set of lines define the robot arm and gripper movements for the robot to move through
    		- MoveSync indicates a synced movement with the other arm
    		- Move indicates an independent movement from the other arm
    	- The last two lines indicate the end of the main function and end of the module

    GRIPPER COMMANDS: The gripper can either be set to open or close. When set to close, the gripper will close its hand until
					  it is either completely closed, or had grasped an object. The gripper close force can also be set from the 
					  TouchPendant. For more instructions on how to properly use the gripper in this script when storing the 
					  gripper position, please refer to the wiki page of this repo (shown below).

	WIKI PAGE: https://github.com/ethz-asl/yumi/wiki
*/
    // INITIALIZE VARIABLES
    std::string robtarget_prefix = "LOCAL VAR robtarget ";
    std::string output_file_path = yumi_scripts_directory + "modules/" + output_file_name + ".mod";
	std::ofstream output_file;

	ROS_INFO("Writing to file at: %s", output_file_path.c_str());

	output_file.open(output_file_path.c_str(), std::ofstream::out | std::ofstream::app);

	// ADD HEADER TO FILE
	output_file << "MODULE " << output_file_name << std::endl;
	output_file << "LOCAL CONST string YuMi_App_Program_Version:=\"1.0.1\"; !Do not edit or remove this line!" << std::endl;
	
	// ADD ROBTARGETS
	if (debug) { ROS_INFO("_____ (debug) Adding Robtargets to File _____"); }
    for (int pose = poses.pose_names.size()-1; pose >= 0; pose--) {
    	if ((poses.pose_names[pose].compare(0, 1, "C") != 0) && (poses.pose_names[pose].compare(0, 1, "O") != 0)) {
    		output_file << robtarget_prefix << getRobtargetOutputLine(poses.pose_names[pose], poses.pose_configs[pose], debug) << std::endl;
    	}
    }

    output_file << "PROC main()" << std::endl;

    // ADD MAIN FUNCTION
    if (debug) { ROS_INFO("_____ (debug) Adding Main Function to File _____"); }
    for (int line = 0; line < poses.pose_names.size(); line++) {
    	if (poses.pose_names[line].compare(0, 1, "p") == 0) {
    		output_file << "Move " << poses.pose_names[line] << ";" << std::endl;

    		if (debug) { ROS_INFO("(debug) Move %s;", poses.pose_names[line].c_str()); }
    	} else if ((poses.pose_names[line].compare(0, 1, "C") == 0) || (poses.pose_names[line].compare(0, 1, "O") == 0)) {
    		output_file << poses.pose_names[line] << ";" << std::endl;
    	} else {
    		output_file << "MoveSync " << poses.pose_names[line] << ";" << std::endl;
    		if (debug) { ROS_INFO("(debug) MoveSync %s;", poses.pose_names[line].c_str()); }
    	}
    }

    output_file << "ENDPROC" << std::endl << "ENDMODULE" << std::endl;

	output_file.close();
}

void writeToFile(std::string output_file_name, leadThroughPoses& poses_left, leadThroughPoses& poses_right, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-27

    PURPOSE: The purpose of this function is create two RAPID modules using the data contained
    		 in the two leadThroughPoses structures supplied. This function creates unique names
    		 for each arm, and them supplied them to function which performs all file writing.

    INSTRUCTIONS: The output_file_name variable should not have any path preceding it (home/folder/etc...)
    			  and show not contain any file extention (*.txt, *.mod, etc.) since this added in this
    			  function. The set of poses that were previously stored from the user for both arms must
    			  also be supplied to this function. If debug is set to true, the user will be notified of
    			  the current status of the function writing as well as each line that has been written to
    			  the file for each arm.
*/
    // INITIALIZE VARIABLES
    std::string output_file_name_left  = output_file_name + "_left";
    std::string output_file_name_right = output_file_name + "_right";

    // WRITE TO FILE LEFT ARM
    if (debug) {
    	ROS_INFO("_____ (debug) Writing Left Arm File _____");
    }
    writeToFile(output_file_name_left, poses_left, debug);

    // WRITE TO FILE RIGHT ARM
    if (debug) {
    	ROS_INFO("_____ (debug) Writing Right Arm File _____");
    }
    writeToFile(output_file_name_right, poses_right, debug);
}

std::string getRobtargetOutputLine(std::string pose_name, poseConfig& pose_config, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-27

    PURPOSE: The purpose of this function is to construct robtarget data according to the ABB
    		 convention and return the robtarget as a string. The appropriate unit conversions
    		 are also applied according to the unit conventions shown below. The ABB robtarget
    		 conventionis also shown below.

    INSTRUCTIONS: The pose name and pose configuration data must be supplied. The returned
    			  value is a string containing the robtarget definition.


    -------------------- OTHER INFORMATION --------------------

    ABB UNITS: milimeters and degrees
    ROS UNITS: meters and radians

    ABB ROBTARGET CONVENTION: robtarget_name := [[position.x,position.y,position.z],[orientation.w,orientation.x,orientation.y,orientation.z],[cf1,cf4,cf6,cfx],[eax1,eax2,eax3,eax4,eax5,eax6]]
        > cf1 represents the configuration data for axis 1
        > cf4 represents the configuration data for axis 4
        > cf6 represents the configuration data for axis 6
        > cfx represents the configuration data for a combination of axis 5, 3, 2, and a compatibility bit
        > eax1 represents the arm angle
        > eax2 to eax6 represents 5 other external axis values, but these are not used on YuMi and thus are all set to 9E+09. This data is ignored.
*/
    // INITIALIZE VARIABLES
    const double PI      = 3.14159;
    const double rad2deg = 180.0/PI;
    const double m2mm    = 1000.0;

    std::ostringstream output;
    std::string empty_value = "9E+09";

    // CONSTRUCT ROBTARGET
    output << pose_name << " := [[";
    output << pose_config.pose.position.x*m2mm << "," << pose_config.pose.position.y*m2mm << "," << pose_config.pose.position.z*m2mm << "],[";
    output << pose_config.pose.orientation.w << "," << pose_config.pose.orientation.x << "," << pose_config.pose.orientation.y << "," << pose_config.pose.orientation.z << "],[";
	output << pose_config.confdata[0] << "," << pose_config.confdata[1] << "," << pose_config.confdata[2] << "," << pose_config.confdata[3] << "],[";
	output << pose_config.external_axis_position*rad2deg;
	for (int i = 0; i < 5; i++) { output << "," << empty_value; }
    output << "]];";

	if (debug) { ROS_INFO("(debug) %s", output.str().c_str()); }

	return output.str();
}


