// INCLUDES
#include <fstream>
#include <regex>
#include <sstream>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>

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
void commandCallback(std_msgs::String);

poseConfig getAxisConfigurations(planningInterface::MoveGroup&, bool debug = false);

void writeToFile(std::string, leadThroughPoses&, bool debug = false);
void writeToFile(std::string, leadThroughPoses&, leadThroughPoses&, bool debug = false);
std::string getRobtargetOutputLine(std::string, poseConfig&, bool debug = false);

// MAIN FUNCTION
int main(int argc,char **argv) {

	// INITIALIZE VARIABLES
	std::string output_file_name;
	std::string end_effector_left    = "yumi_link_7_l";
	std::string end_effector_right   = "yumi_link_7_r";
	std::string pose_reference_frame = "yumi_body";

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
	planningInterface::MoveGroup left_arm("left_arm");
    left_arm.startStateMonitor();
    planningInterface::MoveGroup right_arm("right_arm");
    right_arm.startStateMonitor();

    // SET MOVE GROUP REFRENCE FRAMES AND END EFFECTORS
	left_arm.setPoseReferenceFrame(pose_reference_frame);
	right_arm.setPoseReferenceFrame(pose_reference_frame);

	left_arm.setEndEffectorLink(end_effector_left);
	right_arm.setEndEffectorLink(end_effector_right);

	if (debug) {
		ROS_INFO("(debug) Left arm  | Pose reference frame: %s | End effector: %s", left_arm.getPoseReferenceFrame().c_str(), left_arm.getEndEffectorLink().c_str());
		ROS_INFO("(debug) Right arm | Pose reference frame: %s | End effector: %s", right_arm.getPoseReferenceFrame().c_str(), right_arm.getEndEffectorLink().c_str());
	}

	// ADD leadThroughPoses DATA
	poses_left.arm  = "left";
	poses_right.arm = "right";
	if (left_arm.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
		poses_left.gripper_attached = true;
	}
	if (right_arm.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
		poses_right.gripper_attached = true;
	}

	// CHECK IF DESIRED OUTPUT FILE ALREADY EXISTS
	if (desired_group == 3) {
		std::string file_left_path  = yumi_scripts_directory + "paths/" + output_file_name + "_left" + ".mod";
		std::string file_right_path = yumi_scripts_directory + "paths/" + output_file_name + "_right" + ".mod";
		/* When sotring data for both arms, the files for each arm will have the same name expect with a "_left" or "_right" at the end of the name */
		std::ifstream file_left(file_left_path);
		std::ifstream file_right(file_right_path);
		if ((file_left.good()) || (file_right.good())) {
			ROS_ERROR("Output file name already exists. Please use another name or delete the existing file.");
			ROS_WARN("Left arm output file:  %s", file_left_path.c_str());
			ROS_WARN("Right arm output file: %s", file_right_path.c_str());
			return 1;
		}
	} else {
		std::string file_path = yumi_scripts_directory + "paths/" + output_file_name + ".mod";
		std::ifstream file(file_path);
		if (file.good()) {
			ROS_ERROR("Output file name already exists. Please use another name or delete the existing file.");
			ROS_WARN("Output file path: %s", file_path.c_str());
			return 1;
		}
	}

	// SETUP SUBSCRIBER
	Subscriber sub = node_handle.subscribe("lead_through_commands", 1000, commandCallback);

	// NOTIFY USER THE PROGRAM IS READY TO ACCEPT COMMANDS
	ROS_INFO(">--------------------");
	ROS_INFO("Program ready to accept commands. Run the following command to store position.");
	ROS_INFO("yumi_store_point $command");
	ROS_INFO(" ");
	ROS_INFO("_____ LIST OF RECOGNIZED COMMANDS _____")
	if (desired_group == 3) {
		ROS_INFO("To only store the left arm position, replace $command with: left_only");
		ROS_INFO("To only store the right arm position, replace $command with: right_only");
	}
	ROS_INFO("To store the last trajectory point, replace $command with: finish");
	ROS_INFO("To close/open a gripper, use the following command: <open/close>_$arm");
	ROS_INFO(" NOTE:");
	ROS_INFO("  - Choose either open or close, example of valid command: open_left");
	ROS_INFO("  - Replace $arm with the desired arm, can be either: left or right");
	ROS_INFO("  - Moving the gripper for one arm will not store the position for the other arm");
	ROS_INFO("  - The gripper must be closed using TeachPendant, for more info please look at wiki page")
	ROS_INFO("        Wiki page: https://github.com/ethz-asl/yumi/wiki");
	
	// GET POSES
	ROS_INFO(">--------------------");
	/* The while loop below does all of the following: check if the command that was supplied by
	   the user is one of the recognized commands and perform the appropraite action(s), store the
	   pose and pose configuration for both arms or one arm depending on the parameters supplied
	   to this script and the command sent by the user, store all the relevent data for each pose,
	   and indicate to the user whether there were any issues with storing the pose or the command
	   supplied by the user. These operations are only executed when the user sends a command and
	   is only executed once for each command sent. */
	while (ok()) {	
		if (new_command) {
			// CHECK COMMAND
			if (command.compare("left_only") == 0) {
			/* Store only the pose for the left arm */
				if (desired_group == 1) {
					ROS_WARN("(Pose index: %d) Program was already set to only store the pose for the left arm.", pose_index);
				} else if (desired_group == 2) {
					ROS_WARN("(Pose index: %d) The program was set to only store the pose for the right arm.", pose_index);
					skip_command = true;
				} else {
					left_arm_only = true;
				}
			} else if (command.compare("right_only") == 0) {
			/* Store only the pose for the right arm */
				if (desired_group == 2) {
					ROS_WARN("(Pose index: %d) Program was already set to only store the pose for the right arm.", pose_index);
				} else if (desired_group == 1) {
					ROS_WARN("(Pose index: %d) The program was set to only store the pose for the left arm.", pose_index);
					skip_command = true;
				} else {
					left_arm_only = true;
				}
			} else if (command.compare("open_left") == 0) {
			/* Store command to open the left gripper */
				if (desired_group == 2) {
					ROS_WARN("(Pose index: %d) The program was set to only store the pose for the right arm.", pose_index);
					skip_command = true;
				} else {
					if (poses_left.gripper_attached) {
						poses_left.pose_names.push_back("OpenHand;");
						gripper_movement = true;
					} else {
						ROS_WARN("A gripper is not attached to the left arm.");
						skip_command = true;
					}
				}
			} else if (command.compare("open_right") == 0) {
			/* Store command to open the right gripper */
				if (desired_group == 1) {
					ROS_WARN("(Pose index: %d) The program was set to only store the pose for the left arm.", pose_index);
					skip_command = true;
				} else {
					if (poses_right.gripper_attached) {
						poses_right.pose_names.push_back("OpenHand;");
						gripper_movement = true;
					} else {
						ROS_WARN("A gripper is not attached to the right arm.");
						skip_command = true;
					}
				}
			} else if (command.compare("close_left") == 0) {
			/* Store command to close the left gripper */
				if (desired_group == 2) {
					ROS_WARN("(Pose index: %d) The program was set to only store the pose for the right arm.", pose_index);
					skip_command = true;
				} else {
					if (poses_left.gripper_attached) {
						poses_left.pose_names.push_back("CloseHand;");
						gripper_movement = true;
					} else {
						ROS_WARN("A gripper is not attached to the left arm.");
						skip_command = true;
					}
				}
			} else if (command.compare("close_right") == 0) {
			/* Store command to close the right gripper */
				if (desired_group == 1) {
					ROS_WARN("(Pose index: %d) The program was set to only store the pose for the left arm.", pose_index);
					skip_command = true;
				} else {
					if (poses_right.gripper_attached) {
						poses_right.pose_names.push_back("CloseHand;");
						gripper_movement = true;
					} else {
						ROS_WARN("A gripper is not attached to the right arm.");
						skip_command = true;
					}
				}
			}

			// STORE POSE
			std::string point_name;
			if (((desired_group == 1) || ((desired_group == 3) && (!right_arm_only))) && (!skip_command) && (!gripper_movement)) {
			/* If the left arm point is to be stored, either the user is only intending to store the
			   left arm values as indicated by the initial input, or the user initial input indicated
			   that both arm positions should be stored, but only wants to store the pose of the right
			   arm for the current arm position. */
				poseConfig pose_config_left = getAxisConfigurations(left_arm, debug);
				pose_config_left.pose = left_arm.getCurrentPose().pose;
				poses_left.pose_configs.push_back(pose_config_left);

				if (left_arm_only) {
					point_name = "p" + std::to_string(point_left_move);
					point_left_move++;
				} else { 
					point_name = "s" + std::to_string(point_left_movesync);
					point_left_movesync++; 
				}
				poses_left.pose_names.push_back(point_name);

				if (debug) { ROS_INFO("(Pose index: %d) (debug) Stored point %s for left arm.", pose_index, point_name.c_str()); }
			}
			if (((desired_group == 1) || ((desired_group == 3) && (!left_arm_only))) && (!skip_command) && (!gripper_movement)) {
			/* If the left arm point is to be stored, either the user is only intending to store the
			   left arm values as indicated by the initial input, or the user initial input indicated
			   that both arm positions should be stored, but only wants to store the pose of the right
			   arm for the current arm position. */
				poseConfig pose_config_right = getAxisConfigurations(right_arm, debug);
				pose_config_right.pose = right_arm.getCurrentPose().pose;
				poses_right.pose_configs.push_back(pose_config_right);

				if (right_arm_only) {
					point_name = "p" + std::to_string(point_right_move);
					point_right_move++;
				} else { 
					point_name = "s" + std::to_string(point_right_movesync);
					point_right_movesync++; 
				}
				poses_right.pose_names.push_back(point_name);

				if (debug) { ROS_INFO("(Pose index: %d) (debug) Stored point %s for right arm.", pose_index, point_name.c_str()); }
			}

			// CHECK FOR LAST TRAJECTORY POINT
			if (command.compare("finish") == 0) {
			/* If the last trajectory points has been stored, indicated by the command "finish" */
				ROS_INFO("(Pose index: %d) Last trajectory point received.");
				break;
			}

			// RESET FLAGS AND/OR NOTIFY USER OF ANY ISSUES/TYPE OF DATA STORED
			if (skip_command) {
				ROS_INFO("(Pose index: %d) Pose not stored due to warning.");
				skip_command = false;
			} else {
				if (gripper_movement) {
					ROS_INFO("(Pose index: %d) Gripper command received and stored.", pose_index);
					gripper_movement = false;
				} else {
					ROS_INFO("(Pose index: %d) Pose stored.", pose_index);
				}
				pose_index++;
			}
			
			new_command = 0;
		}
		spinOnce();
	}

	// WRITE DATA TO FILE
	ROS_INFO(">--------------------");
	if (desired_group == 1) { writeToFile(output_file_name, poses_left, debug); }
	else if (desired_group == 2) { writeToFile(output_file_name, poses_right, debug); }
	else if (desired_group == 3) { writeToFile(output_file_name, poses_left, poses_right, debug); }

	ROS_INFO("Finished file writing. Terminating program.");
	return 0;
}

void commandCallback(std_msgs::String msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-17

    PURPOSE: Callback function for receiving string commands through the topic "lead_through_commands". These commands can
             be sent using YuMi quick commands or by executing the full command in another terminal window as shown below.
             A command must be sent using the commands below, but only certain commands affect the script. When a command
             is received, the flag new_command will be set to true and the script will then store the position of the 
             arm(s) depending on the command sent and the desired group set at the script execution.


    -------------------- OTHER INFORMATION --------------------

    SENDING COMMANDS: (replace $command with one of the commands shown below)
    	YuMi Quick Commands: yumi_store_point $command
    	Full Command: rostopic pub /lead_through_commands std_msgs/String $command

   	LIST OF RECOGNIZED COMMANDS: (A command is required, but can be anything other than below if the user would like to store the pose for both arms and not begin writing data to the output file)
   		left_only - only store the pose of the left arm for the current trajectory point
   		right_only - only store the pose of the right arm for the current trajectory point
   		<open/close>_$arm - open or close the gripper on the designated arm (example: open_left opens the left arm's gripper)
   		finish - indicates that this trajectory point is the last point to be stored and to begin writing the trajectory to the output file
*/
	command = msg.data; // take command form topic into local variable
	new_command = true; // indicate that the user would like to write the current point to the file
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

    ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

    ABB YuMi AXIS NAMING CONVENTION (from YuMi base to end effector): [1, 2, 7, 3, 4, 5, 6]

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

    EXTERNAL AXIS POSITION CONVENTION: [axis_7_position]
*/
    if (debug) { ROS_INFO("...................."); }

    // ERROR CHECKING
    std::string group_name = group.getName();
    if (group_name.compare("both_arms") == 0) {
        if (!debug) { ROS_INFO(">--------------------"); }
        ROS_ERROR("From: getAxisConfigurations(group, debug)");
        ROS_ERROR("Cannot get axis configurations for both arms. Please run this function for each arm.");
        poseConfig empty_pose_config;
        return empty_pose_config;
    } else if ((group_name.compare("left_arm") != 0) && (group_name.compare("right_arm") != 0)) {
        if (!debug) { ROS_INFO(">--------------------"); }
        ROS_ERROR("From: getAxisConfigurations(group, debug)");
        ROS_ERROR("The provided group is not recognized by this function.");
        ROS_WARN("Provided group: %s", group_name.c_str());
        ROS_WARN("Recognized groups: left_arm, right_arm");
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
        ROS_INFO("_____ (debug) Current joint positions _____");
        for (int joint = 0; joint < joint_values.size(); joint++) { ROS_INFO("%.5f", joint_values[joint]); } 
    }

    // GET AXIS CONFIGURATIONS
    axis_1_config = std::floor((joint_values[0] * rad2deg) / 90.0); 
    axis_4_config = std::floor((joint_values[4] * rad2deg) / 90.0);
    axis_6_config = std::floor((joint_values[6] * rad2deg) / 90.0);
    if (joint_values[5] < 0)    { cfx += 1000; }
    if (joint_values[3] < PI/2) { cfx += 100;  }
    if (joint_values[1] < 0)    { cfx += 10;   }

    pose_config.confdata.push_back(axis_1_config);
    pose_config.confdata.push_back(axis_4_config);
    pose_config.confdata.push_back(axis_6_config);
    pose_config.confdata.push_back(cfx);

    pose_config.external_axis_position = joint_values[2];

    if (debug) {
        ROS_INFO("_____ (debug) Axis configuration for current joint positions _____");
        ROS_INFO("Configuration: [%d, %d, %d, %d] | External Axis Position: [%0.5f]", 
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
    		 will place all files into the "paths" folder in the "yumi_scripts" ROS package. This function
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
        6) OpenHand;
        7) CloseHand;
        8) ENDPROC
        9) ENDMODULE

    ABB YuMi MODULE CONVENTIONS:
    	- First line is always the module name
    	- Second line always defines that this file is to be used by the YuMi App Program
    	- Next set of lines define the robtargets in descending numerical order
    	- Line after the robtarget definitions is the start of the main function
    	- Next set of lines define the robot arm and gripper movements for the robot to move through
    	- The last two lines indicate the end of the main function and end of the module

    GRIPPER COMMANDS: The gripper can either be set to open or close. When set to close, the gripper will close its hand until
					  it is either completely closed, or had grasped an object. The gripper close force can also be set from the 
					  TouchPendant. For more instructions on how to properly use the gripper in this script when storing the 
					  gripper position, please refer to the wiki page of this repo (shown below).

	WIKI PAGE: https://github.com/ethz-asl/yumi/wiki
*/
    // INITIALIZE VARIABLES
    std::string robtarget_prefix = "LOCAL VAR robtarget ";
    std::string output_file_path = yumi_scripts_directory + "paths/" + output_file_name + ".mod";
	std::ofstream output_file;

	ROS_INFO("Writing to file at: %s", output_file_path.c_str());

	output_file.open(output_file_path.c_str(), std::ofstream::out | std::ofstream::app);

	// ADD HEADER TO FILE
	output_file << "MODULE " << output_file_name << std::endl;
	output_file << "LOCAL CONST string YuMi_App_Program_Version:=\"1.0.1\"; !Do not edit or remove this line!" << std::endl;
	
	// ADD ROBTARGETS
	if (debug) { ROS_INFO("(debug) Adding robtargets to file"); }
    for (int pose = poses.pose_configs.size()-1; pose >= 0; pose--) {
    	if ((poses.pose_names[pose].compare("CloseHand;") == 0) || (poses.pose_names[pose].compare("OpenHand;") == 0)) {
    		output_file << poses.pose_names[pose] << std::endl;
    	} else {
    		output_file << robtarget_prefix << getRobtargetOutputLine(poses.pose_names[pose], poses.pose_configs[pose], debug) << std::endl;
    	}
    }

    output_file << "PROC main()" << std::endl;

    // ADD MAIN FUNCTION
    if (debug) { ROS_INFO("(debug) Adding main function to file"); }
    for (int line = 0; line < poses.pose_names.size(); line++) {
    	if (poses.pose_names[line].compare(0, 1, "p") == 0) {
    		output_file << "something " << poses.pose_names[line] << ";" << std::endl;
    		if (debug) { ROS_INFO("(debug) something %s;", poses.pose_names[line].c_str()); }
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
    	ROS_INFO("_____ (debug) Writing left arm file _____");
    }
    writeToFile(output_file_name_left, poses_left, debug);

    // WRITE TO FILE RIGHT ARM
    if (debug) {
    	ROS_INFO("_____ (debug) Writing right arm file _____");
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
        > eax1 represents the position of the first external axis (axis 7)
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


