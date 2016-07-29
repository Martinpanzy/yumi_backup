#include <sstream> // include string stream functionality
#include <fstream> // include file stream functionality
#include <string> // include string functionality
#include <stack> // include stack functionality
#include <ctime> // include timing functionality
#include <math.h> // include power functionality

#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h> // include pose for move group functionality
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h> // include move group functionality
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h> // include robot state conversion functionality
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h> // include robot trajectory
#include <moveit_msgs/RobotState.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h> // include node functionality
#include <rosbag/bag.h> // allow rosbag functionality
#include <rosbag/view.h> // allow rosbag loading functionality

#include <yumi_scripts/PrePlan.h> // include custom message for planner


// Define Global Constants
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)
const std::string yumiScriptsDirectory = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/"; // full path to folder where trajectory text files should be stored

// Namespace Commands and Variables
using namespace ros; // use namespace of ros
namespace planningInterface = moveit::planning_interface; // define commonly used namespace for planning interface
namespace moveitCore = moveit::core; // define commonly used namespace for planning interface

// Define Structures
struct planner {
	std::string groupName; // include group name
	std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to retrieve planned paths for trajectory points
	int totalPlans; // include count of total plans
	bool success; // include boolean to retrieve whether the planner was successful or not
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
	std::vector<std::vector<int>> confdata_left; // include axis configuration data for the left arm
	std::vector<std::vector<int>> confdata_right; // include axis configuration data for the right arm	
	std::vector<double> joint_7_positions_left;
	std::vector<double> joint_7_positions_right;
	int totalPoints; // include count of total trajectory points
};

struct RAPIDModuleData {
	std::string moduleName;
	std::vector<std::string> pose_names;
	std::vector<geometry_msgs::Pose> poses;
	std::vector<std::vector<int>> confdata;
	std::vector<double> joint7_positions;
	std::vector<bool> gripper_positions;
	bool only_move_sync = true;
	int totalPoints;
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
std::vector<double> computeIK(ServiceClient, planningInterface::MoveGroup&, geometry_msgs::Pose, std::vector<int>, double);
std::vector<double> computeIK(ServiceClient, planningInterface::MoveGroup&, geometry_msgs::Pose, double);
std::vector<double> computeIK(ServiceClient, planningInterface::MoveGroup&, geometry_msgs::Pose);
moveit_msgs::MotionPlanResponse preplanTrajectory(planningInterface::MoveGroup&, geometry_msgs::Pose, std::vector<int>, ros::NodeHandle);
moveit_msgs::Constraints setAxisConfigurations(planningInterface::MoveGroup&, std::vector<int>, double, std::vector<double>&, bool debug = false);
moveit_msgs::Constraints setExternalAxisConstraint(double);

RAPIDModuleData getYuMiLeadThroughData(std::string, bool debug = false);
trajectoryPoses convertRAPIDtoPoseTrajectory(RAPIDModuleData&, planningInterface::MoveGroup&);
trajectoryPoses convertRAPIDtoPoseTrajectory(RAPIDModuleData&, RAPIDModuleData&);
void getRobtargetData(std::string, geometry_msgs::Pose&, std::vector<int>&, double&, bool debug = false);

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
	fullPath << yumiScriptsDirectory << "paths/" << argv[1] << ".txt"; // concatenate full path
	std::string inputFile = fullPath.str(); // retrieve the full path

	// Initialize ROS
	init(argc,argv,"run_lead_through"); // initialize ROS node
	NodeHandle node_handle; // initialize node handle
	AsyncSpinner spinner(1);
	spinner.start();

	// Initialize Move Groups
	planningInterface::MoveGroup left_arm("left_arm");
	left_arm.startStateMonitor();
	planningInterface::MoveGroup right_arm("right_arm");
	right_arm.startStateMonitor();
	planningInterface::MoveGroup both_arms("both_arms");
	both_arms.startStateMonitor();

	// std::vector<planningInterface::MoveGroup> *move_groups(3);
	// move_groups[0] = &left_arm;
	// move_groups[1] = &right_arm;
	// move_groups[2] = &both_arms;

	// int total_allowed_arguments = 8;
	// while (true) {
	// 	std::vector<std::string> inputs(total_allowed_arguments,"");

	// 	std::string input;
	// 	getline(std::cin, input);
	// 	std::istringstream input_stream(input);

	// 	int argument = 0;
	// 	while ((input_stream >> inputs[argument]) && (argument < total_allowed_arguments)) {
	// 		ROS_INFO("Argument %d: %s", argument+1, inputs[argument].c_str());
	// 		argument++;
	// 	}

	// 	if (inputs[0].compare("exit") == 0) {
	// 		break;
	// 	} else if (inputs[0].compare("help") == 0) {
	// 		ROS_INFO("List of allowed arguments:");
	// 		ROS_INFO("module <file_name> <move_group> <execute> <save>");
	// 		ROS_INFO("module <file_name_1> <move_group_1> two_arms <file_name_2> <move_group_2> <execute> <save>");
	// 		ROS_INFO("file <file_name> <move_group> <execute> <save>");
	// 		ROS_INFO(" ");
	// 		ROS_INFO("File names should NOT include file extensions (*.txt, *.mod, etc.)");
	// 		ROS_INFO("<execute> and <save> must be either \"true\" or \"false\".");
	// 		ROS_INFO("  - They indicate if the trajectories should be executed and/or saved respectively.");
	// 		ROS_INFO("To exit the program, type: \"exit\"");
	// 	} 
	// }


	// // both_arms.startStateMonitor();
	// sleep(0.5);
	// displayJointValues(both_arms);
	// sleep(0.5);
	// displayJointValues(right_arm);
	// sleep(0.5);

	// ros::ServiceClient service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	//std::string file_name = argv[1];

	// kdl_kinematics_plugin::KDLKinematicsPlugin *ik_right = new kdl_kinematics_plugin::KDLKinematicsPlugin;


	boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader;
	kinematics::KinematicsBasePtr ik_right;

	kinematics_loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));

    // instantiate our plugin
    std::string plugin_name = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    try
    {
      ik_right = kinematics_loader->createInstance(plugin_name);
    }
    catch(pluginlib::PluginlibException& ex)//handle the class failing to load
    {
      ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
      return false;
    }

	bool success; 
	success = ik_right->initialize("/robot_description", "right_arm", right_arm.getPoseReferenceFrame(), right_arm.getEndEffectorLink(), 0.001);
	ROS_INFO("IK initialize status: %s", success?"success":"failed");

	trajectoryJoints jointTrajectory;
	trajectoryPoses poseTrajectory;

	std::string dataType = getFileDataType(inputFile);
	if (dataType.compare("joints") == 0) {
		jointTrajectory = getTrajectoryJoints(right_arm, inputFile);
	} else if (dataType.compare("poses") == 0) {
		poseTrajectory = getTrajectoryPoses(right_arm, inputFile);
	}

	const double PI = 3.1416;
	std::vector<double> seed;
	std::vector<double> consistency(8, 0.0);
	std::vector<double> solution;
	moveit_msgs::MoveItErrorCodes error_code;

	for (int joint = 0; joint < consistency.size(); joint++) {
		if ((joint == 0) || (joint == 4) || (joint == 6)) {
			consistency[joint] = PI/4;
		} else if (joint == 2) {
			consistency[joint] = PI/32;
		} else {
			consistency[joint] = PI;
		}
	}

	std::vector<std::vector<int>> confdata;
	std::vector<double> joint_7_positions;

	std::vector<int> confdata_entry(3);
	confdata_entry[0] = 0;
	confdata_entry[1] = -2;
	confdata_entry[2] = -1;
	confdata.push_back(confdata_entry);
	joint_7_positions.push_back(-1.2217);

	setAxisConfigurations(right_arm, confdata[0], joint_7_positions[0], seed, true);

	success = ik_right->searchPositionIK(poseTrajectory.pose_right[0], seed, 5.0, consistency, solution, error_code); // 5.0, consistency, 
	ROS_INFO("IK calculation status: %s", success?"success":"failed");

	for (int joint = 0; joint < solution.size(); joint++) {
		ROS_INFO("Joint value: %.4f", solution[joint]);
	}

	gotoJoints(right_arm, solution);


	// geometry_msgs::PoseStamped pose_stamped;
	// geometry_msgs::Pose pose;
	// ROS_INFO("Pose reference frame: %s", right_arm.getPoseReferenceFrame().c_str());

	// //right_arm.setEndEffectorLink("yumi_link_7_r");

	// gotoGroupState(right_arm, "ready");

	// pose_stamped = right_arm.getCurrentPose();
	// pose = pose_stamped.pose;
	// ROS_INFO("Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f",
	// 		pose.position.x, pose.position.y, pose.position.z, 
	// 		pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	// // pose.position.x = 0.286144;
	// // pose.position.y = -0.16856;
	// // pose.position.z = 0.202755;
	// // pose.orientation.x = 0.86268;
	// // pose.orientation.y = -0.0420293;
	// // pose.orientation.z = 0.275756;
	// // pose.orientation.w = -0.421871;

	// std::vector<int> confdata(3);
	// confdata[0] = 0;
	// confdata[1] = -2;
	// confdata[2] = -1;

	// std::vector<double> joint_values_1 = computeIK(service_client, right_arm, pose, confdata);
	// displayJointValues(right_arm);
	// ROS_INFO("--------------------");
	// std::vector<double> joint_values_2 = computeIK(service_client, right_arm, pose);
	// sleep(4);

	// geometry_msgs::PoseStamped pose_stamped_2 = right_arm.getCurrentPose();
	// geometry_msgs::Pose pose_2 = pose_stamped.pose;
	// ROS_INFO("Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f",
	// 		pose_2.position.x, pose_2.position.y, pose_2.position.z, 
	// 		pose_2.orientation.w, pose_2.orientation.x, pose_2.orientation.y, pose_2.orientation.z);

	// gotoPose(right_arm, pose);

	// pose_stamped = right_arm.getCurrentPose();
	// pose = pose_stamped.pose;
	// ROS_INFO("Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f",
	// 		pose.position.x, pose.position.y, pose.position.z, 
	// 		pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

	// sleep(4);
	// gotoGroupState(right_arm, "ready");
	// sleep(4);







	// planner plans;
	// trajectoryJoints jointTrajectory;
	// trajectoryPoses poseTrajectory;

	// std::string dataType = getFileDataType(inputFile);
	// if (dataType.compare("joints") == 0) {
	// 	jointTrajectory = getTrajectoryJoints(right_arm, inputFile);
	// } else if (dataType.compare("poses") == 0) {
	// 	poseTrajectory = getTrajectoryPoses(right_arm, inputFile);
	// }

	// std::vector<std::vector<int>> confdata;
	// std::vector<double> joint_7_positions;

	// std::vector<int> confdata_entry(3);
	// confdata_entry[0] = 0;
	// confdata_entry[1] = -2;
	// confdata_entry[2] = -1;
	// confdata.push_back(confdata_entry);
	// joint_7_positions.push_back(-1.2217);

	// confdata_entry[0] = 0;
	// confdata_entry[1] = -1;
	// confdata_entry[2] = -2;
	// confdata.push_back(confdata_entry);
	// joint_7_positions.push_back(-1.7453);

	// confdata_entry[0] = 0;
	// confdata_entry[1] = -2;
	// confdata_entry[2] = -1;
	// confdata.push_back(confdata_entry);
	// joint_7_positions.push_back(-1.2217);

	// std::vector<std::string> group_states(3);
	// group_states[0] = "ready";
	// group_states[1] = "home";
	// group_states[2] = "ready";

	// for (int point = 0; point < poseTrajectory.totalPoints; point++) {
	// 	ROS_INFO("Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f",
	// 		poseTrajectory.pose_right[point].position.x, poseTrajectory.pose_right[point].position.y, poseTrajectory.pose_right[point].position.z, 
	// 		poseTrajectory.pose_right[point].orientation.w, poseTrajectory.pose_right[point].orientation.x, poseTrajectory.pose_right[point].orientation.y, poseTrajectory.pose_right[point].orientation.z);
	// 	std::vector<double> joint_values_3 = computeIK(service_client, right_arm, poseTrajectory.pose_right[point], confdata[point], joint_7_positions[point]);
	// 	ROS_INFO("--------------------");
	// 	std::vector<double> joint_values_4 = computeIK(service_client, right_arm, poseTrajectory.pose_right[point]);

	// 	if (joint_values_3.size() > 0) {

	// 		int currentJoint = 0;
	// 		std::vector<double> joint_values(8);
	// 		for (int joint = 0; joint < joint_values_3.size(); joint++) {
	// 			if (((joint > 8) && (joint < 16)) || (joint == 17)) {
	// 				joint_values[currentJoint] = joint_values_3[joint];
	// 				ROS_INFO("Joint value: %.4f", joint_values[currentJoint]);
	// 				currentJoint++;
	// 			}
	// 		}

	// 		gotoJoints(right_arm, joint_values_3);
	// 		sleep(5);
	// 		gotoGroupState(right_arm, group_states[point]);
	// 		sleep(5);
	// 		gotoJoints(right_arm, joint_values_3);
	// 		sleep(5);
	// 	}
	// 	//preplanTrajectory(right_arm, module.poses[point], module.confdata[point], node_handle);
	// 	// gotoPose(right_arm, poseTrajectory.pose_right[point]);
	// 	// sleep(4);
	// 	// gotoGroupState(right_arm, group_states[point]);
	// 	// sleep(4);
	// 	//sleep(2);
	// 	ROS_INFO("====================");
	// }







	// dataType = getFileDataType(inputFile);
	// if (dataType.compare("joints") == 0) {
	// 	jointTrajectory = getTrajectoryJoints(left_arm, inputFile);
	// } else if (dataType.compare("poses") == 0) {
	// 	poseTrajectory = getTrajectoryPoses(left_arm, inputFile);
	// }

	// for (int point = 0; point < poseTrajectory.totalPoints; point++) {
	// 	ROS_INFO("Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f",
	// 		poseTrajectory.pose_left[point].position.x, poseTrajectory.pose_left[point].position.y, poseTrajectory.pose_left[point].position.z, 
	// 		poseTrajectory.pose_left[point].orientation.w, poseTrajectory.pose_left[point].orientation.x, poseTrajectory.pose_left[point].orientation.y, poseTrajectory.pose_left[point].orientation.z);
	// 	std::vector<double> joint_values_3 = computeIK(service_client, left_arm, poseTrajectory.pose_left[point]);
	// 	//preplanTrajectory(right_arm, module.poses[point], module.confdata[point], node_handle);
	// 	gotoPose(left_arm, poseTrajectory.pose_left[point]);
	// 	//sleep(2);
	// }


	// RAPIDModuleData module = getYuMiLeadThroughData(file_name);
	// for (int point = 0; point < 5; point++) {
	// 	ROS_INFO("Name: %s | Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f | Configuration: %d, %d, %d, %d | Joint 7: %.4f",
	// 		module.pose_names[point].c_str(),
	// 		module.poses[point].position.x, module.poses[point].position.y, module.poses[point].position.z, 
	// 		module.poses[point].orientation.w, module.poses[point].orientation.x, module.poses[point].orientation.y, module.poses[point].orientation.z, 
	// 		module.confdata[point][0], module.confdata[point][1], module.confdata[point][2], module.confdata[point][3], 
	// 		module.joint7_positions[point]);
	// 	std::vector<double> joint_values_1 = computeIK(service_client, right_arm, module.poses[point], module.confdata[point]);
	// }
	// // ROS_INFO("------------");
	// // for (int point = 0; point < 5; point++) {
	// // 	ROS_INFO("Name: %s | Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f | Configuration: %d, %d, %d, %d | Joint 7: %.4f",
	// // 		module.pose_names[point].c_str(),
	// // 		module.poses[point].position.x, module.poses[point].position.y, module.poses[point].position.z, 
	// // 		module.poses[point].orientation.w, module.poses[point].orientation.x, module.poses[point].orientation.y, module.poses[point].orientation.z, 
	// // 		module.confdata[point][0], module.confdata[point][1], module.confdata[point][2], module.confdata[point][3], 
	// // 		module.joint7_positions[point]);
	// // 	std::vector<double> joint_values_2 = computeIK(service_client, right_arm, module.poses[point], module.joint7_positions[point]);
	// // 	//ROS_INFO("---------");
	// // 	//std::vector<double> joint_values_3 = computeIK(service_client, right_arm, module.poses[point]);
	// // 	//ROS_INFO("=========");
	// // }
	// ROS_INFO("------------");
	// for (int point = 0; point < 5; point++) {
	// 	ROS_INFO("Name: %s | Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f | Configuration: %d, %d, %d, %d | Joint 7: %.4f",
	// 		module.pose_names[point].c_str(),
	// 		module.poses[point].position.x, module.poses[point].position.y, module.poses[point].position.z, 
	// 		module.poses[point].orientation.w, module.poses[point].orientation.x, module.poses[point].orientation.y, module.poses[point].orientation.z, 
	// 		module.confdata[point][0], module.confdata[point][1], module.confdata[point][2], module.confdata[point][3], 
	// 		module.joint7_positions[point]);
	// 	std::vector<double> joint_values_3 = computeIK(service_client, right_arm, module.poses[point]);
	// 	//preplanTrajectory(right_arm, module.poses[point], module.confdata[point], node_handle);
	// 	//gotoPose(right_arm, module.poses[point]);
	// 	//sleep(2);
	// }

	// std::string file = yumiScriptsDirectory + "paths/" + argv[2] + ".txt"; // concatenate full path
	// trajectoryPoses poses = getTrajectoryPoses(right_arm, file);

	// for (int point = 0; point < poses.totalPoints; point++) {
	// 	std::vector<double> joint_values = computeIK(service_client, right_arm, poses.pose_right[point]);
	// 	gotoPose(right_arm, poses.pose_right[point]);
	// }

	/* TRY WITH POSES PATH FILE */


	// NodeHandle node_handle("~");

	// std::string file_name = argv[1];
	// RAPIDModuleData module = getYuMiLeadThroughData(file_name);

	// planningInterface::MoveGroup right_arm("right_arm");
	// for (int i = 0; i < 5; i++) {
	// 	moveit_msgs::MotionPlanResponse motion_plan = preplanTrajectory(right_arm, module.poses[i], module.confdata[i], node_handle);
	// }




	// // Start a background "spinner" for node to process ROS messages
	// AsyncSpinner spinner(1);
	// spinner.start();

	// // Define Move Groups
	// planningInterface::MoveGroup left_arm("left_arm");
	// planningInterface::MoveGroup right_arm("right_arm");
	// planningInterface::MoveGroup both_arms("both_arms");

	// // Set Planning Parameters
	// left_arm.setNumPlanningAttempts(5); // set number of planning attempts for left arm
	// right_arm.setNumPlanningAttempts(5); // set number of planning attempts for right arm
	// both_arms.setNumPlanningAttempts(5); // set number of planning attempts for both arms

	// // Get Trajectory From File and Create Plans
	// planner plans;
	// //plans = retrieveBag("test", "test");
	// std::string dataType = getFileDataType(inputFile);
	// if (dataType.compare("joints") == 0) {
	// 	trajectoryJoints jointTrajectory = getTrajectoryJoints(both_arms, inputFile);
	// 	if (jointTrajectory.groupName.compare("") != 0) {
	// 		plans = generatePlans(both_arms, jointTrajectory);
	// 		//createBag("test","test",plans);
	// 	}
	// } else if (dataType.compare("poses") == 0) {
	// 	trajectoryPoses poseTrajectory = getTrajectoryPoses(both_arms, inputFile);
	// 	if (poseTrajectory.groupName.compare("") != 0) { 
	// 		plans = generatePlans(left_arm, right_arm, both_arms, poseTrajectory);
	// 		//createBag("test", "test", plans);
	// 	}
	// }

	// // Execute Plans
	// bool success = executePlans(both_arms, plans);

	shutdown();

	return 0;
}

/* -----------------------------------------------
   --------- READ WINDOWS 10 APP FILES -----------
   ----------------------------------------------- */
std::vector<double> computeIK(ServiceClient serviceIK, planningInterface::MoveGroup& group, geometry_msgs::Pose pose_unstamped, std::vector<int> confdata, double joint_7_position) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-04
*/
	std::vector<std::string> joint_names = group.getActiveJoints();
	std::vector<double> currentJointValues = group.getCurrentJointValues();
	
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = group.getPoseReferenceFrame();
	pose.pose = pose_unstamped;
	
	// ROS_INFO("Working 1");
	
	moveit_msgs::GetPositionIK::Request serviceRequest;
	moveit_msgs::GetPositionIK::Response serviceResponse;

	std::vector<double> tolerance_pose(3, 0.001);
	std::vector<double> tolerance_angle(3, 0.001);
	moveit_msgs::Constraints pose_constraints  = kinematic_constraints::constructGoalConstraints(group.getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	moveit_msgs::Constraints joint_constraints = setAxisConfigurations(group, confdata, joint_7_position, currentJointValues, true);
	moveit_msgs::Constraints goal_constraints  = kinematic_constraints::mergeConstraints(pose_constraints, joint_constraints);

	ROS_INFO("--------------------");
	ROS_INFO("Seed Joint Values");
	for (int i = 0; i < currentJointValues.size(); i++) {
		ROS_INFO("Joint: %s | Seed: %.4f", joint_names[i].c_str(), currentJointValues[i]);
	}
	ROS_INFO("--------------------");

	serviceRequest.ik_request.group_name       = group.getName();
	serviceRequest.ik_request.avoid_collisions = true;
	serviceRequest.ik_request.attempts         = 20; 
	serviceRequest.ik_request.constraints      = goal_constraints;
	serviceRequest.ik_request.pose_stamped     = pose;
	serviceRequest.ik_request.robot_state.joint_state.name     = joint_names;
	serviceRequest.ik_request.robot_state.joint_state.position = currentJointValues;
	
	// ROS_INFO("Working 2");
	
	serviceIK.call(serviceRequest,serviceResponse);
	
	// ROS_INFO("Working 3");
	
	std::vector<double> jointValues     = serviceResponse.solution.joint_state.position;
	std::vector<std::string> jointNames = serviceResponse.solution.joint_state.name;
	for (int joint = 0; joint < jointValues.size(); joint++) {
		ROS_INFO("Joint %s: %.4f",jointNames[joint].c_str(),jointValues[joint]);
	}
	
	ROS_INFO("Joint value size %lu",jointValues.size());
	ROS_INFO("Error code: %d",serviceResponse.error_code.val);
	
	return jointValues;
}

// std::vector<double> computeIK(ServiceClient serviceIK, planningInterface::MoveGroup& group, geometry_msgs::Pose pose_unstamped, double confdata) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
// 	DATE CREATED: 2016-07-18
// */
// 	std::vector<std::string> joint_names = group.getActiveJoints();
// 	std::vector<double> currentJointValues = group.getCurrentJointValues();
	
// 	geometry_msgs::PoseStamped pose;
// 	pose.header.frame_id = group.getPoseReferenceFrame();
// 	pose.pose = pose_unstamped;
	
// 	// ROS_INFO("Working 1");
	
// 	moveit_msgs::GetPositionIK::Request serviceRequest;
// 	moveit_msgs::GetPositionIK::Response serviceResponse;

// 	serviceRequest.ik_request.group_name       = group.getName();
// 	serviceRequest.ik_request.avoid_collisions = true;
// 	serviceRequest.ik_request.attempts         = 10;
// 	serviceRequest.ik_request.constraints      = setExternalAxisConstraint(confdata);
// 	serviceRequest.ik_request.pose_stamped     = pose;
// 	serviceRequest.ik_request.robot_state.joint_state.name     = joint_names;
// 	serviceRequest.ik_request.robot_state.joint_state.position = currentJointValues;
	
// 	// ROS_INFO("Working 2");
	
// 	serviceIK.call(serviceRequest,serviceResponse);
	
// 	// ROS_INFO("Working 3");
	
// 	std::vector<double> jointValues     = serviceResponse.solution.joint_state.position;
// 	std::vector<std::string> jointNames = serviceResponse.solution.joint_state.name;
// 	for (int joint = 0; joint < jointValues.size(); joint++) {
// 		ROS_INFO("Joint %s: %.4f",jointNames[joint].c_str(),jointValues[joint]);
// 	}
	
// 	ROS_INFO("Joint value size %lu",jointValues.size());
// 	ROS_INFO("Error code: %d",serviceResponse.error_code);
	
// 	return jointValues;
// }

std::vector<double> computeIK(ServiceClient serviceIK, planningInterface::MoveGroup& group, geometry_msgs::Pose pose_unstamped) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-18
*/
	std::vector<std::string> joint_names = group.getActiveJoints();
	std::vector<double> joint_values = group.getCurrentJointValues();
	
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = group.getPoseReferenceFrame();
	pose.pose = pose_unstamped;
	
	// ROS_INFO("Working 1");
	
	moveit_msgs::GetPositionIK::Request serviceRequest;
	moveit_msgs::GetPositionIK::Response serviceResponse;

	std::vector<double> tolerance_pose(3, 0.001);
	std::vector<double> tolerance_angle(3, 0.001);
	moveit_msgs::Constraints pose_constraints  = kinematic_constraints::constructGoalConstraints(group.getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	//moveit_msgs::Constraints joint_constraints = setAxisConfigurations(group, confdata);
	//moveit_msgs::Constraints goal_constraints  = kinematic_constraints::mergeConstraints(pose_constraints, joint_constraints);

	serviceRequest.ik_request.group_name       = group.getName();
	serviceRequest.ik_request.avoid_collisions = true;
	serviceRequest.ik_request.attempts         = 10;
	serviceRequest.ik_request.pose_stamped     = pose;
	// serviceRequest.ik_request.constraints      = pose_constraints;
	serviceRequest.ik_request.robot_state.joint_state.name     = joint_names;
	serviceRequest.ik_request.robot_state.joint_state.position = joint_values;
	// serviceRequest.ik_request.ik_link_name = "yumi_link_7_r";
	// 	serviceRequest.ik_request.timeout = Duration(10000000);
	
	// ROS_INFO("Working 2");
	
	serviceIK.call(serviceRequest,serviceResponse);
	
	// ROS_INFO("Working 3");
	
	std::vector<double> jointValues     = serviceResponse.solution.joint_state.position;
	std::vector<std::string> jointNames = serviceResponse.solution.joint_state.name;
	for (int joint = 0; joint < jointValues.size(); joint++) {
		ROS_INFO("Joint %s: %.4f", jointNames[joint].c_str(), jointValues[joint]);
	}
	
	ROS_INFO("Joint value size: %lu", jointValues.size());
	ROS_INFO("Error code: %d", serviceResponse.error_code.val);
	
	return jointValues;
}

// moveit_msgs::MotionPlanResponse preplanTrajectory(planningInterface::MoveGroup& group, geometry_msgs::Pose pose_prev, std::vector<int> confdata, ros::NodeHandle node_handle) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
// 	DATE CREATED: 2016-07-02
// */
// 	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
// 	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
// 	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
// 	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

// 	planning_interface::MotionPlanRequest motion_request;
// 	planning_interface::MotionPlanResponse motion_response;
// 	// geometry_msgs::PoseStamped poses_left  = pose_trajectory.pose_left;
// 	// geometry_msgs::PoseStamped poses_right = pose_trajectory.pose_right;

// 	geometry_msgs::PoseStamped pose;
// 	pose.header.frame_id = group.getEndEffectorLink();
// 	pose.pose = pose_prev;

// 	std::vector<double> tolerance_pose(3, 0.001);
// 	std::vector<double> tolerance_angle(3, 0.001);

// 	ROS_INFO("End effector: %s", group.getEndEffectorLink().c_str());
// 	moveit_msgs::Constraints pose_constraints  = kinematic_constraints::constructGoalConstraints(group.getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
// 	moveit_msgs::Constraints joint_constraints = setAxisConfigurations(group, confdata);
// 	moveit_msgs::Constraints goal_constraints  = kinematic_constraints::mergeConstraints(pose_constraints, joint_constraints);

// 	motion_request.group_name = group.getName();
// 	motion_request.goal_constraints.push_back(goal_constraints);
	
// 	group.setStartStateToCurrentState();
// 	planning_pipeline->generatePlan(planning_scene, motion_request, motion_response);
// 	if (motion_response.error_code_.val != motion_response.error_code_.SUCCESS) {
// 		ROS_ERROR("Could not compute plan successfully");
// 		ROS_WARN("Error code: %d", motion_response.error_code_.val);
// 	} else {
// 		ROS_INFO("Planning Successful!");
// 	}

// 	moveit_msgs::MotionPlanResponse motion_plan;
//   	motion_response.getMessage(motion_plan);

//   	return motion_plan;
// }


moveit_msgs::Constraints setAxisConfigurations(planningInterface::MoveGroup& group, std::vector<int> confdata, double joint_7_position, std::vector<double>& joint_values_seed, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-02

	INPUT(S):
	  > confdata - ABB naming convention for axis configurations for a specified target
*/
	// Initialize Variables
	const double PI = 3.1416;
	const double TOLERANCE_CF146  = PI/4 + 0.01; // 45 degree tolerance, with a slight extra room for positions that are PI/2, for max and min of joint position constraint
	const double TOLERANCE_AXIS_7 = PI/36; // 5 degree tolerance for the seventh axis
	const double WEIGHT = 1;
	const std::vector<int> config_joints = { 0, 4, 6 }; // axis configuration joints (joint 1, 4, and 6 starting with index 0)
	const std::vector<int> config_joints_cfx = { 1, 3, 5 }; // axis configuration joints for CFX in confdata (joint 2, 3, and 5 starting with index 0)

	std::string group_name = group.getName(); // get the name of the provided group
	std::vector<std::string> active_joint_names = group.getActiveJoints(); // get the names of all the active joints
	std::vector<double> joint_values = group.getCurrentJointValues();

	// Add Joint Constraints for Axis 1, 4, and 6
	moveit_msgs::Constraints goal_constraints;
	for (int constraint = 0; constraint < config_joints.size(); constraint++) {
		moveit_msgs::JointConstraint joint_constraint;
		joint_constraint.joint_name      = active_joint_names[config_joints[constraint]];
		joint_constraint.position        = (confdata[constraint] * PI/2) + TOLERANCE_CF146 - 0.01;
		joint_constraint.tolerance_above = TOLERANCE_CF146;
		joint_constraint.tolerance_below = TOLERANCE_CF146;
		joint_constraint.weight          = WEIGHT;

		joint_values[config_joints[constraint]] = joint_constraint.position;

		if (debug) {
			ROS_INFO("Name: %s, Window: %.4f to %.4f", joint_constraint.joint_name.c_str(), joint_constraint.position-TOLERANCE_CF146, joint_constraint.position+TOLERANCE_CF146);
		}

		goal_constraints.joint_constraints.push_back(joint_constraint);
	}

	// Add Joint Constraints for Axis 7
	moveit_msgs::JointConstraint joint_constraint;
	joint_constraint.joint_name      = active_joint_names[2];
	joint_constraint.position        = joint_7_position;
	joint_constraint.tolerance_above = TOLERANCE_AXIS_7;
	joint_constraint.tolerance_below = TOLERANCE_AXIS_7;
	joint_constraint.weight          = WEIGHT; 

	joint_values[2] = joint_constraint.position;

	if (debug) {
		ROS_INFO("Name: %s, Window: %.4f to %.4f", joint_constraint.joint_name.c_str(), joint_constraint.position-TOLERANCE_AXIS_7, joint_constraint.position+TOLERANCE_AXIS_7);
	}

	goal_constraints.joint_constraints.push_back(joint_constraint);

	// Add Joint Constraints for CFX from confdata If Present
	if (confdata.size() == 4) {
		// Need to add code for CFX from confdata
	}

	joint_values_seed = joint_values;
	return goal_constraints;
}

moveit_msgs::Constraints setExternalAxisConstraint(double external_axis_position) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-18
*/
	const double PI = 3.1416;
	const double TOLERANCE = PI/8;
	const double WEIGHT = 1;

	moveit_msgs::Constraints goal_constraints;
	moveit_msgs::JointConstraint joint_constraint;

	joint_constraint.joint_name = "yumi_joint_7_r";
	joint_constraint.position   = external_axis_position;
	joint_constraint.tolerance_above = TOLERANCE;
	joint_constraint.tolerance_below = TOLERANCE;
	joint_constraint.weight = WEIGHT;

	goal_constraints.joint_constraints.push_back(joint_constraint);

	return goal_constraints;
}

/* -----------------------------------------------
   --------- READ WINDOWS 10 APP FILES -----------
   ----------------------------------------------- */
trajectoryPoses convertRAPIDtoPoseTrajectory(RAPIDModuleData& module, planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-08
*/
	// Initialize Variables
	trajectoryPoses pose_trajectory;

	//if (group.getName.compare("right_arm") == 0) || 

	pose_trajectory.groupName = group.getName();
	// Don't know the intended group

	return pose_trajectory;
}

trajectoryPoses convertRAPIDtoPoseTrajectory(RAPIDModuleData& left_module, RAPIDModuleData& right_module) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-08
*/
	trajectoryPoses pose_trajectory;
}

RAPIDModuleData getYuMiLeadThroughData(std::string file_name, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-07

	ASSUMPTIONS: Modules do not start with a OpenHand or CloseHand command.
*/
	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string empty; // variable to store unwanted data from file
	std::string module_name; // variable to store the module name
	std::string data_type; // variable to store the data type from the provided input file
	std::string robtarget;

	geometry_msgs::Pose pose;
	std::vector<geometry_msgs::Pose> poses;
	std::vector<int> confdata(4);
	std::vector<std::vector<int>> confdatas;
	double joint_7_position;
	std::vector<double> joint_7_positions;
	bool gripper_position = 1; // variable to indicate the gripper state - 0) closed, 2) open
	std::vector<bool> gripper_positions; // vector of gripper states - 0) closed, 1) open
	
	std::string store_point_name;
	std::vector<std::string> store_point_names;
	std::string move_point_name; // variable to store the point name
	std::vector<std::string> move_point_names; // vector of point names

	int line_index = 0; // variable to indicate what line is currently being pulled
	int execution_line = 0;
	bool robtarget_end = 0; // variable to indicate if still storing robtargets
	bool success = true; // flag to indicate if any error occurred

 	std::string input_file = yumiScriptsDirectory + "demo/" + file_name + ".mod";

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically// Check Optional Variable(s)
	if (debug) { // if this function is being run in debug mode
		ROS_WARN("Retrieving YuMi file is being run in debug mode."); // notify the user that this function is being run in debug mode
	}

	ROS_INFO("Getting YuMi file data."); // notify the user that the file data type for the provided input file is being retrived
	tic();

	// Get Provided File Data Type
	std::ifstream text_file(input_file.c_str()); // open provided text file
	if (text_file.is_open()) { // if the file was able to successfully open
		// Get Module Name for Provided File
		if (std::getline(text_file, line)) { // if the file exists and contains a first line
			line_index++; // increment the current line counter
			if (debug) { ROS_INFO("----- Pulling line: %d -----",line_index); }

			std::istringstream first_line(line); // create a string steam element for delimiting data by spaces
			first_line >> empty >> module_name; // get the module name
			ROS_INFO("Module name: %s",module_name.c_str());
			while (std::getline(text_file, line)) {
				line_index++; // increment the current line counter
				if (debug) { ROS_INFO("----- Pulling line: %d -----",line_index); }

				std::istringstream current_line(line); // create a string steam element for delimiting data by spaces
				current_line >> data_type;

				if (data_type.compare(0, 1, "P") == 0) {
					robtarget_end = 1;
					if (debug) { ROS_INFO("Reading from Main function."); }
				} else if (!(robtarget_end)) {
					current_line >> empty >> data_type;
					if (data_type.compare(0, 1, "r") == 0) {
						current_line >> store_point_name >> empty >> robtarget;
						getRobtargetData(robtarget, pose, confdata, joint_7_position, debug);

						poses.push_back(pose);
						confdatas.push_back(confdata);
						joint_7_positions.push_back(joint_7_position);
						store_point_names.push_back(store_point_name);

						if (debug) { ROS_INFO("Pulled robtarget for position: %s. Stored data.",store_point_name.c_str()); }
					} else if (data_type.compare(0, 1, "s") == 0) {
						std::string yumi_app;
						current_line >> yumi_app;
						if (yumi_app.compare(0, 8, "YuMi_App") != 0) {
							ROS_ERROR("The provided file was not generated from the YuMi Windows 10 App.");
							success = false;
							break;
						}
					}
				} else if (data_type.compare(0, 1, "E") == 0) {
					ROS_INFO("End of file reached.");
					break;
				} else {
					execution_line++;
					if (data_type.compare("MoveSync") == 0) {
						current_line >> move_point_name;
						move_point_name = move_point_name.substr(0, move_point_name.length()-1);
					} else if (data_type.compare("OpenHand;") == 0) {
						if (execution_line == 1) {
							ROS_ERROR("The first command cannot be \"OpenHand\". ");
							success = false;
							break;
						}
						gripper_position = 1;
					} else if (data_type.compare("CloseHand;") == 0) {
						if (execution_line == 1) {
							ROS_ERROR("The first command cannot be \"CloseHand\". ");
							success = false;
							break;
						}
						gripper_position = 0;
					}
					move_point_names.push_back(move_point_name);
					gripper_positions.push_back(gripper_position);
					if (debug) { ROS_INFO("Main function line: %d, pose name: %s, gripper position (open is 1): %d", execution_line, move_point_name.c_str(), gripper_position); }
				}
			}
		} else { // if the provided file is empty
			ROS_ERROR("The provided file is empty."); // notify the user that the provided file is empty
			success = false; // indicate that there was an error
		}
		text_file.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s",input_file.c_str()); // notify user of the supplied file
		success = false; // indicate that there was an error
	}

	RAPIDModuleData module;
	if (success) {
		if (debug) { ROS_INFO(" ===================="); }
		ROS_INFO("Storing trajectory. Total trajectory points to store: %lu", move_point_names.size());

		module.moduleName     = module_name;
		module.pose_names     = move_point_names;
		module.totalPoints    = move_point_names.size();
		//module.only_move_sync = only_move_sync;

		int stored_location;
		int total_stored_points = store_point_names.size();
		for (int point = 0; point < module.totalPoints; point++) {
			std::string point_name = module.pose_names[point];
			for (int position = 0; position < total_stored_points; position++) {
				if (store_point_names[position].compare(point_name) == 0) {
					stored_location = position;
					break;
				}
			}
			module.gripper_positions.push_back(gripper_positions[point]);
			module.poses.push_back(poses[stored_location]);
			module.confdata.push_back(confdatas[stored_location]);
			module.joint7_positions.push_back(joint_7_positions[stored_location]);

			if (debug) { ROS_INFO("Trajectory point %d was stored successfully", point+1); }
		}
	} else {
		ROS_ERROR("Not able to parse YuMi lead through file.");
	}

	if (debug) {
		ROS_INFO(" ====================");
		ROS_INFO("Module Name: %s", module.moduleName.c_str());

		std::string current_point_name;
		std::string previous_point_name = "";
		for (int point = 0; point < module.totalPoints; point++) {
			current_point_name = module.pose_names[point].c_str();
			if (previous_point_name.compare(current_point_name) == 0) {
				if (module.gripper_positions[point] == 1) {
					ROS_INFO("Open Hand");
				} else {
					ROS_INFO("Close Hand");
				}
			} else {
				ROS_INFO("Name: %s | Position: %.4f, %.4f, %.4f | Orientation: %.4f, %.4f, %.4f, %.4f | Configuration: %d, %d, %d, %d | Joint 7: %.4f",
					module.pose_names[point].c_str(),
					module.poses[point].position.x, module.poses[point].position.y, module.poses[point].position.z, 
					module.poses[point].orientation.w, module.poses[point].orientation.x, module.poses[point].orientation.y, module.poses[point].orientation.z, 
					module.confdata[point][0], module.confdata[point][1], module.confdata[point][2], module.confdata[point][3], 
					module.joint7_positions[point]);
			}
			previous_point_name = current_point_name;
		}
	}

	ROS_INFO("Processing time: %.4f",toc()); // display processing time
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
	return module;
}

void getRobtargetData(std::string robtarget, geometry_msgs::Pose& pose, std::vector<int>& confdata, double& joint_7_position, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
	DATE CREATED: 2016-07-07
*/
	// Initialize Variables
	const double PI = 3.1415927;
	const double mm_to_m = 1.0/1000.0;
	const double deg_to_rad = PI / 180;
	std::string robtarget_search = "0123456789.-+E";

	bool pose_pulled = 0;
	std::size_t start_char = 2;
	std::size_t end_char = 0;
	std::size_t E_location;
	int element = 0;

	while (!(pose_pulled)) {
		element++;

		end_char = robtarget.find_first_not_of(robtarget_search, start_char);
		std::string current_val_string = robtarget.substr(start_char, (end_char-start_char));

		double current_val;
		E_location = current_val_string.find_first_of("E");
		if (E_location != -1) {
			double exponent = std::stod(current_val_string.substr(E_location+1, current_val_string.length()-1-E_location), nullptr);
			current_val = std::stod(current_val_string.substr(0,E_location), nullptr) * pow(10.0, exponent);
		} else {
			current_val = std::stod(current_val_string, nullptr);
		}

		if (element == 1) { pose.position.x = current_val * mm_to_m; }
		else if (element == 2) { pose.position.y = current_val * mm_to_m; }
		else if (element == 3) { pose.position.z = current_val * mm_to_m; }
		else if (element == 4) { pose.orientation.w = current_val; }
		else if (element == 5) { pose.orientation.x = current_val; }
		else if (element == 6) { pose.orientation.y = current_val; }
		else if (element == 7) { pose.orientation.z = current_val; }
		else if (element == 8) { confdata[0] = ((int)current_val); }
		else if (element == 9) { confdata[1] = ((int)current_val); }
		else if (element == 10) { confdata[2] = ((int)current_val); }
		else if (element == 11) { confdata[3] = ((int)current_val); }
		else if (element == 12) { joint_7_position = current_val * deg_to_rad; }
		else if (element >= 13) { break; }

		start_char = robtarget.find_first_of(robtarget_search, end_char);
	}

	if (debug) {
		ROS_INFO("Pose: %.4f, %.4f, %.4f | %.4f, %.4f, %.4f, %.4f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
		ROS_INFO("Confdata size: %lu, values: %d, %d, %d, %d", confdata.size(), confdata[0], confdata[1], confdata[2], confdata[3]);
		ROS_INFO("Joint 7 position: %.4f", joint_7_position);
	}
}

/* -----------------------------------------------
   -------- BAG READNG/WRITING FUNCTIONS ---------
   ----------------------------------------------- */
void createBag(std::string bagName, std::string topicName, planner& plan) {
	// check if bag name is empty

	/* =============================================== */
	/* NEED TO ENSURE .bag IS NOT INCLUDED IN BAG NAME */
	/* =============================================== */
	// Notify User of Storing a Bag
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
	ROS_INFO("Creating ROS bag for group: %s", plan.groupName.c_str());
	
	// Create Bag with Provided Name
	bagName = yumiScriptsDirectory + "bags/" + bagName + ".bag";
	rosbag::Bag bag(bagName, rosbag::bagmode::Write);

	// Store Planner Data into Bag Message
	yumi_scripts::PrePlan prePlan_msg;
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
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
}

planner retrieveBag(std::string bagName, std::string topicName) {
	// Need to check if provided topic exists

	// Notify User of Retrieving a Bag
	bagName = yumiScriptsDirectory + "bags/" + bagName + ".bag";
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
	ROS_INFO("Retrieving ROS bag at location: %s", bagName.c_str());

	// Get Bag with Specified Topic
	rosbag::Bag bag(bagName, rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topicName));

	// Store Bag Elements
	yumi_scripts::PrePlan::ConstPtr prePlan_msg;
	BOOST_FOREACH(rosbag::MessageInstance const m, view) {
		prePlan_msg = m.instantiate<yumi_scripts::PrePlan>();
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
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

	return plan; // return the plan
}

/* -----------------------------------------------
   --- LEAD THROUGH READNG/WRITING FUNCTIONS -----
   ----------------------------------------------- */
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
 
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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


 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

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
	
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

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
	
 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically

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

 	ROS_INFO(">--------------------"); // add a cutoff to group together and make it easier to see info from this function specifically
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
	std::vector<std::string> jointNames; // vector to store the joint names
	std::vector<double> jointValues; // vector to contain joint values
	std::vector<double>::size_type totalJoints; // unsigned long int signifying total joints in move group

	// Get Joint Names, Joint Values and Total Joints
	jointNames  = group.getActiveJoints(); // get active joint names
	jointValues = group.getCurrentJointValues(); // get joint values
	totalJoints = jointValues.size(); // get amount of joints within move group

	// Iterate Through Joints and Display Positions
	ROS_INFO("Total joints: %lu",totalJoints);
	for (int i = 0; i < totalJoints; i++) {
		ROS_INFO("Joint: %s | Position: %f", jointNames[i].c_str(), jointValues[i]);
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


