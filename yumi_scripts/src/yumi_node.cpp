// INCLUDES
#include <fstream>
#include <chrono>
#include <cmath>
#include <ctime>
#include <math.h>
#include <sstream>
#include <stack>
#include <string>
#include <thread>

#include <boost/foreach.hpp>
#include <geometry_msgs/Pose.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>

#include <yumi_scripts/JointMsg.h>
#include <yumi_scripts/JointConfigMsg.h>
#include <yumi_scripts/PoseConfigMsg.h>
#include <yumi_scripts/ModuleDataMsg.h>
#include <yumi_scripts/ModuleMsg.h>
#include <yumi_scripts/PlannerMsg.h>

// NAMESPACE DECLARATION
namespace planningInterface = moveit::planning_interface;

// STRUCTURES
struct poseConfig {
    std::string group_name;
    geometry_msgs::Pose pose;
    std::vector<int> confdata;
    double external_axis_position;
    bool gripper_attached;
    double gripper_position;
};

struct jointConfig {
    std::string group_name;
    std::vector<double> joint_values;
    int total_joints;
};

struct RAPIDModuleData {
    std::string group_name;
    std::string module_name;
    std::vector<std::string> pose_names;
    std::vector<poseConfig> pose_configs;
    bool gripper_attached = false;
    int total_points;
};

struct trajectoryJoints {
    std::string group_name;
    std::string intended_group;
    std::vector<std::vector<double>> joints;
    int total_joints;
    int total_points;
};

struct planner {
    std::string group_name;
    std::vector<planningInterface::MoveGroup::Plan> plans;
    int total_plans;
};

// GLOBAL VARIABLE
const double GRIPPER_OPEN_POSITION = 0.024; // gripper open position (m)
const double GRIPPER_CLOSED_POSITION = 0.0; // gripper closed position (m)
const std::string YUMI_SCRIPTS_DIRECTORY = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/";
const std::string YUMI_ROSBAG_TOPIC_NAME = "yumi";

bool module_lock(false), joint_config_lock(false), pose_config_lock(false), command_lock(false);
bool new_module(false), new_joint_config(false), new_pose_config(false), new_command(false);
yumi_scripts::ModuleDataMsg global_module_left, global_module_right;
std::string global_command, global_group_name;
poseConfig global_pose_config;
jointConfig global_joint_config;
std::vector<yumi_scripts::JointMsg> global_joint_trajectory;

// BASIC FUNCTIONS
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

// FUNCTION PROTOTYPES
/* Subscriber Functions */
void moduleCallback(const yumi_scripts::ModuleMsg::ConstPtr&);
void jointConfigCallback(const yumi_scripts::JointConfigMsg::ConstPtr&);
void poseConfigCallback(const yumi_scripts::PoseConfigMsg::ConstPtr&);
void commandCallback(const std_msgs::String::ConstPtr&);

/* Message Conversion and Checking Functions */
RAPIDModuleData convertModuleMsgToModuleData(yumi_scripts::ModuleDataMsg&, bool debug = false);
void appendMasterModule(RAPIDModuleData&, RAPIDModuleData&, std::string, bool debug = false);
void appendMasterJointTrajectory(std::vector<trajectoryJoints>&, std::vector<yumi_scripts::JointMsg>&, std::string, bool debug = false);
bool checkModulesLineUp(RAPIDModuleData&, RAPIDModuleData&, bool debug = false);

/* File Checking and ROS Bag Functions */
bool fileExists(std::string, std::string, std::string);
void savePlanner(std::string, planner&);

/* RAPID Module Writing Functions */
bool renameModulePoses(RAPIDModuleData&, RAPIDModuleData&, RAPIDModuleData&, RAPIDModuleData&, bool debug = false);
poseConfig getAxisConfigurations(std::vector<double>, bool debug = false);
void writeToFile(std::string, RAPIDModuleData&, bool debug = false);
void writeToFile(std::string, RAPIDModuleData&, RAPIDModuleData&, bool debug = false);
std::string getRobtargetOutputLine(std::string, poseConfig&, bool debug = false);

/* Plan and Execution Functions */
trajectoryJoints fillJointTrajectory(planningInterface::MoveGroup&, std::vector<trajectoryJoints>&, bool debug = false);
bool generatePlans(planner&, planningInterface::MoveGroup&, trajectoryJoints&, bool debug = false);
bool executePlans(planningInterface::MoveGroup&, planner&, bool debug = false);

// MAIN FUNCTION
int main(int argc, char **argv) {

    ros::init(argc, argv, "moveit_interface");
    ros::NodeHandle node_handle("yumi");
    ros::NodeHandle node_handle_move_group("move_group");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // INITIALIZE VARIABLES FROM PARAMETER SERVER
    bool debug = false;
    if (!(ros::param::get("/yumi/yumi_node/debug", debug))) {
        ROS_INFO(">--------------------");
        ROS_ERROR("Not able to get debug parameter from parameter server.");
        ROS_WARN("The configuration parameters must have not been loaded before the YuMi node was started.");
        return 0;
    }

    // INITIALIZE MOVE GROUPS FOR THE LEFT ARM, RIGHT ARM, AND BOTH ARMS
    planningInterface::MoveGroup left_arm("left_arm");
    left_arm.startStateMonitor();
    left_arm.getCurrentJointValues();
    planningInterface::MoveGroup right_arm("right_arm");
    right_arm.startStateMonitor();
    left_arm.getCurrentJointValues();
    planningInterface::MoveGroup both_arms("both_arms");
    both_arms.startStateMonitor();
    left_arm.getCurrentJointValues();

    // SET MOVE GROUP REFRENCE FRAMES AND END EFFECTORS
    std::string pose_reference_frame = "yumi_body";
    left_arm.setPoseReferenceFrame(pose_reference_frame);
    right_arm.setPoseReferenceFrame(pose_reference_frame);
    both_arms.setPoseReferenceFrame(pose_reference_frame);

    // SET UP SUBSCRIBERS AND PUBLISHERS
    ros::Subscriber module_sub       = node_handle.subscribe("modules", 10, moduleCallback);
    ros::Subscriber joint_config_sub = node_handle.subscribe("joint_configs", 1000, jointConfigCallback);
    ros::Subscriber pose_config_sub  = node_handle.subscribe("pose_configs", 1000, poseConfigCallback);
    ros::Subscriber command_sub      = node_handle.subscribe("commands", 1000, commandCallback);
    ros::Publisher display_publisher = node_handle_move_group.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);

    // SETUP NODE STRUCTURES AND VARIABLES
    std::string group_name;
    RAPIDModuleData module_left_master, module_right_master, module_left, module_right, module_empty;
    std::vector<trajectoryJoints> joint_trajectories_master, joint_trajectory_empty;
    std::vector<yumi_scripts::JointMsg> joint_trajectory_msg;
    poseConfig pose_config;
    jointConfig joint_config;
    std::string command;

    planner plans_master, plans_empty;

    bool success_generated_plans = false;

    module_left_master.total_points  = 0;
    module_right_master.total_points = 0;

    // CHECK IF GRIPPERS ARE ATTACHED
    if (left_arm.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
        module_left_master.gripper_attached = true;
    }
    if (right_arm.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
        module_right_master.gripper_attached = true;
    }

    // WAIT FOR COMMANDS
    int system_return = std::system("clear");
    ROS_INFO("Waiting for messages.");
    while (ros::ok()) {
    /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
        DATE CREATED: 2016-08-22
    */
        if (new_module) {
        /* If the user is sending a module and joint trajectory to add to the master modules and joint trajectory */
            new_module  = false;

            // COPY GLOBAL VARIABLES/STRUCTURES TO LOCAL
            group_name = global_group_name;
            joint_trajectory_msg = global_joint_trajectory;
            if (group_name.compare("both_arms") == 0) {
                module_left  = convertModuleMsgToModuleData(global_module_left, debug);
                module_right = convertModuleMsgToModuleData(global_module_right, debug);
            } else if (group_name.compare("left_arm") == 0) {
                module_left  = convertModuleMsgToModuleData(global_module_left, debug);
            } else if (group_name.compare("right_arm") == 0) {
                module_right = convertModuleMsgToModuleData(global_module_right, debug);
            } else {
                module_lock = false;
                ROS_INFO(" ");
                ROS_INFO(">--------------------");
                ROS_ERROR("Group name in received module message not recognized.");
                ROS_WARN("Recognized group names: left_arm, right_arm, both_arms");
                ROS_WARN("Received group name: %s", group_name.c_str());
                ROS_WARN("Not storing module(s) and joint trajectory due to error.");
                continue;
            }

            // RELEASE DATA LOCK
            module_lock = false;
            if (!debug) { ROS_INFO(" "); }
            ROS_INFO(">--------------------");
            ROS_INFO("Received module for group: %s.", group_name.c_str());

            // CHECK JOINT AND POSE TRAJECTORY SIZES ADD DATA TO MASTER STRUCTURES
            bool size_mismatch = false;
            if (group_name.compare("both_arms") == 0) {
                if ((joint_trajectory_msg.size() != module_left.pose_names.size()) || (joint_trajectory_msg.size() != module_right.pose_names.size())) {
                    size_mismatch = true;
                } else {
                    bool success = checkModulesLineUp(module_left, module_right, debug);
                    if (success) {
                        appendMasterModule(module_left_master, module_left, group_name, debug);
                        appendMasterModule(module_right_master, module_right, group_name, debug);
                    } else {
                        ROS_INFO(">--------------------");
                        ROS_WARN("Not storing modules and joint trajectory due to error.");
                        continue;
                    }
                }
            } else if (group_name.compare("left_arm") == 0) {
                if (joint_trajectory_msg.size() != module_left.pose_names.size()) {
                    size_mismatch = true;
                } else {
                    appendMasterModule(module_left_master, module_left, group_name, debug);
                }
            } else if (group_name.compare("right_arm") == 0) {
                if (joint_trajectory_msg.size() != module_right.pose_names.size()) {
                    size_mismatch = true;
                } else {
                    appendMasterModule(module_right_master, module_right, group_name, debug);
                }
            }

            if (size_mismatch) {
                ROS_ERROR("The received joint trajectory and module data do not have the same size.");
                ROS_WARN("Joint trajectory size: %lu", joint_trajectory_msg.size());
                ROS_WARN("Module sizes: %lu (left) %lu (right)", module_left.pose_names.size(), module_right.pose_names.size());
                ROS_WARN("Not storing module(s) and joint trajectory due to error.");
                continue;
            }

            appendMasterJointTrajectory(joint_trajectories_master, joint_trajectory_msg, group_name, debug);

            if (debug) { ROS_INFO(">--------------------"); }
            ROS_INFO("Successfully appended received module(s) to master module.");
        }

        if (new_joint_config) {
        /* If the user would like to store a new joint configuration into the joint trajectory and modules */
            new_joint_config  = false;
            joint_config = global_joint_config;
            joint_config_lock = false;

            ROS_INFO(" ");
            ROS_INFO(">--------------------");
            ROS_INFO("Received joint configuration for group: %s.", global_group_name.c_str());
        }

        if (new_pose_config) {
        /* If the user would like to store a new pose configuration into the joint trajectory and modules */
            new_pose_config  = false;
            pose_config = global_pose_config;
            pose_config_lock = false;

            ROS_INFO(" ");
            ROS_INFO(">--------------------");
            ROS_INFO("Received pose configuration for group: %s.", global_group_name.c_str());
        }

        if (new_command) {
        /* If the user is sending a command to the YuMi node */
            new_command  = false;

            // COPY GLOBAL VARIABLES/STRUCTURES TO LOCAL THEN RELEASE DATA LOCK
            command = global_command;
            command_lock = false;

            ROS_INFO(" ");
            ROS_INFO(">--------------------");
            ROS_INFO("Received command: %s", command.c_str());

            if (command.compare("create_plan") == 0) {
            /* If the user would like to create a joint trajectory plan for the current joint trajectory */
                plans_master = plans_empty; // reset plans_master to empty

                // FILL MASTER TRAJECTORY AND GENERATE PLAN
                trajectoryJoints joint_trajectory_filled = fillJointTrajectory(both_arms, joint_trajectories_master, debug);
                bool success = generatePlans(plans_master, both_arms, joint_trajectory_filled, debug);

                ROS_INFO("....................");
                if (success) {
                    ROS_INFO("Successfully created joint trajectory plan. Total trajectory points: %d", plans_master.total_plans);
                    success_generated_plans = true;
                } else {
                    ROS_ERROR("Not able to generate joint trajectory plans due to error.");
                    ROS_WARN("Not storing any joint trajectory plans.");
                    success_generated_plans = false;
                }

            } else if (command.compare("show_trajectory") == 0) {
            /* If the user would like to show the current trajectory of the plan in RViz */
                if (success_generated_plans) {
                    ROS_INFO("Showing trajectories.");

                    moveit_msgs::DisplayTrajectory display_trajectory;
                    display_trajectory.trajectory_start = plans_master.plans[0].start_state_;

                    for (int plan = 0; plan < plans_master.total_plans; plan++) {
                        display_trajectory.trajectory.push_back(plans_master.plans[plan].trajectory_);
                    }
                    display_publisher.publish(display_trajectory);
                } else {
                    ROS_ERROR("Plans have not been successfully created yet.");
                    ROS_WARN("To create plans for trajectories, send the command: create plan");
                }

            } else if (command.compare(0, 11, "save_rosbag") == 0) {
            /* If the user would like to save the current joint trajectory plan */
                if (!success_generated_plans) {
                    ROS_ERROR("There are no valid joint trajectory plans to save.");
                    ROS_WARN("A plan needs to be constructed successfully before saving.");
                    continue;
                }

                if ((command.length() > 11) && (command.compare(11, 2, ":=") == 0)) {
                    std::string file_name = command.substr(13);
                    RAPIDModuleData module_left_save, module_right_save;
                    bool file_exists;

                    file_exists = fileExists(file_name, "bags", ".bag");
                    if (file_exists) {
                        ROS_ERROR("File name for ROS bag already exists.");
                        ROS_WARN("Please re-supply a file name that does not already exists.");
                        continue;
                    }

                    ROS_INFO(">--------------------");
                    ROS_INFO("Saving joint trajectory plan.");
                    savePlanner(file_name, plans_master);
                    ROS_INFO("Successfully saved joint trajectory plan.");
                } else {
                    ROS_ERROR("ROS bag name not provided or not provided correctly.");
                    ROS_WARN("Correct format: save_rosbag:=file_name");
                    ROS_WARN("replace file_name with the name of ROS bag file.");
                }

            } else if (command.compare(0, 11, "save_module") == 0) {
            /* If the user would like to save the current module data */
                if ((module_left_master.total_points == 0) && (module_right_master.total_points == 0)) {
                    ROS_ERROR("There are no trajecotry points to save.");
                    continue;
                }

                if ((command.length() > 11) && (command.compare(11, 2, ":=") == 0)) {
                    std::string file_name = command.substr(13);
                    RAPIDModuleData module_left_save, module_right_save;
                    bool file_exists, file_exists_left, file_exists_right;

                    file_exists       = fileExists(file_name, "modules", ".mod");
                    file_exists_left  = fileExists(file_name + "_left", "modules", ".mod");
                    file_exists_right = fileExists(file_name + "_right", "modules", ".mod");
                    if ((file_exists) || (file_exists_left) || (file_exists_right)) {
                        ROS_ERROR("File name for module already exists.");
                        ROS_WARN("Please re-supply a file name that does not already exists.");
                        continue;
                    }

                    ROS_INFO(">--------------------");
                    ROS_INFO("Renaming module poses.");
                    bool success = renameModulePoses(module_left_master, module_left_save, module_right_master, module_right_save, debug);
                    if (success) {
                        ROS_INFO("Successfully renamed module poses.");

                        ROS_INFO(">--------------------");
                        ROS_INFO("Writing module(s) to file.");
                        if (module_left_save.total_points == 0) {
                            ROS_INFO("Stored module for the left arm is empty, only writing module for right arm");
                            ROS_INFO("Module name: %s", file_name.c_str());
                            writeToFile(file_name, module_right_save, debug);
                        } else if (module_right_save.total_points == 0) {
                            ROS_INFO("Stored module for the right arm is empty, only writing module for left arm");
                            ROS_INFO("Module name: %s", file_name.c_str());
                            writeToFile(file_name, module_left_save, debug);
                        } else {
                            ROS_INFO("Stored modules for both arms.");
                            ROS_INFO("Module name: %s_left (left), %s_right (right)", file_name.c_str(), file_name.c_str());
                            writeToFile(file_name, module_left_save, module_right_save, debug);
                        }
                    } else {
                        ROS_INFO("....................");
                        ROS_INFO("Not saving modules due to error");
                        continue;
                    }
                    ROS_INFO(">--------------------");
                    ROS_INFO("Successfully saved module(s).");
                } else {
                    ROS_ERROR("Module name not provided or not provided correctly.");
                    ROS_WARN("Correct format: save_module:=file_name");
                    ROS_WARN("replace file_name with the name of module.");
                }

            } else if (command.compare("update_params") == 0) {
            /* If the use would like for the internal parameters to be updated from the parameter server */
                bool previous_debug = debug;
                if (!(ros::param::get("/yumi/yumi_node/debug", debug))) {
                    ROS_INFO(">--------------------");
                    ROS_ERROR("Not able to get debug parameter from parameter server.");
                    ROS_WARN("The configuration parameters must have not been loaded to the ROS parameter server.");
                } else {
                    if (previous_debug != debug) {
                        ROS_INFO("Updated parameters: debug = %s", (debug)?"true":"false");
                    } else {
                        ROS_INFO("All parameters are already up to date.");
                    }
                }

            } else if (command.compare("clear_data") == 0) {
            /* If the user would like to clear all data */
                joint_trajectories_master = joint_trajectory_empty;
                module_left_master  = module_empty;
                module_right_master = module_empty;

                module_left_master.total_points  = 0;
                module_right_master.total_points = 0;

                plans_master = plans_empty;
                success_generated_plans = false;

                ROS_INFO("Cleared modules and joint trajectories.");

            } else {
                ROS_ERROR("Command not recognized.");
                ROS_WARN("Recognized commands: create_plan, show_trajectory");
            }

            /* NEED TO CHECK IF THE SENT COMMAND IS RECOGNIZED */
        }

        ros::spinOnce();
    }

    return 0;
}


/* ------------------------------------------------------------ */
/* ------------------- SUBSCRIBER FUNCTIONS ------------------- */
/* ------------------------------------------------------------ */

void moduleCallback(const yumi_scripts::ModuleMsg::ConstPtr& module_msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (!module_lock) {
        module_lock = true;
        new_module  = true;
        global_group_name       = module_msg->group_name;
        global_joint_trajectory = module_msg->joint_trajectory;
        if (global_group_name.compare("both_arms") == 0) {
            global_module_left  = module_msg->module_left;
            global_module_right = module_msg->module_right;
        } else if (global_group_name.compare("left_arm") == 0) {
            global_module_left  = module_msg->module_left;
        } else if (global_group_name.compare("right_arm") == 0) {
            global_module_right = module_msg->module_right;
        }
    } else {
        /* NOTIFY USER OF ISSUE */
    }
}

void jointConfigCallback(const yumi_scripts::JointConfigMsg::ConstPtr& joint_config_msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (!joint_config_lock) {
        joint_config_lock = true;
        new_joint_config  = true;
        /* STORE JOINT CONFIG CORRECTLY */
    } else {
        /* NOTIFY USER OF ISSUE */
    }
}

void poseConfigCallback(const yumi_scripts::PoseConfigMsg::ConstPtr& pose_config_msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (!pose_config_lock) {
        pose_config_lock = true;
        new_pose_config  = true;
        /* STORE POSE CONFIG CORRECTLY */
    } else {
        /* NOTIFY USER OF ISSUE */
    }
}

void commandCallback(const std_msgs::String::ConstPtr& command_msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (!command_lock) {
        command_lock = true;
        new_command  = true;
        global_command = command_msg->data;
    } else {
        /* NOTIFY USER OF ISSUE */
    }
}

/* ------------------------------------------------------------ */
/* -------- MESSAGE CONVERSION AND CHECKING FUNCTIONS --------- */
/* ------------------------------------------------------------ */

RAPIDModuleData convertModuleMsgToModuleData(yumi_scripts::ModuleDataMsg& module_data_msg, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    // INITIALIZE VARIABLES
    RAPIDModuleData module;
    poseConfig pose_config;

    if (debug) { 
        ROS_INFO(" ");
        ROS_INFO(">--------------------");
        ROS_INFO("(debug) Converting received module message to module data strcuture.");
    }

    // CONVERT MODULE DATA MESSAGE TO RAPID MODULE DATA STRUCTURE
    module.group_name       = module_data_msg.group_name;
    module.total_points     = module_data_msg.total_points;
    module.gripper_attached = module_data_msg.gripper_attached;

    for (int pose = 0; pose < module.total_points; pose++) {
        pose_config.pose     = module_data_msg.pose_configs[pose].pose;
        pose_config.confdata = module_data_msg.pose_configs[pose].confdata;
        pose_config.external_axis_position = module_data_msg.pose_configs[pose].external_axis_position; 
        if (module.gripper_attached) {
            pose_config.gripper_position       = module_data_msg.pose_configs[pose].gripper_position;
        }

        module.pose_configs.push_back(pose_config);
        module.pose_names.push_back(module_data_msg.pose_names[pose]);
    }

    if (debug) { ROS_INFO("(debug) Successfully converted received module message to module data strcuture."); }

    return module;
}

void appendMasterModule(RAPIDModuleData& master_module, RAPIDModuleData& module, std::string group_name, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (debug) { 
        ROS_INFO(">--------------------");
        ROS_INFO("(debug) Appending received module to master module.");
    }

    if (group_name.compare("both_arms") == 0) {
        for (int pose = 0; pose < module.total_points; pose++) {
            master_module.pose_names.push_back(module.pose_names[pose]);
            master_module.pose_configs.push_back(module.pose_configs[pose]);
        }
    } else {
        if (debug) { ROS_INFO("(debug) Assuming only non-syncronized movements since only one module was sent."); }

        for (int pose = 0; pose < module.total_points; pose++) {
            module.pose_names[pose].replace(0, 1, "p");
            master_module.pose_names.push_back(module.pose_names[pose]);
            master_module.pose_configs.push_back(module.pose_configs[pose]);
        }
    }

    master_module.total_points = master_module.pose_configs.size();

    if (debug) { ROS_INFO("(debug) Successfully appended received module to master module."); }
}

void appendMasterJointTrajectory(std::vector<trajectoryJoints>& master_joint_trajectory, std::vector<yumi_scripts::JointMsg>& joint_trajectory_msg, std::string group_name, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (debug) { 
        ROS_INFO(">--------------------");
        ROS_INFO("(debug) Appending received joint trajectory to master joint trajectory.");
    }

    trajectoryJoints joint_trajectory;

    joint_trajectory.group_name   = group_name;
    joint_trajectory.total_joints = joint_trajectory_msg[0].joint_values.size();
    joint_trajectory.total_points = joint_trajectory_msg.size();

    for (int point = 0; point < joint_trajectory.total_points; point++) {
        joint_trajectory.joints.push_back(joint_trajectory_msg[point].joint_values);
    }

    master_joint_trajectory.push_back(joint_trajectory);

    if (debug) {
        ROS_INFO("(debug) Trajectory info: %s (group name) %d (total joints) %d (trajectory size)", joint_trajectory.group_name.c_str(), joint_trajectory.total_joints, joint_trajectory.total_points);
        ROS_INFO("(debug) Successfully appended received joint trajectory to master joint trajectory."); 
    }
}

bool checkModulesLineUp(RAPIDModuleData& module_1, RAPIDModuleData& module_2, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    // INITIALIZE VARIABLES
    int index_1(0), index_2(0), index(0);
    bool success = true;
    bool extra_move_sync = false;

    // CHECK IF MODULES ARE LINED UP
    if (debug) { 
        ROS_INFO(">--------------------");
        ROS_INFO("(debug) Checking if received modules are lined up."); 
    }

    while (true) {

        if ((index_1 == module_1.total_points) && (index_2 == module_2.total_points)) {
            break;
        } else if (index_1 == module_1.total_points) {
            index_1--;
            if (module_2.pose_names[index_2].compare(0, 1, "s") == 0) {
                extra_move_sync = true;
                success = false;
            }
        } else if (index_2 == module_2.total_points) {
            index_2--;
            if (module_1.pose_names[index_1].compare(0, 1, "s") == 0) {
                extra_move_sync = true;
                success = false;
            }
        } else if (module_1.pose_names[index_1].compare(module_2.pose_names[index_2]) != 0) {
            if ((module_1.pose_names[index_1].compare(0, 1, "p") == 0) && (module_2.pose_names[index_2].compare(0, 1, "p") != 0)) {
            /* If the first module is going to a non-syncronized point (designated by p#) and the second module is going to a syncronized point */
                if ((index_2-1 < 0) || (module_2.pose_names[index_2].compare(module_2.pose_names[index_2-1]) != 0)) {
                /* If previous point for the second module is not the same syncronized point as the current point or if this is the first pose for the second module */
                    success = false;
                }
            } else if ((module_1.pose_names[index_1].compare(0, 1, "p") != 0) && (module_2.pose_names[index_2].compare(0, 1, "p") == 0)) {
            /* If the second module is going to a non-syncronized point (designated by p#) and the first module is going to a syncronized point */
                if ((index_1-1 < 0) || (module_1.pose_names[index_1].compare(module_1.pose_names[index_1-1]) != 0)) {
                /* If previous point for the first module is not the same syncronized point as the current point of if this is the first pose for the first module */
                    success = false;
                }
            } else if ((module_1.pose_names[index_1].compare(0, 1, "s") == 0) && (module_2.pose_names[index_2].compare(0, 1, "s") == 0)) {
            /* If both modules are at a MoveSync command, but the robtarget names are different meaning neither of them are able to continue execution */
                success = false;
            }
        }

        index_1++;
        index_2++;
        index++;

        if (debug) {
            std::string index_adjustment = "";
            if (index < 10) { index_adjustment = " "; }
            if (module_1.pose_names[index_1-1].length() < 3) {
                ROS_INFO("(debug) Line: %d%s | Module 1 point: %s  | Module 2 point: %s", index, index_adjustment.c_str(), module_1.pose_names[index_1-1].c_str(), module_2.pose_names[index_2-1].c_str());
            } else {
                ROS_INFO("(debug) Line: %d%s | Module 1 point: %s | Module 2 point: %s", index, index_adjustment.c_str(), module_1.pose_names[index_1-1].c_str(), module_2.pose_names[index_2-1].c_str());
            }
            
        }

        if (!success) {
            ROS_ERROR("From: checkModulesLineUp(module_1, module_2, debug)");
            if ((extra_move_sync) && (index_1 == module_1.total_points)) { ROS_WARN("There are more MoveSync commands in the second module than the first."); }
            else if ((extra_move_sync) && (index_2 == module_2.total_points)) { ROS_WARN("There are more MoveSync commands in the first module than the second."); }
            else { ROS_ERROR("Modules received are not lined up."); }
            return success;
        }
    }

    if (debug) { ROS_INFO("(debug) Modules are lined up."); }
    return success;
}

/* ------------------------------------------------------------ */
/* ----------- FILE CHECKING AND ROS BAG FUNCTIONS ------------ */
/* ------------------------------------------------------------ */

bool fileExists(std::string file_name, std::string folder_name, std::string file_type) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-05

    PURPOSE: Determine if the provided file name exists in the provided folder within the
             yumi_scripts ROS package. The function returns true if the file exists and 
             false if the file does not exist.

    INSTRUCTIONS: Input the file name and the folder that contains the file. The inputted
                  folder must be located within the yumi_scripts ROS package. The function
                  returns true if the file exists and false if the file does not exist.
*/
    // INITIALIZE VARIABLES
    std::string file_path = YUMI_SCRIPTS_DIRECTORY + folder_name + "/" + file_name + file_type;
    std::ifstream file(file_path);

    // CHECK IF FILE EXISTS
    if (file.good()) {
        return true;
    } else {
        return false;
    }
}

void savePlanner(std::string bag_name, planner& plans) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-05

    PRUPOSE: The purpose of this function is to store the provided planner in a ROS bag with the
             provided bag name. All bags will be stored within the "yumi_scripts" package in a folder
             called "bags". A custom messages called PlannerMsg was created in order to store the 
             appropriate data from the planner. The idea behind storing the planner structure is
             that all data is first converted into a planner structure, and if the planner was
             successfully create then this data can be stored and retrieved later for execution.
             All information is stored within a topic called "yumi".

    INSTRUCTIONS: Provide the bag name and the planner structure containing successfully created
                  plans for the desired data. The function will create the bag with the topic name
                  of "yumi".

    NOTE: There is not error checking in this function since it is expected that all the relevent
          information and requirements mentioned above are satisfied.
*/  
    // INITIALIZE VARIABLES
    std::string bag_path = YUMI_SCRIPTS_DIRECTORY + "bags/" + bag_name + ".bag";
    yumi_scripts::PlannerMsg planner_msg;

    // CREATE ROSBAG
    rosbag::Bag bag(bag_path, rosbag::bagmode::Write);

    // CONSTRUCT PLANNER MESSAGE
    planner_msg.group_name  = plans.group_name;
    planner_msg.start_state = plans.plans[0].start_state_;
    planner_msg.total_plans = plans.total_plans;
    for (int plan = 0; plan < plans.total_plans; plan++) {
        planner_msg.trajectory.push_back(plans.plans[plan].trajectory_);
    }

    // WRITE TO BAG
    bag.write(YUMI_ROSBAG_TOPIC_NAME, ros::Time::now(), planner_msg);
    bag.close();

    ROS_INFO("Planner successfully saved for group %s.", plans.group_name.c_str());
    ROS_INFO("Bag location: %s", bag_path.c_str());
}

/* ------------------------------------------------------------ */
/* -------------- RAPID MODULE WRITING FUNCTIONS -------------- */
/* ------------------------------------------------------------ */

bool renameModulePoses(RAPIDModuleData& module_1, RAPIDModuleData& module_1_new, RAPIDModuleData& module_2, RAPIDModuleData& module_2_new, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-23
*/
    // INITIALIZE VARIABLES
    const double GRIPPER_MOVE_TOL = 0.0005; // 0.5mm tolerance to be considered a gripper movement

    int module_1_index(0), module_2_index(0), index(0);
    int move_index_1(1), move_index_2(1), move_sync_index(1);
    bool module_1_only(false), module_2_only(false), modules_move_sync(false), modules_move_nosync(false);
    bool repeated_point_1, repeated_point_2;

    double previous_gripper_position, current_gripper_position;

    std::string module_1_pose_name, module_2_pose_name;
    RAPIDModuleData empty_module;

    module_1_new = empty_module; // ensure module is empty
    module_2_new = empty_module; // ensure module is empty

    // TRANSFER DATA FROM ORIGINAL MODULES TO NEW ONES
    module_1_new.group_name = module_1.group_name;
    module_1_new.gripper_attached = module_1.gripper_attached;
    
    if (debug) { 
        ROS_INFO("....................");
        ROS_INFO("(debug) Reassigning pose names to modules.");
        ROS_INFO("_____ (debug) Aligned Modules with New Pose Names _____");
    }

    // CHECK IF MODULES ARE LINED UP
    while (true) {
        // DETERMINE WHICH POSE NAMES WILL BE UPDATED ON THIS ITERATION
        if ((module_1_index == module_1.total_points) && (module_2_index == module_2.total_points)) {
            break;
        } else if ((module_1_index == module_1.total_points) || (module_2_index == module_2.total_points)) {
        /* If the modules are not lined up properly*/
            ROS_ERROR("From: renameModulePoses(module_1, module_1_new, module_2, module_2_new, debug)");
            ROS_WARN("This is due to some internal error from incorrect error checking with incoming data.");
            return false;
        } else if (module_1.pose_names[module_1_index].compare(0, 1, "p") == 0) {
            if (module_2.pose_names[module_2_index].compare(0, 1, "p") == 0) {
                modules_move_nosync = true;
            } else {
                module_1_only = true;
            }
        } else if (module_2.pose_names[module_2_index].compare(0, 1, "p") == 0) {
            if (module_1.pose_names[module_1_index].compare(0, 1, "p") == 0) {
                modules_move_nosync = true;
            } else {
                module_2_only = true;
            }
        } else {
            modules_move_sync = true;
        }

        // SET FLAG TO INDICATE IF THERE IS A POSSIBILITY OF A GRIPPER MOVEMENT
        repeated_point_1 = ((module_1_index > 0) && (module_1.pose_names[module_1_index].compare(module_1.pose_names[module_1_index-1]) == 0));
        repeated_point_2 = ((module_2_index > 0) && (module_2.pose_names[module_2_index].compare(module_2.pose_names[module_2_index-1]) == 0));

        // REASSIGN POSE NAMES
        if ((module_1_only) || (modules_move_nosync) || (modules_move_sync)) {
            if (repeated_point_1) {
                previous_gripper_position = module_1.pose_configs[module_1_index-1].gripper_position;
                current_gripper_position  = module_1.pose_configs[module_1_index].gripper_position;
                if ((module_1.gripper_attached) && (std::abs(previous_gripper_position-current_gripper_position) > GRIPPER_MOVE_TOL)) {
                    if (current_gripper_position > previous_gripper_position) {
                        module_1_new.pose_names.push_back("OpenHand");
                        module_1_new.pose_configs.push_back(module_1.pose_configs[module_1_index]);
                    } else {
                        module_1_new.pose_names.push_back("CloseHand");
                        module_1_new.pose_configs.push_back(module_1.pose_configs[module_1_index]);
                    }
                }
            } else {
                if (modules_move_sync) {
                    if (!repeated_point_2) {
                        module_1_new.pose_names.push_back("s" + std::to_string(move_sync_index));
                        module_1_new.pose_configs.push_back(module_1.pose_configs[module_1_index]);
                    }
                } else {
                    module_1_new.pose_names.push_back("p" + std::to_string(move_index_1));
                    module_1_new.pose_configs.push_back(module_1.pose_configs[module_1_index]);
                    move_index_1++;
                }
            }
        } 

        if ((module_2_only) || (modules_move_nosync) || (modules_move_sync)) {
            if (repeated_point_2) {
                previous_gripper_position = module_2.pose_configs[module_2_index-1].gripper_position;
                current_gripper_position  = module_2.pose_configs[module_2_index].gripper_position;
                if ((module_2.gripper_attached) && (std::abs(previous_gripper_position-current_gripper_position) > GRIPPER_MOVE_TOL)) {
                    if (current_gripper_position > previous_gripper_position) {
                        module_2_new.pose_names.push_back("OpenHand");
                        module_2_new.pose_configs.push_back(module_2.pose_configs[module_2_index]);
                    } else {
                        module_2_new.pose_names.push_back("CloseHand");
                        module_2_new.pose_configs.push_back(module_2.pose_configs[module_2_index]);
                    }
                }
            } else {
                if (modules_move_sync) {
                    if (!repeated_point_1) {
                        module_2_new.pose_names.push_back("s" + std::to_string(move_sync_index));
                        module_2_new.pose_configs.push_back(module_2.pose_configs[module_2_index]);
                        move_sync_index++;
                    }
                } else {
                    module_2_new.pose_names.push_back("p" + std::to_string(move_index_2));
                    module_2_new.pose_configs.push_back(module_2.pose_configs[module_2_index]);
                    move_index_2++;
                }
            }
        }

        module_1_index++;
        module_2_index++;

        index++;
        if (debug) { 
        /* The output lines differ by command to try to line up consecutive outputs which is the reasoning behind the multiple if statements */
            std::string output_adjustment = "";
            if (index < 10) {
                output_adjustment = " ";
            }
            if (module_1_only) {
                if (module_1_new.pose_names.back().compare(0, 1, "O") == 0) {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s  | Module 2: ---", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str());
                } else if (module_1_new.pose_names.back().compare(0, 1, "C") == 0) {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s | Module 2: ---", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str());
                } else if (module_1_new.pose_names.back().length() < 3) {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s        | Module 2: ---", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str());
                } else {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s       | Module 2: ---", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str());
                }
            } else if (module_2_only) {
                ROS_INFO("(debug) Line: %d%s | Module 1: ---      | Module 2: %s", index, output_adjustment.c_str(), module_2_new.pose_names.back().c_str());
            } else {
                if (module_1_new.pose_names.back().compare(0, 1, "O") == 0) {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s  | Module 2: %s", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str(), module_2_new.pose_names.back().c_str());
                } else if (module_1_new.pose_names.back().compare(0, 1, "C") == 0) {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s | Module 2: %s", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str(), module_2_new.pose_names.back().c_str());
                } else if (module_1_new.pose_names.back().length() < 3) {
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s        | Module 2: %s", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str(), module_2_new.pose_names.back().c_str());
                } else { 
                    ROS_INFO("(debug) Line: %d%s | Module 1: %s       | Module 2: %s", index, output_adjustment.c_str(), module_1_new.pose_names.back().c_str(), module_2_new.pose_names.back().c_str());
                }
            } 
        }

        module_1_only = false;
        module_2_only = false;
        modules_move_sync   = false;
        modules_move_nosync = false;
    }

    module_1_new.total_points = module_1_new.pose_configs.size();
    module_2_new.total_points = module_2_new.pose_configs.size();

    if (debug) { ROS_INFO("(debug) Successfully reassigned pose names for modules."); }
    return true;
}

poseConfig getAxisConfigurations(std::vector<double> joint_values, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-26

    PURPOSE: The purpose of this function is to get the axis configuration of the robot based on the
             supplied joint values. The axis configurations are based on the ABB convention, which is 
             shown under the variabe type "confdata" in the ABB RAPID reference manual. These 
             configurations are also described below.

    INSTRUCTIONS: The axis configraution is generated based on the provided joint values. If debug 
                  mode is set to true, then the current joint values with be displayed along with the 
                  confdata calculated from the current joint values.


    -------------------- OTHER INFORMATION --------------------

    ABB YuMi AXIS NAMING CONVENTION (from YuMi base to end effector): [1, 2, 7, 3, 4, 5, 6]

    ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

    CONFIGURATION SETS: -4 -> [-360, -270)   -3 -> [-270, -180)   -2 -> [-180, -90)   -1 -> [-90,   0)
                         0 -> [   0,   90)    1 -> [  90,  180)    2 -> [ 180, 270)    3 -> [270, 360)
        - The configuration values above are for joints 1, 4, and 6

    CFX: ABCD
        - A represents configuration for axis 5 and can be either (1) or (0)
            > (0) if axis 5 position >= 0 degrees, (1) if axis 5 position < 0 degrees
        - B represents configuration for axis 3 and can be either (1) or (0)
            > (0) if axis 3 position >= -90 degrees, (1) if axis 3 position < -90 degrees
        - C represents configuration for axis 2 and can be either (1) or (0)
            > (0) if axis 2 position >= 0 degrees, (1) if axis 2 position < 0 degrees
        - D represents the compatability bit, particulary used for linear movements
            > This value is not used for the IK solver

    Examples: (Compatibility bit assumed to be 0)
        - Axis 5 = 0 degrees, Axis 3 = -90 degrees, Axis 2 = 0 degrees   | cfx = 0000 or 0
        - Axis 5 = 0 degrees, Axis 3 = -91 degrees, Axis 2 = 0 degrees   | cfx = 0100 or 100
        - Axis 5 = -90 degrees, Axis 3 = 0 degrees, Axis 2 = -90 degrees | cfx = 1010

    EXTERNAL AXIS POSITION CONVENTION: [arm_angle, 9E+09, 9E+09, 9E+09, 9E+09, 9E+09]
*/
    if (debug) { ROS_INFO(">--------------------"); }

    // INITIALIZE VARIABLES
    const double PI = 3.14159;
    const double rad2deg = 180.0/PI;

    poseConfig pose_config;
    std::vector<double> joint_vaues;
    int axis_1_config, axis_4_config, axis_6_config, cfx = 0;

    if (debug) {
        ROS_INFO("_____ (debug) Provided joint positions _____");
        for (int joint = 0; joint < joint_values.size(); joint++) { ROS_INFO("%.5f", joint_values[joint]); } 
    }

    // GET AXIS CONFIGURATIONS
    axis_1_config = std::floor(joint_values[0] / (PI/2.0)); // joint 1 configuration value
    axis_4_config = std::floor(joint_values[4] / (PI/2.0)); // joint 4 configuration value
    axis_6_config = std::floor(joint_values[6] / (PI/2.0)); // joint 6 configuration value
    if (joint_values[5] < 0)    { cfx += 1000; } // joint 5 configuration value
    if (joint_values[3] < PI/2) { cfx += 100;  } // joint 3 configuration value
    if (joint_values[1] < 0)    { cfx += 10;   } // joint 2 configuration value

    pose_config.confdata.push_back(axis_1_config);
    pose_config.confdata.push_back(axis_4_config);
    pose_config.confdata.push_back(axis_6_config);
    pose_config.confdata.push_back(cfx);

    pose_config.external_axis_position = joint_values[2]; // need to update to arm angle calculation

    if (debug) {
        ROS_INFO("_____ (debug) Axis configuration for provided joint positions _____");
        ROS_INFO("Configuration: [%d, %d, %d, %d] | External Axis Position: [%0.5f]", 
            pose_config.confdata[0], pose_config.confdata[1], pose_config.confdata[2], pose_config.confdata[3], 
            pose_config.external_axis_position);
    }

    return pose_config;
}

void writeToFile(std::string output_file_name, RAPIDModuleData& module, bool debug) {
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
    std::string output_file_path = YUMI_SCRIPTS_DIRECTORY + "modules/" + output_file_name + ".mod";
    std::ofstream output_file;

    ROS_INFO("Writing to file at: %s", output_file_path.c_str());

    output_file.open(output_file_path.c_str(), std::ofstream::out | std::ofstream::app);

    // ADD HEADER TO FILE
    output_file << "MODULE " << output_file_name << std::endl;
    output_file << "LOCAL CONST string YuMi_App_Program_Version:=\"1.0.1\"; !Do not edit or remove this line!" << std::endl;
    
    // ADD ROBTARGETS
    if (debug) { ROS_INFO("_____ (debug) Adding Robtargets to File _____"); }
    for (int pose = module.pose_names.size()-1; pose >= 0; pose--) {
        if ((module.pose_names[pose].compare(0, 1, "C") != 0) && (module.pose_names[pose].compare(0, 1, "O") != 0)) {
            output_file << robtarget_prefix << getRobtargetOutputLine(module.pose_names[pose], module.pose_configs[pose], debug) << std::endl;
        }
    }

    output_file << "PROC main()" << std::endl;

    // ADD MAIN FUNCTION
    if (debug) { ROS_INFO("_____ (debug) Adding Main Function to File _____"); }
    for (int line = 0; line < module.pose_names.size(); line++) {
        if (module.pose_names[line].compare(0, 1, "p") == 0) {
            output_file << "Move " << module.pose_names[line] << ";" << std::endl;
            if (debug) { ROS_INFO("(debug) Move %s;", module.pose_names[line].c_str()); }
        } else if ((module.pose_names[line].compare(0, 1, "C") == 0) || (module.pose_names[line].compare(0, 1, "O") == 0)) {
            output_file << module.pose_names[line] << ";" << std::endl;
            if (debug) { ROS_INFO("(debug) %s;", module.pose_names[line].c_str()); }
        } else {
            output_file << "MoveSync " << module.pose_names[line] << ";" << std::endl;
            if (debug) { ROS_INFO("(debug) MoveSync %s;", module.pose_names[line].c_str()); }
        }
    }

    output_file << "ENDPROC" << std::endl << "ENDMODULE" << std::endl;

    output_file.close();
}

void writeToFile(std::string output_file_name, RAPIDModuleData& module_left, RAPIDModuleData& module_right, bool debug) {
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
    writeToFile(output_file_name_left, module_left, debug);

    // WRITE TO FILE RIGHT ARM
    if (debug) {
        ROS_INFO("_____ (debug) Writing Right Arm File _____");
    }
    writeToFile(output_file_name_right, module_right, debug);
}

std::string getRobtargetOutputLine(std::string pose_name, poseConfig& pose_config, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-27

    PURPOSE: The purpose of this function is to construct robtarget data according to the ABB
             convention and return the robtarget as a string. The appropriate unit conversions
             are also applied according to the unit conventions shown below. The ABB robtarget
             conventionis also shown below. If one of the output values are less than 1e-5, then
             by default those numbers will be outputted in scientific for using "e". ABB 
             convention uses "E" instead of "e" for scientific notation, so this is also checked
             before returning the output line.

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
    const int total_empty_values = 5;

    std::ostringstream output;
    std::string empty_value = "9E+09";

    // CONSTRUCT ROBTARGET
    output << pose_name << " := [[";
    output << pose_config.pose.position.x*m2mm << "," << pose_config.pose.position.y*m2mm << "," << pose_config.pose.position.z*m2mm << "],[";
    output << pose_config.pose.orientation.w << "," << pose_config.pose.orientation.x << "," << pose_config.pose.orientation.y << "," << pose_config.pose.orientation.z << "],[";
    output << pose_config.confdata[0] << "," << pose_config.confdata[1] << "," << pose_config.confdata[2] << "," << pose_config.confdata[3] << "],[";
    output << pose_config.external_axis_position*rad2deg;
    for (int i = 0; i < total_empty_values; i++) { output << "," << empty_value; }
    output << "]];";

    // CONVERT ANY 'e' to 'E'
    std::string output_string = output.str();
    for (int i = 0; i < output_string.length(); i++) {
        if (output_string.compare(i, 1, "e") == 0) {
            output_string.replace(i, 1, "E");
        }
    }

    if (debug) { ROS_INFO("(debug) %s", output_string.c_str()); }

    return output_string;
}

/* ------------------------------------------------------------ */
/* ------------- PLANNING AND EXECUTION FUNCTIONS ------------- */
/* ------------------------------------------------------------ */
trajectoryJoints fillJointTrajectory(planningInterface::MoveGroup& both_arms, std::vector<trajectoryJoints>& joint_trajectories, bool debug) {
/*  PROGRAMMER: Frederick Wachter
    DATE CREATED: 2016-08-26
*/
    ROS_INFO(">--------------------");

    // INITIALIZE VARIABLES
    trajectoryJoints joint_trajectory;
    std::vector<double> init_joint_values_left, init_joint_values_right, init_joint_values, joint_values;
    int total_joints_left, total_joints_right;

    init_joint_values = both_arms.getCurrentJointValues();
    init_joint_values_left  = std::vector<double>(&init_joint_values[0], &init_joint_values[total_joints_left]);
    init_joint_values_right = std::vector<double>(&init_joint_values[total_joints_left], &init_joint_values[joint_trajectory.total_joints]);

    joint_trajectory.group_name   = both_arms.getName();
    joint_trajectory.total_joints = init_joint_values.size();

    std::vector<std::string> joint_names = both_arms.getActiveJoints();
    for (int joint = 7; joint < joint_trajectory.total_joints; joint++) {
    /* Find the first joint in the right arm to calculate the total joints in the left and right arm */
        if (joint_names[joint].compare("yumi_joint_1_r") == 0) {
            total_joints_left  = joint;
            total_joints_right = joint_trajectory.total_joints - total_joints_left;
            break;
        }
    }

    ROS_INFO("Filling joint trajectory.");

    for (int trajectory = 0; trajectory < joint_trajectories.size(); trajectory++) {
        trajectoryJoints current_joint_trajectory = joint_trajectories[trajectory];
        if (current_joint_trajectory.group_name.compare("left_arm") == 0) {
        /* If the current joint trajectory only contains joint values for the left arm */
            std::vector<double> right_joint_values;

            if (trajectory = 0) {
            /* If the first trajectory is being stored, then use the current joint values for the right arm as the joint values in the trajectory */
                right_joint_values = init_joint_values_right;
            } else {
            /* If this is not the first trajectory being stored, set the right joint values as the last trajectory point joint values corresponding 
               to the right arm of the joint trajectory being constructed */
                right_joint_values = std::vector<double>(&joint_trajectory.joints.back()[total_joints_left], &joint_trajectory.joints.back()[joint_trajectory.total_joints]);
            }

            for (int point = 0; point < current_joint_trajectory.total_points; point++) {
                joint_values = current_joint_trajectory.joints[point];
                for (int joint = 0; joint < total_joints_right; joint++) {
                    joint_values.push_back(right_joint_values[joint]);
                }
                joint_trajectory.joints.push_back(joint_values);
            }
        } else if (current_joint_trajectory.group_name.compare("right_arm") == 0) {
        /* If the current joint trajectory only contains joint values for the right arm */
            std::vector<double> left_joint_values, right_joint_values;

            if (trajectory = 0) {
            /* If the first trajectory is being stored, then use the current joint values for the left arm as the joint values in the trajectory */
                left_joint_values = init_joint_values_left;
            } else {
            /* If this is not the first trajectory being stored, set the left joint values as the last trajectory point joint values corresponding 
               to the left arm of the joint trajectory being constructed */
                left_joint_values = std::vector<double>(&joint_trajectory.joints.back()[0], &joint_trajectory.joints.back()[total_joints_left]);
            }

            for (int point = 0; point < current_joint_trajectory.total_points; point++) {
                joint_values = left_joint_values;
                right_joint_values = current_joint_trajectory.joints[point];
                for (int joint = 0; joint < total_joints_right; joint++) {
                    joint_values.push_back(right_joint_values[joint]);
                }
                joint_trajectory.joints.push_back(joint_values);
            }
        } else {
        /* If the current joint trajectory contains joint values for both arms */
            for (int point = 0; point < current_joint_trajectory.total_points; point++) {
                joint_trajectory.joints.push_back(current_joint_trajectory.joints[point]);
            }
        }
    }

    joint_trajectory.total_points = joint_trajectory.joints.size();

    if (debug) {
        ROS_INFO("....................");
        ROS_INFO("(debug) Displaying filled joint trajectory.");
        for (int point = 0; point < joint_trajectory.total_points; point++) {
            ROS_INFO("....................");
            ROS_INFO("_____ (debug) Joint Trajectory Point %d Joint Values _____", point+1);
            for (int joint = 0; joint < joint_trajectory.total_joints; joint++) {
                ROS_INFO("%s joint value: %.5f", joint_names[joint].c_str(), joint_trajectory.joints[point][joint]);
            }
        }
    }

    ROS_INFO("Successfully filled joint trajectory.");
    return joint_trajectory;
}

bool generatePlans(planner& plans, planningInterface::MoveGroup& group, trajectoryJoints& joint_trajectory, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-22

    PURPOSE: The purpose of this function is to create a set of plans for given joint trajectory points.
             This is done by first setting the robots start state to the current state. Creating a plan
             for going for the start state to the first trajectory point, then set the first trajectory 
             point as the start state and plan for moving from there to the second trajectory point. 
             This happens for all trajectory points, and is then stored into a planner structure.

    INSTRUCTIONS: Input an empty planner structure, joint trajectory, and it associated move group. If
                  If debug mode is set to true, then if any plan fails the trajectory point that failed 
                  will be skipped, and the new target will be set to the next trajectory point. If debug 
                  mode is set to false, then if any plan fails the function will notify the user and exit 
                  returning false.
*/
    // CHECK INPUTS
    if (joint_trajectory.group_name.compare(group.getName()) != 0) {
        ROS_ERROR("From: generatePlans(group, joint_trajectory, debug)");
        ROS_ERROR("Provided joint trajectory is not meant for the provided group.");
        if (joint_trajectory.group_name.compare("") == 0) {
            ROS_INFO("....................");
            ROS_WARN("Group name for joint trajectory is empty.");
            ROS_WARN("This is likely due to a failure when converting to a joint trajectory.");
            ROS_INFO("....................");
        }
        ROS_WARN("Joint trajectory group: %s",joint_trajectory.group_name.c_str());
        ROS_WARN("Provided group: %s",group.getName().c_str());
        ROS_WARN("Plans were not able to be generated. Exiting function.");
        return false;
    }

    // INITIALIZE VARIABLES
    plans.group_name = joint_trajectory.group_name;
    int plan_index = 0;
    bool success = false;

    planningInterface::MoveGroup::Plan current_plan;
    planningInterface::MoveGroup::Plan last_successful_plan;

    moveit::core::RobotState state(group.getRobotModel());

    // NOTIFY USER FUNCTION IS ABOUT TO START
    if (debug) { ROS_INFO("(debug) Creating plans for provided joint trajectory."); }

    // CREATE PLANS FOR JOINT TRAJECTORY
    for (int plan = 0; plan < joint_trajectory.total_points; plan++) {
        if (plan_index == 0) {
            group.setStartStateToCurrentState();
        } else {
            if (!success) {
                current_plan = last_successful_plan;
            } else {
                last_successful_plan = current_plan;
            }
            moveit::core::jointTrajPointToRobotState(plans.plans[plan_index-1].trajectory_.joint_trajectory, (plans.plans[plan_index-1].trajectory_.joint_trajectory.points.size()-1), state);
            group.setStartState(state);
            /* Set the start state for the next plan to the last point in the previous successful plan */
        }

        group.setJointValueTarget(joint_trajectory.joints[plan]);
        planningInterface::MoveGroup::Plan currentPlan;
        success = group.plan(currentPlan);

        if (success) {
            plan_index++;
            plans.plans.push_back(currentPlan);

            if (debug) { ROS_INFO("(debug) Plan created for trajectory point: %d of %d", plan+1, joint_trajectory.total_points); }
        } else {
            if (debug) {
                ROS_WARN("(debug) Plan failed for trajectory point: %d of %d", plan+1, joint_trajectory.total_points);
                ROS_WARN("(debug) Skipping trajectory point and continuing planner.");
            } else {
                ROS_INFO("....................");
                ROS_ERROR("From: generatePlans(group, joint_trajectory, debug)");
                ROS_ERROR("Plan failed to be created for trajectory point %d.", plan+1);
                ROS_WARN("Exiting function.");
                return false;
            }
        }
    }

    plans.total_plans = plans.plans.size();
    if (debug) { ROS_INFO("(debug) Finished planning. Total plans created: %d of %d", plans.total_plans, joint_trajectory.total_points); }
    
    return true;
}


bool executePlans(planningInterface::MoveGroup& group, planner& plans, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-06-22

    PURPOSE: The purpose of this function is to execute all the plans in the provided planner structure.
             The robot will first move to the first state state indicated in the first plan in the planner
             structure. From there, each consecutive plan will be executed. If there is an error in 
             execution, the function will notify the user and the function will be exited. If debug mode
             is set to true, then the current plan will be skipped and the next plan will be executed.

    INSTRUCTIONS: Input a planner structure and its ssociated move group. If debug is set to true, then
                  if an execution of a plan fails, the plan will be skipped and the next one will be
                  executed. If debug is set to false, then the function will notify the user when an
                  execution of a plan fails, and the function will then be exited.
*/
    ROS_INFO(">--------------------");

    // INITIALIZE VARIABLES
    bool success;
    int successful_executions = 0;

    // CHECK INPUTS
    if (plans.group_name.compare(group.getName()) != 0) {
        ROS_ERROR("From: executePlans(group, plan, debug)");
        ROS_ERROR("Provided planner is not meant for the provided group.");
        ROS_WARN("Planner Group: %s",plans.group_name.c_str());
        ROS_WARN("Provided Group: %s",group.getName().c_str());
        ROS_WARN("Exiting function.");
        return false;
    }
        
    // NOTIFY USER FUNCTION IS ABOUT TO START
    ROS_INFO("Executing provided planner structure for group %s.", group.getName().c_str());

    // GO TO START STATE OF PLANNER STRUCTURE
    group.setStartStateToCurrentState();
    group.setJointValueTarget(plans.plans[0].start_state_.joint_state.position);
    success = group.move();
    if (!success) {
        ROS_INFO("....................");
        ROS_WARN("From: executePlans(group, plan, debug)");
        ROS_WARN("Not able to go to the start state of the trajectory.");

        if (debug) {
            ROS_WARN("(debug) Skipping start state of trajectory.");
        } else {
            ROS_ERROR("Error with executing plans. Exiting function.");
            return false;
        }
    }

    // EXECUTE PLANS
    for (int plan = 0; plan < plans.total_plans; plan++) {
        success = group.execute(plans.plans[plan]);

        if (success) {
            ROS_INFO("Executed trajectory: %d", plan+1);
            successful_executions++;
        } else {
            ROS_WARN("Failed to execute trajectory for plan: %d", plan+1);
            
            if (debug) {
                ROS_WARN("(debug) Skipping plan, executing next plan.");
            } else {
                ROS_INFO("....................");
                ROS_WARN("From: executePlans(group, plan, debug)");
                ROS_ERROR("Error with executing plans. Exiting function.");
                return false;
            }
        }
    }

    ROS_INFO("Finished executing. Total plans executed: %d of %d", successful_executions, plans.total_plans);

    return true;
}


