// INCLUDES
#include <fstream>
#include <cmath>
#include <ctime>
#include <math.h>
#include <sstream>
#include <stack>
#include <string>

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
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)
const std::string yumi_scripts_directory = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/";
const std::string yumi_rosbag_topic_name = "yumi";

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
void appendMasterJointTrajectory(std::vector<std::vector<double>>&, std::vector<yumi_scripts::JointMsg>&, bool debug = false);
bool checkModulesLineUp(RAPIDModuleData&, RAPIDModuleData&, bool debug = false);

// /* File Checking and ROS Bag Functions */
// bool fileExists(std::string, std::string, std::string);
// void savePlanner(std::string, planner&);

// /* RAPID Module Writing Functions */
// poseConfig getAxisConfigurations(std::vector<double>, bool debug = false);
// void writeToFile(std::string, leadThroughPoses&, bool debug = false);
// void writeToFile(std::string, leadThroughPoses&, leadThroughPoses&, bool debug = false);
// std::string getRobtargetOutputLine(std::string, poseConfig&, bool debug = false);

// /* Plan and Execution Functions */
// bool generatePlans(planner&, planningInterface::MoveGroup&, trajectoryJoints&, bool debug = false);
// bool executePlans(planningInterface::MoveGroup&, planner&, bool debug = false);

// MAIN FUNCTION
int main(int argc, char **argv) {

    ros::init(argc, argv, "moveit_interface");
    ros::NodeHandle node_handle("yumi");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // INITIALIZE VARIABLES
    bool debug = false;
    std::string pose_reference_frame = "yumi_body";
    std::string group_name;

    RAPIDModuleData module_master_left, module_master_right, module_left, module_right;
    std::vector<std::vector<double>> joint_trajectory_master;
    std::vector<yumi_scripts::JointMsg> joint_trajectory;
    poseConfig pose_config;
    jointConfig joint_config;
    std::string command;

    if (!(ros::param::get("/yumi/yumi_node/debug", debug))) {
        ROS_INFO(">--------------------");
        ROS_ERROR("Not able to get debug parameter from parameter server.");
        ROS_WARN("The configuration parameters must have not been loaded before the YuMi node was started.");
        return 0;
    }

    // INITIALIZE MOVE GROUPS FOR THE LEFT ARM, RIGHT ARM, AND BOTH ARMS
    planningInterface::MoveGroup left_arm("left_arm");
    left_arm.startStateMonitor();
    planningInterface::MoveGroup right_arm("right_arm");
    right_arm.startStateMonitor();
    planningInterface::MoveGroup both_arms("both_arms");
    both_arms.startStateMonitor();

    // SET MOVE GROUP REFRENCE FRAMES AND END EFFECTORS
    left_arm.setPoseReferenceFrame(pose_reference_frame);
    right_arm.setPoseReferenceFrame(pose_reference_frame);
    both_arms.setPoseReferenceFrame(pose_reference_frame);

    // SET UP SUBSCRIBERS
    ros::Subscriber module_sub       = node_handle.subscribe("modules", 1000, moduleCallback);
    ros::Subscriber joint_config_sub = node_handle.subscribe("joint_configs", 1000, jointConfigCallback);
    ros::Subscriber pose_config_sub  = node_handle.subscribe("pose_configs", 1000, poseConfigCallback);
    ros::Subscriber command_sub      = node_handle.subscribe("commands", 1000, commandCallback);

    // WAIT FOR COMMANDS
    int system_return = std::system("clear");
    ROS_INFO("Waiting for messages.");
    while (ros::ok()) {
    /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
        DATE CREATED: 2016-08-22
    */
        if (new_module) {
            new_module  = false;
            ROS_INFO(">--------------------");

            // COPY GLOBAL VARIABLES/STRUCTURES TO LOCAL
            group_name       = global_group_name;
            joint_trajectory = global_joint_trajectory;
            if (group_name.compare("both_arms") == 0) {
                module_left  = convertModuleMsgToModuleData(global_module_left, debug);
                module_right = convertModuleMsgToModuleData(global_module_right, debug);
            } else if (group_name.compare("left_arm") == 0) {
                module_left  = convertModuleMsgToModuleData(global_module_left, debug);
            } else if (group_name.compare("right_arm") == 0) {
                module_right = convertModuleMsgToModuleData(global_module_right, debug);
            } else {
                module_lock = false;
                ROS_ERROR("Group name in received module message not recognized.");
                ROS_WARN("Recognized group names: left_arm, right_arm, both_arms");
                ROS_WARN("Received group name: %s", group_name.c_str());
                ROS_WARN("Not storing module(s) and joint trajectory due to error.");
                continue;
            }

            // RELEASE DATA LOCK
            module_lock = false;
            ROS_INFO("Received module for group: %s.", group_name.c_str());

            // CHECK JOINT AND POSE TRAJECTORY SIZES ADD DATA TO MASTER STRUCTURES
            if (group_name.compare("both_arms") == 0) {
                if ((joint_trajectory.size() != module_left.pose_names.size()) || (joint_trajectory.size() != module_right.pose_names.size())) {
                    ROS_ERROR("The received joint trajectory and module data do not have the same size.");
                    ROS_WARN("Joint trajectory size: %lu", joint_trajectory.size());
                    ROS_WARN("Module sizes: %lu (left) %lu (right)", module_left.pose_names.size(), module_right.pose_names.size());
                    ROS_WARN("Not storing module(s) and joint trajectory due to error.");
                    continue;
                }

                bool success = checkModulesLineUp(module_left, module_right, debug);
                if (success) {
                    appendMasterModule(module_master_left, module_left, group_name, debug);
                    appendMasterModule(module_master_right, module_right, group_name, debug);
                } else {
                    ROS_INFO(">--------------------");
                    ROS_WARN("Not storing modules and joint trajectory due to error.");
                    continue;
                }
            } else if (group_name.compare("left_arm") == 0) {
                if (joint_trajectory.size() != module_left.pose_names.size()) {
                    ROS_INFO(">--------------------");
                    ROS_ERROR("The received joint trajectory and module data do not have the same size.");
                    ROS_WARN("Not storing module(s) and joint trajectory due to error.");
                    continue;
                }

                appendMasterModule(module_master_left, module_left, group_name, debug);
            } else if (group_name.compare("right_arm") == 0) {
                if (joint_trajectory.size() != module_right.pose_names.size()) {
                    ROS_INFO(">--------------------");
                    ROS_ERROR("The received joint trajectory and module data do not have the same size.");
                    ROS_WARN("Not storing module(s) and joint trajectory due to error.");
                    continue;
                }

                appendMasterModule(module_master_right, module_right, group_name, debug);
            }
            appendMasterJointTrajectory(joint_trajectory_master, joint_trajectory, debug);

            ROS_INFO("Successfully appended received module(s) to master module.");
        }

        if (new_joint_config) {
            new_joint_config  = false;
            ROS_INFO(">--------------------");

            joint_config = global_joint_config;
            joint_config_lock = false;
            ROS_INFO("Received joint configuration for group: %s.", global_group_name.c_str());
        }

        if (new_pose_config) {
            new_pose_config  = false;
            ROS_INFO(">--------------------");

            pose_config = global_pose_config;
            pose_config_lock = false;

            ROS_INFO("Received pose configuration for group: %s.", global_group_name.c_str());
        }

        if (new_command) {
            new_command  = false;
            ROS_INFO(">--------------------");

            command = global_command;
            command_lock = false;

            ROS_INFO("Received command: %s.", command.c_str());

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
        ROS_INFO(">--------------------");
        ROS_INFO("Converting received module message to module data strcuture.");
    }

    // CONVERT MODULE DATA MESSAGE TO RAPID MODULE DATA STRUCTURE
    module.group_name   = module_data_msg.group_name;
    module.total_points = module_data_msg.total_points;

    for (int pose = 0; pose < module.total_points; pose++) {
        pose_config.pose     = module_data_msg.pose_configs[pose].pose;
        pose_config.confdata = module_data_msg.pose_configs[pose].confdata;
        pose_config.external_axis_position = module_data_msg.pose_configs[pose].external_axis_position; 

        module.pose_configs.push_back(pose_config);
        module.pose_names.push_back(module_data_msg.pose_names[pose]);
    }

    if (debug) { ROS_INFO("Successfully converted received module message to module data strcuture."); }

    return module;
}

void appendMasterModule(RAPIDModuleData& master_module, RAPIDModuleData& module, std::string group_name, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (debug) { 
        ROS_INFO(">--------------------");
        ROS_INFO("Appending received module to master module.");
    }

    if (group_name.compare("both_arms") == 0) {
        for (int pose = 0; pose < module.total_points; pose++) {
            master_module.pose_names.push_back(module.pose_names[pose]);
            master_module.pose_configs.push_back(module.pose_configs[pose]);
        }
    } else {
        if (debug) { ROS_INFO("Assuming only non-syncronized movements since only one module was sent."); }

        std::string pose_name;
        for (int pose = 0; pose < module.total_points; pose++) {
            pose_name = "p" + std::to_string(pose+1);
            master_module.pose_names.push_back(pose_name);
            master_module.pose_configs.push_back(module.pose_configs[pose]);
        }
    }

    if (debug) { ROS_INFO("Successfully appended received module to master module."); }
}

void appendMasterJointTrajectory(std::vector<std::vector<double>>& master_joint_trajectory, std::vector<yumi_scripts::JointMsg>& joint_trajectory, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    if (debug) { 
        ROS_INFO(">--------------------");
        ROS_INFO("Appending received joint trajectory to master joint trajectory.");
    }

    for (int point = 0; point < joint_trajectory.size(); point++) {
        master_joint_trajectory.push_back(joint_trajectory[point].joint_values);
    }

    if (debug) { ROS_INFO("Successfully appended received joint trajectory to master joint trajectory."); }
}

bool checkModulesLineUp(RAPIDModuleData& module_1, RAPIDModuleData& module_2, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-22
*/
    // INITIALIZE VARIABLES
    int index_1(0), index_2(0), index(0);
    bool success = true;

    // CHECK IF MODULES ARE LINED UP
    if (debug) { 
        ROS_INFO(">--------------------");
        ROS_INFO("Checking if received modules are lined up."); 
    }

    while (true) {

        if ((index_1 == module_1.total_points) && (index_2 == module_2.total_points)) {
            break;
        } else if (index_1 == module_1.total_points) {
            index_1--;
        } else if (index_2 == module_2.total_points) {
            index_2--;
        } else if (module_1.pose_names[index_1].compare(module_2.pose_names[index_2]) != 0) {
            if ((module_1.pose_names[index_1].compare(0, 1, "p") == 0) && (module_2.pose_names[index_2].compare(0, 1, "p") != 0)) {
            /* If the first module is going to a non-syncronized point (designated by p#) and the second module is going to a syncronized point */
                if (module_2.pose_names[index_2].compare(module_2.pose_names[index_2-1]) != 0) {
                /* If previous point for the second module is not the same syncronized point as the current point */
                    success = false;
                }
            } else if ((module_1.pose_names[index_1].compare(0, 1, "p") != 0) && (module_2.pose_names[index_2].compare(0, 1, "p") == 0)) {
            /* If the second module is going to a non-syncronized point (designated by p#) and the first module is going to a syncronized point */
                if (module_1.pose_names[index_1].compare(module_1.pose_names[index_1-1]) != 0) {
                /* If previous point for the first module is not the same syncronized point as the current point */
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

        if (debug) { ROS_INFO("(debug) Line: %d | Module 1 point: %s | Module 2 point: %s", index, module_1.pose_names[index_1-1].c_str(), module_2.pose_names[index_2-1].c_str()); }

        if (!success) {
            ROS_ERROR("From: checkModulesLineUp(module_1, module_2, debug)");
            ROS_ERROR("Modules received are not lined up.");
            return success;
        }
    }

    if (debug) { ROS_INFO("Modules can be combined."); }
    return success;
}

// /* ------------------------------------------------------------ */
// /* ----------- FILE CHECKING AND ROS BAG FUNCTIONS ------------ */
// /* ------------------------------------------------------------ */

// bool fileExists(std::string file_name, std::string folder_name, std::string file_type) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-08-05

//     PURPOSE: Determine if the provided file name exists in the provided folder within the
//              yumi_scripts ROS package. The function returns true if the file exists and 
//              false if the file does not exist.

//     INSTRUCTIONS: Input the file name and the folder that contains the file. The inputted
//                   folder must be located within the yumi_scripts ROS package. The function
//                   returns true if the file exists and false if the file does not exist.
// */
//     // INITIALIZE VARIABLES
//     std::string file_path = yumi_scripts_directory + folder_name + "/" + file_name + file_type;
//     std::ifstream file(file_path);

//     // CHECK IF FILE EXISTS
//     if (file.good()) {
//         return true;
//     } else {
//         return false;
//     }
// }

// void savePlanner(std::string bag_name, planner& plans) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-08-05

//     PRUPOSE: The purpose of this function is to store the provided planner in a ROS bag with the
//              provided bag name. All bags will be stored within the "yumi_scripts" package in a folder
//              called "bags". A custom messages called PlannerMsg was created in order to store the 
//              appropriate data from the planner. The idea behind storing the planner structure is
//              that all data is first converted into a planner structure, and if the planner was
//              successfully create then this data can be stored and retrieved later for execution.
//              All information is stored within a topic called "yumi".

//     INSTRUCTIONS: Provide the bag name and the planner structure containing successfully created
//                   plans for the desired data. The function will create the bag with the topic name
//                   of "yumi".

//     NOTE: There is not error checking in this function since it is expected that all the relevent
//           information and requirements mentioned above are satisfied.
// */  
//     // INITIALIZE VARIABLES
//     std::string bag_path = yumi_scripts_directory + "bags/" + bag_name + ".bag";
//     yumi_scripts::PlannerMsg planner_msg;

//     // CREATE ROSBAG
//     rosbag::Bag bag(bag_path, rosbag::bagmode::Write);

//     // CONSTRUCT PLANNER MESSAGE
//     planner_msg.group_name  = plans.group_name;
//     planner_msg.start_state = plans.plans[0].start_state_;
//     planner_msg.total_plans = plans.total_plans;
//     for (int plan = 0; plan < plans.total_plans; plan++) {
//         planner_msg.trajectory.push_back(plans.plans[plan].trajectory_);
//     }

//     // WRITE TO BAG
//     bag.write(yumi_rosbag_topic_name, ros::Time::now(), planner_msg);
//     bag.close();

//     ROS_INFO("Planner successfully saved for group %s.", plans.group_name.c_str());
//     ROS_INFO("Bag location: %s", bag_path.c_str());
// }

// /* ------------------------------------------------------------ */
// /* -------------- RAPID MODULE WRITING FUNCTIONS -------------- */
// /* ------------------------------------------------------------ */

// poseConfig getAxisConfigurations(std::vector<double> joint_values, bool debug) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-07-26

//     PURPOSE: The purpose of this function is to get the axis configuration of the robot based on the
//              supplied joint values. The axis configurations are based on the ABB convention, which is 
//              shown under the variabe type "confdata" in the ABB RAPID reference manual. These 
//              configurations are also described below.

//     INSTRUCTIONS: The axis configraution is generated based on the provided joint values. If debug 
//                   mode is set to true, then the current joint values with be displayed along with the 
//                   confdata calculated from the current joint values.


//     -------------------- OTHER INFORMATION --------------------

//     ABB YuMi AXIS NAMING CONVENTION (from YuMi base to end effector): [1, 2, 7, 3, 4, 5, 6]

//     ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

//     CONFIGURATION SETS: -4 -> [-360, -270)   -3 -> [-270, -180)   -2 -> [-180, -90)   -1 -> [-90,   0)
//                          0 -> [   0,   90)    1 -> [  90,  180)    2 -> [ 180, 270)    3 -> [270, 360)
//         - The configuration values above are for joints 1, 4, and 6

//     CFX: ABCD
//         - A represents configuration for axis 5 and can be either (1) or (0)
//             > (0) if axis 5 position >= 0 degrees, (1) if axis 5 position < 0 degrees
//         - B represents configuration for axis 3 and can be either (1) or (0)
//             > (0) if axis 3 position >= -90 degrees, (1) if axis 3 position < -90 degrees
//         - C represents configuration for axis 2 and can be either (1) or (0)
//             > (0) if axis 2 position >= 0 degrees, (1) if axis 2 position < 0 degrees
//         - D represents the compatability bit, particulary used for linear movements
//             > This value is not used for the IK solver

//     Examples: (Compatibility bit assumed to be 0)
//         - Axis 5 = 0 degrees, Axis 3 = -90 degrees, Axis 2 = 0 degrees   | cfx = 0000 or 0
//         - Axis 5 = 0 degrees, Axis 3 = -91 degrees, Axis 2 = 0 degrees   | cfx = 0100 or 100
//         - Axis 5 = -90 degrees, Axis 3 = 0 degrees, Axis 2 = -90 degrees | cfx = 1010

//     EXTERNAL AXIS POSITION CONVENTION: [arm_angle, 9E+09, 9E+09, 9E+09, 9E+09, 9E+09]
// */
//     if (debug) { ROS_INFO(">--------------------"); }

//     // INITIALIZE VARIABLES
//     const double PI = 3.14159;
//     const double rad2deg = 180.0/PI;

//     poseConfig pose_config;
//     std::vector<double> joint_vaues;
//     int axis_1_config, axis_4_config, axis_6_config, cfx = 0;

//     if (debug) {
//         ROS_INFO("_____ (debug) Provided joint positions _____");
//         for (int joint = 0; joint < joint_values.size(); joint++) { ROS_INFO("%.5f", joint_values[joint]); } 
//     }

//     // GET AXIS CONFIGURATIONS
//     axis_1_config = std::floor(joint_values[0] / (PI/2.0)); // joint 1 configuration value
//     axis_4_config = std::floor(joint_values[4] / (PI/2.0)); // joint 4 configuration value
//     axis_6_config = std::floor(joint_values[6] / (PI/2.0)); // joint 6 configuration value
//     if (joint_values[5] < 0)    { cfx += 1000; } // joint 5 configuration value
//     if (joint_values[3] < PI/2) { cfx += 100;  } // joint 3 configuration value
//     if (joint_values[1] < 0)    { cfx += 10;   } // joint 2 configuration value

//     pose_config.confdata.push_back(axis_1_config);
//     pose_config.confdata.push_back(axis_4_config);
//     pose_config.confdata.push_back(axis_6_config);
//     pose_config.confdata.push_back(cfx);

//     pose_config.external_axis_position = joint_values[2]; // need to update to arm angle calculation

//     if (debug) {
//         ROS_INFO("_____ (debug) Axis configuration for provided joint positions _____");
//         ROS_INFO("Configuration: [%d, %d, %d, %d] | External Axis Position: [%0.5f]", 
//             pose_config.confdata[0], pose_config.confdata[1], pose_config.confdata[2], pose_config.confdata[3], 
//             pose_config.external_axis_position);
//     }

//     return pose_config;
// }

// void writeToFile(std::string output_file_name, leadThroughPoses& poses, bool debug) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-07-27

//     PURPOSE: The purpose of this function is to write the set of data points that were stored previously
//              to the desired file supplied by the user. The user supplied the file name, and this function
//              will place all files into the "modules" folder in the "yumi_scripts" ROS package. This function
//              will write a module with the ABB convention for a RAPID Module file for YuMi.

//     INSTRUCTIONS: The output_file_name variable should not have any path preceding it (home/folder/etc...)
//                   and show not contain any file extention (*.txt, *.mod, etc.) since this added in this
//                   function. The set of poses that were previously stored from the user also must be
//                   supplied to this function. If debug is set to true, the user will be notified of the
//                   current status of the function writing as well as each line that has been written to the
//                   file. 


//     -------------------- OTHER INFORMATION --------------------

//     SIMPLE ABB YuMi MODULE EXAMPLE:
//         1) MODULE module_name
//         2) LOCAL CONST string YuMi_App_Program_Version:="1.0.1";
//         3) LOCAL VAR robtarget s1 := [[316.65,45.52,21.47],[0.0502929,0.702983,-0.635561,0.315196],[-1,0,-1,1010],[-169.577,9E+09,9E+09,9E+09,9E+09,9E+09]];
//         4) PROC main()
//         5) MoveSync s1;
//         6) Move p1
//         7) OpenHand;
//         8) CloseHand;
//         9) ENDPROC
//        10) ENDMODULE

//     ABB YuMi MODULE CONVENTIONS:
//         - First line is always the module name
//         - Second line always defines that this file is to be used by the YuMi App Program
//         - Next set of lines define the robtargets in descending numerical order
//         - Line after the robtarget definitions is the start of the main function
//         - Next set of lines define the robot arm and gripper movements for the robot to move through
//             - MoveSync indicates a synced movement with the other arm
//             - Move indicates an independent movement from the other arm
//         - The last two lines indicate the end of the main function and end of the module

//     GRIPPER COMMANDS: The gripper can either be set to open or close. When set to close, the gripper will close its hand until
//                       it is either completely closed, or had grasped an object. The gripper close force can also be set from the 
//                       TouchPendant. For more instructions on how to properly use the gripper in this script when storing the 
//                       gripper position, please refer to the wiki page of this repo (shown below).

//     WIKI PAGE: https://github.com/ethz-asl/yumi/wiki
// */
//     // INITIALIZE VARIABLES
//     std::string robtarget_prefix = "LOCAL VAR robtarget ";
//     std::string output_file_path = yumi_scripts_directory + "modules/" + output_file_name + ".mod";
//     std::ofstream output_file;

//     ROS_INFO("Writing to file at: %s", output_file_path.c_str());

//     output_file.open(output_file_path.c_str(), std::ofstream::out | std::ofstream::app);

//     // ADD HEADER TO FILE
//     output_file << "MODULE " << output_file_name << std::endl;
//     output_file << "LOCAL CONST string YuMi_App_Program_Version:=\"1.0.1\"; !Do not edit or remove this line!" << std::endl;
    
//     // ADD ROBTARGETS
//     if (debug) { ROS_INFO("_____ (debug) Adding Robtargets to File _____"); }
//     for (int pose = poses.pose_names.size()-1; pose >= 0; pose--) {
//         if ((poses.pose_names[pose].compare(0, 1, "C") != 0) && (poses.pose_names[pose].compare(0, 1, "O") != 0)) {
//             output_file << robtarget_prefix << getRobtargetOutputLine(poses.pose_names[pose], poses.pose_configs[pose], debug) << std::endl;
//         }
//     }

//     output_file << "PROC main()" << std::endl;

//     // ADD MAIN FUNCTION
//     if (debug) { ROS_INFO("_____ (debug) Adding Main Function to File _____"); }
//     for (int line = 0; line < poses.pose_names.size(); line++) {
//         if (poses.pose_names[line].compare(0, 1, "p") == 0) {
//             output_file << "Move " << poses.pose_names[line] << ";" << std::endl;

//             if (debug) { ROS_INFO("(debug) Move %s;", poses.pose_names[line].c_str()); }
//         } else if ((poses.pose_names[line].compare(0, 1, "C") == 0) || (poses.pose_names[line].compare(0, 1, "O") == 0)) {
//             output_file << poses.pose_names[line] << std::endl;
//         } else {
//             output_file << "MoveSync " << poses.pose_names[line] << ";" << std::endl;
//             if (debug) { ROS_INFO("(debug) MoveSync %s;", poses.pose_names[line].c_str()); }
//         }
//     }

//     output_file << "ENDPROC" << std::endl << "ENDMODULE" << std::endl;

//     output_file.close();
// }

// void writeToFile(std::string output_file_name, leadThroughPoses& poses_left, leadThroughPoses& poses_right, bool debug) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-07-27

//     PURPOSE: The purpose of this function is create two RAPID modules using the data contained
//              in the two leadThroughPoses structures supplied. This function creates unique names
//              for each arm, and them supplied them to function which performs all file writing.

//     INSTRUCTIONS: The output_file_name variable should not have any path preceding it (home/folder/etc...)
//                   and show not contain any file extention (*.txt, *.mod, etc.) since this added in this
//                   function. The set of poses that were previously stored from the user for both arms must
//                   also be supplied to this function. If debug is set to true, the user will be notified of
//                   the current status of the function writing as well as each line that has been written to
//                   the file for each arm.
// */
//     // INITIALIZE VARIABLES
//     std::string output_file_name_left  = output_file_name + "_left";
//     std::string output_file_name_right = output_file_name + "_right";

//     // WRITE TO FILE LEFT ARM
//     if (debug) {
//         ROS_INFO("_____ (debug) Writing Left Arm File _____");
//     }
//     writeToFile(output_file_name_left, poses_left, debug);

//     // WRITE TO FILE RIGHT ARM
//     if (debug) {
//         ROS_INFO("_____ (debug) Writing Right Arm File _____");
//     }
//     writeToFile(output_file_name_right, poses_right, debug);
// }

// std::string getRobtargetOutputLine(std::string pose_name, poseConfig& pose_config, bool debug) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-07-27

//     PURPOSE: The purpose of this function is to construct robtarget data according to the ABB
//              convention and return the robtarget as a string. The appropriate unit conversions
//              are also applied according to the unit conventions shown below. The ABB robtarget
//              conventionis also shown below.

//     INSTRUCTIONS: The pose name and pose configuration data must be supplied. The returned
//                   value is a string containing the robtarget definition.


//     -------------------- OTHER INFORMATION --------------------

//     ABB UNITS: milimeters and degrees
//     ROS UNITS: meters and radians

//     ABB ROBTARGET CONVENTION: robtarget_name := [[position.x,position.y,position.z],[orientation.w,orientation.x,orientation.y,orientation.z],[cf1,cf4,cf6,cfx],[eax1,eax2,eax3,eax4,eax5,eax6]]
//         > cf1 represents the configuration data for axis 1
//         > cf4 represents the configuration data for axis 4
//         > cf6 represents the configuration data for axis 6
//         > cfx represents the configuration data for a combination of axis 5, 3, 2, and a compatibility bit
//         > eax1 represents the arm angle
//         > eax2 to eax6 represents 5 other external axis values, but these are not used on YuMi and thus are all set to 9E+09. This data is ignored.
// */
//     // INITIALIZE VARIABLES
//     const double PI      = 3.14159;
//     const double rad2deg = 180.0/PI;
//     const double m2mm    = 1000.0;

//     std::ostringstream output;
//     std::string empty_value = "9E+09";

//     // CONSTRUCT ROBTARGET
//     output << pose_name << " := [[";
//     output << pose_config.pose.position.x*m2mm << "," << pose_config.pose.position.y*m2mm << "," << pose_config.pose.position.z*m2mm << "],[";
//     output << pose_config.pose.orientation.w << "," << pose_config.pose.orientation.x << "," << pose_config.pose.orientation.y << "," << pose_config.pose.orientation.z << "],[";
//     output << pose_config.confdata[0] << "," << pose_config.confdata[1] << "," << pose_config.confdata[2] << "," << pose_config.confdata[3] << "],[";
//     output << pose_config.external_axis_position*rad2deg;
//     for (int i = 0; i < 5; i++) { output << "," << empty_value; }
//     output << "]];";

//     if (debug) { ROS_INFO("(debug) %s", output.str().c_str()); }

//     return output.str();
// }

// /* ------------------------------------------------------------ */
// /* ------------- PLANNING AND EXECUTION FUNCTIONS ------------- */
// /* ------------------------------------------------------------ */
// bool generatePlans(planner& plans, planningInterface::MoveGroup& group, trajectoryJoints& joint_trajectory, bool debug) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-06-22

//     PURPOSE: The purpose of this function is to create a set of plans for given joint trajectory points.
//              This is done by first setting the robots start state to the current state. Creating a plan
//              for going for the start state to the first trajectory point, then set the first trajectory 
//              point as the start state and plan for moving from there to the second trajectory point. 
//              This happens for all trajectory points, and is then stored into a planner structure.

//     INSTRUCTIONS: Input an empty planner structure, joint trajectory, and it associated move group. If
//                   If debug mode is set to true, then if any plan fails the trajectory point that failed 
//                   will be skipped, and the new target will be set to the next trajectory point. If debug 
//                   mode is set to false, then if any plan fails the function will notify the user and exit 
//                   returning false.
// */
//     ROS_INFO(">--------------------");

//     // CHECK INPUTS
//     if (joint_trajectory.group_name.compare(group.getName()) != 0) {
//         ROS_INFO("....................");
//         ROS_ERROR("From: generatePlans(group, joint_trajectory, debug)");
//         ROS_ERROR("Provided joint trajectory is not meant for the provided group.");
//         if (joint_trajectory.group_name.compare("") == 0) {
//             ROS_WARN("Group name for joint trajectory is empty.");
//             ROS_WARN("This is likely due to a failure when converting to a joint trajectory.");
//         }
//         ROS_WARN("Joint trajectory group: %s",joint_trajectory.group_name.c_str());
//         ROS_WARN("Provided group: %s",group.getName().c_str());
//         ROS_WARN("Plans were not able to be generated. Exiting function.");
//         return false;
//     }

//     // INITIALIZE VARIABLES
//     plans.group_name = joint_trajectory.group_name;
//     int plan_index = 0;
//     bool success = false;

//     planningInterface::MoveGroup::Plan current_plan;
//     planningInterface::MoveGroup::Plan last_successful_plan;

//     moveit::core::RobotState state(group.getRobotModel());

//     // NOTIFY USER FUNCTION IS ABOUT TO START
//     ROS_INFO("Creating plans for provided joint trajectory.");

//     // CREATE PLANS FOR JOINT TRAJECTORY
//     for (int plan = 0; plan < joint_trajectory.total_points; plan++) {
//         if (plan_index == 0) {
//             group.setStartStateToCurrentState();
//         } else {
//             if (!success) {
//                 current_plan = last_successful_plan;
//             } else {
//                 last_successful_plan = current_plan;
//             }
//             moveit::core::jointTrajPointToRobotState(plans.plans[plan_index-1].trajectory_.joint_trajectory, (plans.plans[plan_index-1].trajectory_.joint_trajectory.points.size()-1), state);
//             group.setStartState(state);
//             /* Set the start state for the next plan to the last point in the previous successful plan */
//         }

//         group.setJointValueTarget(joint_trajectory.joints[plan]);
//         planningInterface::MoveGroup::Plan currentPlan;
//         success = group.plan(currentPlan);

//         if (success) {
//             plan_index++;
//             plans.plans.push_back(currentPlan);

//             ROS_INFO("Plan created for trajectory point: %d of %d", plan+1, joint_trajectory.total_points);
//         } else {
//             ROS_WARN("Plan failed for trajectory point: %d of %d", plan+1, joint_trajectory.total_points);

//             if (debug) {
//                 ROS_WARN("(debug) Skipping trajectory point and continuing planner.");
//             } else {
//                 ROS_INFO("....................");
//                 ROS_ERROR("From: generatePlans(group, joint_trajectory, debug)");
//                 ROS_ERROR("Plan failed to be created for trajectory point %d.", plan+1);
//                 ROS_WARN("Exiting function.");
//                 return false;
//             }
//         }
//     }

//     plans.total_plans = plans.plans.size();
//     ROS_INFO("Finished planning. Total plans created: %d of %d", plans.total_plans, joint_trajectory.total_points);
    
//     return true;
// }

// bool executePlans(planningInterface::MoveGroup& group, planner& plans, bool debug) {
// /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
//     DATE CREATED: 2016-06-22

//     PURPOSE: The purpose of this function is to execute all the plans in the provided planner structure.
//              The robot will first move to the first state state indicated in the first plan in the planner
//              structure. From there, each consecutive plan will be executed. If there is an error in 
//              execution, the function will notify the user and the function will be exited. If debug mode
//              is set to true, then the current plan will be skipped and the next plan will be executed.

//     INSTRUCTIONS: Input a planner structure and its ssociated move group. If debug is set to true, then
//                   if an execution of a plan fails, the plan will be skipped and the next one will be
//                   executed. If debug is set to false, then the function will notify the user when an
//                   execution of a plan fails, and the function will then be exited.
// */
//     ROS_INFO(">--------------------");

//     // INITIALIZE VARIABLES
//     bool success;
//     int successful_executions = 0;

//     // CHECK INPUTS
//     if (plans.group_name.compare(group.getName()) != 0) {
//         ROS_ERROR("From: executePlans(group, plan, debug)");
//         ROS_ERROR("Provided planner is not meant for the provided group.");
//         ROS_WARN("Planner Group: %s",plans.group_name.c_str());
//         ROS_WARN("Provided Group: %s",group.getName().c_str());
//         ROS_WARN("Exiting function.");
//         return false;
//     }
        
//     // NOTIFY USER FUNCTION IS ABOUT TO START
//     ROS_INFO("Executing provided planner structure for group %s.", group.getName().c_str());

//     // GO TO START STATE OF PLANNER STRUCTURE
//     group.setStartStateToCurrentState();
//     group.setJointValueTarget(plans.plans[0].start_state_.joint_state.position);
//     success = group.move();
//     if (!success) {
//         ROS_INFO("....................");
//         ROS_WARN("From: executePlans(group, plan, debug)");
//         ROS_WARN("Not able to go to the start state of the trajectory.");

//         if (debug) {
//             ROS_WARN("(debug) Skipping start state of trajectory.");
//         } else {
//             ROS_ERROR("Error with executing plans. Exiting function.");
//             return false;
//         }
//     }

//     // EXECUTE PLANS
//     for (int plan = 0; plan < plans.total_plans; plan++) {
//         success = group.execute(plans.plans[plan]);

//         if (success) {
//             ROS_INFO("Executed trajectory: %d", plan+1);
//             successful_executions++;
//         } else {
//             ROS_WARN("Failed to execute trajectory for plan: %d", plan+1);
            
//             if (debug) {
//                 ROS_WARN("(debug) Skipping plan, executing next plan.");
//             } else {
//                 ROS_INFO("....................");
//                 ROS_WARN("From: executePlans(group, plan, debug)");
//                 ROS_ERROR("Error with executing plans. Exiting function.");
//                 return false;
//             }
//         }
//     }

//     ROS_INFO("Finished executing. Total plans executed: %d of %d", successful_executions, plans.total_plans);

//     return true;
// }


// INITIALIZE IK SERVICES FOR EACH ARM
// std::string end_effector_left    = "yumi_link_7_l";
// std::string end_effector_right   = "yumi_link_7_r";
// double search_discretization     = 0.001;
// boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader;
// kinematics::KinematicsBasePtr ik_left;
// kinematics::KinematicsBasePtr ik_right;

// kinematics_loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
// std::string plugin_name = "kdl_kinematics_plugin/KDLKinematicsPlugin";
// try {
//     ik_left  = kinematics_loader->createInstance(plugin_name);
//     ik_right = kinematics_loader->createInstance(plugin_name);
// } catch (pluginlib::PluginlibException& ex) {
//     ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
//     return 1;
// }

// bool success_ik_left, success_ik_right; 
// success_ik_left  = ik_left->initialize("/robot_description", "left_arm_ik", pose_reference_frame, end_effector_left, search_discretization);
// success_ik_right = ik_right->initialize("/robot_description", "right_arm_ik", pose_reference_frame, end_effector_right, search_discretization);

// if ((success_ik_left) && (success_ik_right)) {
//     ROS_INFO("Successfully initialized IK services for the left and right arm.");
// } else {
//     if ((!success_ik_left) && (!success_ik_right)) {
//         ROS_ERROR("Failed to initialize IK for left and right arm.");
//     } else if (!success_ik_left) {
//         ROS_ERROR("Failed to initialize IK for left arm.");
//     } else {
//         ROS_ERROR("Failed to initialize IK for right arm.");
//     }
//     ROS_ERROR("Program existing.");
//     return 1;
// }


