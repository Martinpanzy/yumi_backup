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

#include <yumi_scripts/PrePlan.h>

// NAMESPACE DECLARATION
using namespace ros;
namespace planningInterface = moveit::planning_interface;

// GLOBAL CONSTANTS
const double gripper_open_position = 0.024; // gripper open position (m)
const double gripper_closed_position = 0.0; // gripper closed position (m)
const std::string yumi_scripts_directory = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/"; // full path to folder where trajectory text files should be stored
const std::string yumi_rosbag_topic_name = "yumi";

// STRUCTURES
struct poseConfig {
    geometry_msgs::Pose pose;
    double gripper_position;
    std::vector<int> confdata;
    double external_axis_position;
};

struct trajectoryPoses {
    std::string group_name;
    std::string intended_group;
    std::vector<poseConfig> pose_configs_left;
    std::vector<poseConfig> pose_configs_right;
    bool gripper_attached_left  = false;
    bool gripper_attached_right = false; 
    int total_points;
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
/* File Checking and ROS Bag Functions */
bool fileExists(std::string, std::string, std::string);
void savePlanner(std::string, planner&);
planner retrievePlanner(std::string);

/* Configuration Functions */
poseConfig getCurrentPoseConfig(planningInterface::MoveGroup&, bool debug = false);
poseConfig getAxisConfigurations(planningInterface::MoveGroup&, bool debug = false);
poseConfig getAxisConfigurations(std::vector<double>, bool debug = false);

/* RAPID Module Retrieval/Conversion Functions */
RAPIDModuleData getYuMiLeadThroughData(std::string, planningInterface::MoveGroup&, bool debug = false);
poseConfig getRobtargetData(std::string, bool debug = false);

/* Conversion Functions */
trajectoryPoses combineModules(RAPIDModuleData&, RAPIDModuleData&, bool debug = false);
trajectoryPoses convertModuleToPoseTrajectory(RAPIDModuleData&, bool debug = false);
trajectoryJoints convertPoseTrajectoryToJointTrajectory(planningInterface::MoveGroup&, trajectoryPoses&, kinematics::KinematicsBasePtr&, kinematics::KinematicsBasePtr&, const bool, bool debug = false);
bool convertPoseConfigToJointTrajectory(trajectoryJoints&, kinematics::KinematicsBasePtr&, std::vector<poseConfig>&, const bool, bool debug = false);
bool parseCFX(int&, int&, int&, int&, bool debug = false);
bool isClose(poseConfig&, poseConfig&, bool debug = false);

/* Plan and Execution Functions */
bool generatePlans(planner&, planningInterface::MoveGroup&, trajectoryJoints&, bool debug = false);
bool executePlans(planningInterface::MoveGroup&, planner&, bool debug = false);

// MAIN FUNCTION
int main(int argc, char **argv) {
    ros::init (argc, argv, "test");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // INITIALIZE VARIABLES
    bool debug = false;
    std::string end_effector_left    = "yumi_link_7_l";
    std::string end_effector_right   = "yumi_link_7_r";
    std::string pose_reference_frame = "yumi_body";
    double search_discretization     = 0.001;

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

    // MOVE ARMS TO READY POSITION
    both_arms.setNamedTarget("ready");
    both_arms.move();

    // INITIALIZE IK SERVICES FOR EACH ARM
    boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader;
    kinematics::KinematicsBasePtr ik_left;
    kinematics::KinematicsBasePtr ik_right;

    kinematics_loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
    std::string plugin_name = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    try {
        ik_left  = kinematics_loader->createInstance(plugin_name);
        ik_right = kinematics_loader->createInstance(plugin_name);
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
        return 1;
    }

    bool success_ik_left, success_ik_right; 
    success_ik_left  = ik_left->initialize("/robot_description", "left_arm_ik", pose_reference_frame, end_effector_left, search_discretization);
    success_ik_right = ik_right->initialize("/robot_description", "right_arm_ik", pose_reference_frame, end_effector_right, search_discretization);

    if ((success_ik_left) && (success_ik_right)) {
        ROS_INFO("Successfully initialized IK services for the left and right arm.");
    } else {
        if ((!success_ik_left) && (!success_ik_right)) {
            ROS_ERROR("Failed to initialize IK for left and right arm.");
        } else if (!success_ik_left) {
            ROS_ERROR("Failed to initialize IK for left arm.");
        } else {
            ROS_ERROR("Failed to initialize IK for right arm.");
        }
        ROS_ERROR("Program existing.");
        return 1;
    }

    // WAIT FOR COMMANDS FROM USER
    const int MAX_ARGUMENTS = 4;

    int system_return;
    bool bounds(true), planner_successfully_created(false);
    std::string left_file(""), right_file(""), rosbag_file(""), module_folder("modules"), rosbag_folder("bags"), module_file_type(".mod"), rosbag_file_type(".bag");
    planner plans, empty_plans;

    system_return = std::system("clear"); // clear the screen
    ROS_INFO("Ready to take arguments. For a list of recognized arguments, enter \"help\"");

    while (ok()) {
    /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
        DATE CREATED: 2016-08-04

        DESCRIPTION: This while loop waits for a command line input from the user and determines if the inputted argument is on
                     of the regonized arguments. The recognized arguments include: exit, clear, help, debug, bounds, state, 
                     module, rosbag, end_effector, run. If the provided argument is not recognized, then the user will be 
                     notified and the script will wait for another input from the user. For more information on the recognized 
                     commands, please refer to the wiki page shown below.

        WIKI: github.com/ethz-asl/yumi/wiki/YuMi-MoveIt!-Interface
    */
        std::vector<std::string> inputs(MAX_ARGUMENTS,"");
        int argument = 0;

        std::string input;
        getline(std::cin, input);
        std::istringstream input_stream(input);

        while ((input_stream >> inputs[argument]) && (argument < MAX_ARGUMENTS)) {
        /* Get all arguments supplied by the user on the command line */
            argument++;
        }
        if (argument == MAX_ARGUMENTS) {
        /* If the user supplied more agurments than that max arguments allowed */
            ROS_ERROR("The max amount of arguments allowed is %d", MAX_ARGUMENTS);
            ROS_WARN("Please check inputted command and ensure it is in the list provided from typing \"help\".");
            continue;
        }

        if (inputs[0].compare("exit") == 0) {
        /* If the user would like to exit the program*/
            break;

        } else if (inputs[0].compare("clear") == 0) {
        /* If the user would like to clear the terminal window screen */
            system_return = std::system("clear");

        } else if (inputs[0].compare("help") == 0) {
        /* If the user would like a list of allowed arguments*/
            ROS_INFO("_____ List of Allowed Arguments _____");
            ROS_INFO("module <left/right> $module_name");
            ROS_INFO("module both $left_module_name $right_module_name");
            ROS_INFO("rosbag <run/save> $file_name");
            ROS_INFO("end_effector <left/right> $end_effector_name");
            ROS_INFO("end_effector both $left_end_effector_name $right_end_effector_name");
            ROS_INFO("run <left/right/both> (opt: dont_execute)");
            ROS_INFO(" ");
            ROS_INFO("debug <on/off>");
            ROS_INFO("bounds <on/off>");
            ROS_INFO("state");
            ROS_INFO("clear");
            ROS_INFO("exit");
            ROS_INFO("----------------------------------------");
            ROS_INFO("File names should NOT include file extensions (*.txt, *.mod, etc.)");
            ROS_INFO("For more information, please refer to the Wiki: github.com/ethz-asl/yumi/wiki/YuMi-Main-Script");

        } else if (inputs[0].compare("debug") == 0) {
        /* If the user would like to modify if the functions will run in debug mode or not */
            if (inputs[1].compare("on") == 0) {
            /* If the user would like to turn debug mode on */
                debug = true;
            } else if (inputs[1].compare("off") == 0) {
            /* If the user would like to turn debug mode off */
                debug = false;
            } else {
            /* If the agument is not recognized */
                ROS_ERROR("Argument not recognized.");
                ROS_WARN("Provided argument: %s", inputs[1].c_str());
                ROS_WARN("Expected argument(s): on, off");
            }

            ROS_INFO("Debug mode: %s", debug?"on":"off");

        } else if (inputs[0].compare("bounds") == 0) {
        /* If the user would like to remove or add bounded solutions for the IK solver */
            if (inputs[1].compare("on") == 0) {
            /* If the user would like to have the IK solutions bounded according to the axis configuration values in the module(s) */
                bounds = true;
            } else if (inputs[1].compare("off") == 0) {
            /* If the user would not like to have bounded IK solutions */
                bounds = false;
            } else {
            /* If the agument is not recognized */
                ROS_ERROR("Argument not recognized.");
                ROS_WARN("Provided argument: %s", inputs[1].c_str());
                ROS_WARN("Expected argument(s): on, off");
            }

            ROS_INFO("Bounded IK solutions: %s", bounds?"on":"off");

        } else if (inputs[0].compare("state") == 0) {
        /* If the user would like to know the state variables of the function (debug, bounds, etc.) */
            ROS_INFO("                   Debug mode: %s", debug?"on":"off");
            ROS_INFO("         Bounded IK solutions: %s", bounds?"on":"off");
            ROS_INFO("    Left arm mdolue file name: %s", left_file.c_str());
            ROS_INFO("   Right arm module file name: %s", right_file.c_str());
            ROS_INFO(" Left arm end effector for IK: %s", end_effector_left.c_str());
            ROS_INFO("Right arm end effector for IK: %s", end_effector_right.c_str());
            ROS_INFO("    Planner ready to be saved: %s", planner_successfully_created?"yes":"no");

        } else if (inputs[0].compare("module") == 0) {
        /* If the user would like to set the file name to be retrieved for one or both of the arms*/
            if (inputs[1].compare("left") == 0) {
            /* If the user is providing a file name of the module to be used for the left arm */
                if (fileExists(inputs[2], module_folder, module_file_type)) {
                /* If the provided file name for the module for the left arm exists */
                    left_file  = inputs[2];
                    ROS_INFO("Left arm module file name: %s", left_file.c_str());
                } else {    
                    ROS_ERROR("File could not be found, please check inputted file name.");
                }
            } else if (inputs[1].compare("right") == 0) {
            /* If the user is providing a file name of the module to be used for the right arm */
                if (fileExists(inputs[2], module_folder, module_file_type)) {
                /* If the provided file name for the module for the right arm exists */
                    right_file = inputs[2];
                    ROS_INFO("Right arm module file name: %s", right_file.c_str());
                } else {    
                    ROS_ERROR("File could not be found, please check inputted file name.");
                }
            } else if (inputs[1].compare("both") == 0) {
            /* If the user is providing a file name of the modules to be used for the left and right arm */
                if ((fileExists(inputs[2], module_folder, module_file_type)) && (fileExists(inputs[3], module_folder, module_file_type))) {
                /* If the provided file name for the module for both arms exists */
                    left_file  = inputs[2];
                    right_file = inputs[3];
                    ROS_INFO(" Left arm module file name: %s", left_file.c_str());
                    ROS_INFO("Right arm module file name: %s", right_file.c_str());
                } else {    
                    ROS_ERROR("File(s) could not be found, please check inputted file name.");
                }
            } else {
            /* If the agument is not recognized */
                ROS_ERROR("Argument not recognized.");
                ROS_WARN("Provided argument: %s", inputs[1].c_str());
                ROS_WARN("Expected argument(s): left, right, both");
            }

        } else if (inputs[0].compare("rosbag") == 0) {
        /* If the user would like to store or run a planner structure stored in a ROS bag */
            if (inputs[1].compare("run") == 0) {
            /* If the user would like to run plans that have been previously stored in a ROS bag*/
                if (fileExists(inputs[2], rosbag_folder, rosbag_file_type)) {
                /* If the ROS bag file exists */
                    planner_successfully_created = false;

                    rosbag_file = inputs[2];
                    planner plans = retrievePlanner(rosbag_file);

                    if (plans.group_name.compare("left_arm") == 0) { executePlans(left_arm, plans, debug); }
                    else if (plans.group_name.compare("right_arm") == 0) { executePlans(right_arm, plans, debug); }
                    else if (plans.group_name.compare("both_arms") == 0) { executePlans(both_arms, plans, debug); }    
                } else {    
                    ROS_ERROR("File could not be found, please check inputted file name.");
                }
            } else if (inputs[1].compare("save") == 0) {
            /* If the user would like to save plans that have been constructed from running the "run <left/right/both>" command */
                if (!fileExists(inputs[2], rosbag_folder, rosbag_file_type)) {
                /* If the ROS bag file does not already exist */
                    if (planner_successfully_created) {
                    /* If a planner structure has successfully been constructed previously by running the "run <left/right/both>" command */
                        planner_successfully_created = false;

                        rosbag_file = inputs[2];
                        savePlanner(rosbag_file, plans);
                    } else {
                        ROS_ERROR("Not able to save planner since it was not successfully created.");
                        ROS_WARN("Please rerun the modules and ensure the planner is successfully created.");
                    }
                } else {    
                    ROS_ERROR("File already exists. Please delete exiting file or provide a different file name.");
                }
            } else {
            /* If the agument is not recognized */
                ROS_ERROR("Argument not recognized.");
                ROS_WARN("Provided argument: %s", inputs[1].c_str());
                ROS_WARN("Expected argument(s): run, save");
            }

        } else if (inputs[0].compare("end_effector") == 0) {
        /* If the user would like to change the end effector link for the IK solver(s) */
            if (inputs[1].compare("left") == 0) {
            /* If the user would like to change the end effector for the IK solver only for the left arm */
                if (inputs[2].compare("") != 0) {
                    ik_left  = kinematics_loader->createInstance(plugin_name);
                    success_ik_left  = ik_left->initialize("/robot_description", "left_arm_ik", pose_reference_frame, inputs[2], search_discretization);
                    success_ik_right = true;
                }
            } else if (inputs[1].compare("right") == 0) {
            /* If the user would like to change the end effector for the IK solver only for the right arm */
                if (inputs[2].compare("") != 0) {
                    ik_right = kinematics_loader->createInstance(plugin_name);
                    success_ik_right = ik_right->initialize("/robot_description", "right_arm_ik", pose_reference_frame, inputs[2], search_discretization);
                    success_ik_left  = true;
                }
            } else if (inputs[1].compare("both") == 0) {
            /* If the user would like to change the end effector for the IK solvers for both arms */
                if ((inputs[2].compare("") != 0) && (inputs[3].compare("") != 0)) {
                    ik_left  = kinematics_loader->createInstance(plugin_name);
                    ik_right = kinematics_loader->createInstance(plugin_name);
                    success_ik_left  = ik_left->initialize("/robot_description", "left_arm_ik", pose_reference_frame, inputs[2], search_discretization);
                    success_ik_right = ik_right->initialize("/robot_description", "right_arm_ik", pose_reference_frame, inputs[3], search_discretization);
                }
            } else {
            /* If the agument is not recognized */
                ROS_ERROR("Argument not recognized.");
                ROS_WARN("Provided argument: %s", inputs[1].c_str());
                ROS_WARN("Expected argument(s): run, save");
                continue;
            }

            if ((success_ik_left) && (success_ik_right)) {
            /* If the IK solvers have successfully been initialized with the desired end effector */
                ROS_INFO("Successfully updated IK end effector for the %s arm(s).", inputs[1].c_str());

                if (inputs[1].compare("left") == 0) {
                    end_effector_left = inputs[2];
                } else if (inputs[1].compare("right") == 0) {
                    end_effector_right = inputs[2];
                } else {
                    end_effector_left  = inputs[2];
                    end_effector_right = inputs[3]; 
                }
            } else {
                if ((!success_ik_left) && (!success_ik_right)) {
                    ROS_ERROR("Failed to initialize IK for left and right arm.");
                } else if (!success_ik_left) {
                    ROS_ERROR("Failed to initialize IK for left arm.");
                } else {
                    ROS_ERROR("Failed to initialize IK for right arm.");
                }
            }

        } else if (inputs[0].compare("run") == 0) {
        /* If the user would like to run through the planning pipeline using the previously stored module names */
            planningInterface::MoveGroup *move_group;
            bool execute_plans = true;
            if (inputs[1].compare("left") == 0) {
            /* If the user would like to retrieve and execute a module only for the left arm */
                if (left_file.compare("") != 0) {
                /* If the user has provided file names for the left arm module */
                    move_group = &left_arm;
                } else {
                    ROS_ERROR("File name for the left arm was not set.");
                    ROS_WARN("Please set the file name for the left arm and before rerunning command.");
                    continue;
                }
            } else if (inputs[1].compare("right") == 0) {
            /* If the user would like to retrieve and execute a module only for the right arm */
                if (right_file.compare("") != 0) {
                /* If the user has provided file names for the right arm module */
                    move_group = &right_arm;
                } else {
                    ROS_ERROR("File name for the left arm was not set.");
                    ROS_WARN("Please set the file name for the left arm and before rerunning command.");
                    continue;
                }
            } else if (inputs[1].compare("both") == 0) {
            /* If the user would like to retrieve and execute a module for both arms */
                if ((left_file.compare("") != 0) && (right_file.compare("") != 0)) {
                /* If the user has provided file names for the left and right arm modules */
                    move_group = &both_arms;
                } else {
                    if ((left_file.compare("") == 0) && (right_file.compare("") == 0)) {
                        ROS_ERROR("File name for the left and right arm was not set.");
                        ROS_WARN("Please set the file name for the left and right arm and before rerunning command.");
                    } else if (left_file.compare("") == 0) {
                        ROS_ERROR("File name for the left arm was not set.");
                        ROS_WARN("Please set the file name for the left arm and before rerunning command.");
                    } else {
                        ROS_ERROR("File name for the right arm was not set.");
                        ROS_WARN("Please set the file name for the right arm and before rerunning command.");
                    }
                    continue;
                }
            } else {
            /* If the agument is not recognized */
                ROS_ERROR("Argument not recognized.");
                ROS_WARN("Provided argument: %s", inputs[1].c_str());
                ROS_WARN("Expected argument(s): run, save");
                continue;
            }

            if (inputs[2].compare("dont_execute") == 0) {
            /* If the user would not like to execute any constructed plans */
                execute_plans = false;
            } else if (inputs[2].compare("") != 0) {
                ROS_ERROR("Optional argument not recognized. Ignoring argument.");
            }

            plans = empty_plans;
            planner_successfully_created = false;

            RAPIDModuleData module_left, module_right;
            if (move_group->getName().compare("left_arm") == 0) {
            /* If the user would only like to get a module for the left arm */
                module_left  = getYuMiLeadThroughData(left_file, left_arm, debug);
                module_left.group_name = "null";
            } else if (move_group->getName().compare("right_arm") == 0) {
            /* If the user would only like to get a module for the right arm */
                module_right = getYuMiLeadThroughData(right_file, right_arm, debug);
                module_right.group_name = "null";
            } else {
            /* If the user would only like to get a module for both arms */
                module_left  = getYuMiLeadThroughData(left_file, left_arm, debug);
                module_right = getYuMiLeadThroughData(right_file, right_arm, debug);
            }
            
            if ((module_left.group_name.compare("") != 0) && (module_right.group_name.compare("") != 0)) {
            /* If the data from the module(s) was able to be retrieved properly without errors, then convert the module data into a pose trajectory structure */
                trajectoryPoses pose_trajectory;
                if (move_group->getName().compare("left_arm") == 0) {
                    pose_trajectory = convertModuleToPoseTrajectory(module_left, debug);
                } else if (move_group->getName().compare("right_arm") == 0) {
                    pose_trajectory = convertModuleToPoseTrajectory(module_right, debug);
                } else {
                    pose_trajectory = combineModules(module_left, module_right, debug);
                }

                if (pose_trajectory.group_name.compare("") != 0) {
                /* If the module(s) were able to be converted into a pose trajectory structure, then convert the pose trajectory to a joint trajectory */
                    trajectoryJoints joint_trajectory = convertPoseTrajectoryToJointTrajectory(*move_group, pose_trajectory, ik_left, ik_right, bounds, debug);

                    if (joint_trajectory.group_name.compare("") != 0) {
                    /* If the pose trajectory was able to converted into a joint trajectory structure, then generate plans for the trajectories */
                        planner_successfully_created = generatePlans(plans, *move_group, joint_trajectory, debug);

                        if ((planner_successfully_created) && (execute_plans)) { 
                        /* If a set of plans for the pose trajectory was created successsfully, then execute plans */
                            executePlans(both_arms, plans, debug); 
                        } else if (!planner_successfully_created) {
                            ROS_INFO("....................");
                            ROS_ERROR("Error creating plans for the joint trajectory.");
                            ROS_WARN("Most likely due to a robot collision during planning. Check reported errors if any.");
                        } else if (!execute_plans) {
                            ROS_INFO("....................");
                            ROS_INFO("Not executing plans as desired by user.");
                        }
                    } else {
                        ROS_INFO("....................");
                        ROS_ERROR("Unsuccessfuly conversion from poses to joints trajectories.");
                        ROS_WARN("Most likely due a failed inverse kinematics calculation. Check reported errors if any.");
                    }
                } else {
                    ROS_INFO("....................");
                    ROS_ERROR("Unsuccessfuly conversion from module data structure to pose trajectory structure.");
                    ROS_WARN("Most likely due to not being able to combine two RAPID modules. Check reported errors if any.");
                }
            } else {
                ROS_INFO("....................");
                ROS_ERROR("Error retirving module data.");
                ROS_WARN("The provided module(s) were not able to be retrived. Most likely due to being misformatted.");
                ROS_WARN("Please ensure the provided module(s) have the proper formatting. Check reported errors if any.");
            }

        } else {
            ROS_ERROR("First argument not recognized.");
        }
    }

    return 0;
}


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
    std::string file_path = yumi_scripts_directory + folder_name + "/" + file_name + file_type;
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
             called "bags". A custom messages called PrePlan was created in order to store the 
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
    std::string bag_path = yumi_scripts_directory + "bags/" + bag_name + ".bag";
    yumi_scripts::PrePlan pre_plan_msg;

    // CREATE ROSBAG
    rosbag::Bag bag(bag_path, rosbag::bagmode::Write);

    // CONSTRUCT PLANNER MESSAGE
    pre_plan_msg.group_name         = plans.group_name;
    pre_plan_msg.start_state        = plans.plans[0].start_state_;
    pre_plan_msg.total_trajectories = plans.total_plans;
    for (int plan = 0; plan < plans.total_plans; plan++) {
        pre_plan_msg.trajectory.push_back(plans.plans[plan].trajectory_);
    }

    // WRITE TO BAG
    bag.write(yumi_rosbag_topic_name, ros::Time::now(), pre_plan_msg);
    bag.close();

    ROS_INFO("Planner successfully saved for group %s.", plans.group_name.c_str());
    ROS_INFO("Bag location: %s", bag_path.c_str());
}

planner retrievePlanner(std::string bag_name) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-05

    PRUPOSE: The purpose of this function is to retrieve a stored planner structure with the provided
             name. The function searches for ROS bags in the "bags" folder within the yumi_scripts
             package. All data should have been stored within these bags with the topic name of "yumi"
             and stored using the custom message called PrePlan. This function will retrive the PrePlan
             message and reconstruct it into a planner structure. 

    INSTRUCTIONS: Provide the bag name containing the desired planner data. This function will then
                  search for the file within the "bags" folder in the "yumi_scripts", pull the PrePlan
                  message within the file from the topic called "yumi", then reconstruct and return
                  the planner structure. The planner structured needs to have been stored using the
                  PrePlan message in a topic called "yumi" in order for the data retrival to work
                  correctly.

    NOTE: There is not error checking in this function since it is expected that all the relevent
          information and requirements mentioned above are satisfied.
*/  
    // INITIALIZE VARIABLES
    std::string bag_path = yumi_scripts_directory + "bags/" + bag_name + ".bag";
    yumi_scripts::PrePlan::ConstPtr pre_plan_msg;

    std::vector<moveit_msgs::RobotTrajectory> trajectories;
    planningInterface::MoveGroup::Plan current_plan;
    planner plans;

    // GET BAG
    rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(yumi_rosbag_topic_name));

    // GET BAG ELEMENTS
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        pre_plan_msg = m.instantiate<yumi_scripts::PrePlan>();
    }

    // TRANSFER MESSAGE TO PLANNER STRUCTURE
    plans.group_name  = pre_plan_msg->group_name;
    plans.total_plans = pre_plan_msg->total_trajectories;
    trajectories      = pre_plan_msg->trajectory;

    for (int plan = 0; plan < plans.total_plans; plan++) {
        current_plan.trajectory_ = trajectories[plan];
        plans.plans.push_back(current_plan);
    }
    plans.plans[0].start_state_ = pre_plan_msg->start_state;

    // CLOSE BAG
    bag.close();

    ROS_INFO("Planner successfully retrieved for group %s.", plans.group_name.c_str());

    return plans;
}

/* ------------------------------------------------------------ */
/* --------------- AXIS CONFIGURATION FUNCTIONS --------------- */
/* ------------------------------------------------------------ */

poseConfig getCurrentPoseConfig(planningInterface::MoveGroup& group, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-26

    PURPOSE: The purpose of this function is to get the current pose and configuration of the robot.
             This function calles the function getAxisConfigurations to get the confdata and
             external axis position from the robots current position, and then the current pose and
             gripper position (if a gripper is attached to the provided group) is added to the pose
             configuration structure.

    INSTRUCTIONS: The only information that need to be provided to this function is the move group
                  which the user would like to get the current axis configuration for. The pose and
                  configuration data structure is then retruned after retrieving the data. The degug
                  variable is not using in this function, but is passed into the 
                  getAxisConfigurations function in case the user would like the debug messages.
*/
    // INTIIALIZE VARIABLES
    poseConfig pose_config;
    geometry_msgs::PoseStamped pose_stamped;

    // GET AXIS CONFIGURATION
    pose_config = getAxisConfigurations(group, debug);
    
    // GET CURRENT POSE
    pose_stamped = group.getCurrentPose();
    pose_config.pose = pose_stamped.pose;
    if (group.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
        pose_config.gripper_position = group.getCurrentJointValues().back();
    }

    return pose_config;
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
            > This value is not used for the IK solver

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
        for (int joint = 0; joint < joint_values.size(); joint++) { 
            ROS_INFO("Joint value: %.5f", joint_values[joint]); 
        } 
    }

    // GET AXIS CONFIGURATIONS
    axis_1_config = floor((joint_values[0] * rad2deg) / 90.0); 
    axis_4_config = floor((joint_values[4] * rad2deg) / 90.0);
    axis_6_config = floor((joint_values[6] * rad2deg) / 90.0);
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
    axis_1_config = std::floor(joint_values[0] / (PI/2.0)); 
    axis_4_config = std::floor(joint_values[4] / (PI/2.0));
    axis_6_config = std::floor(joint_values[6] / (PI/2.0));
    if (joint_values[5] < 0)    { cfx += 1000; }
    if (joint_values[3] < PI/2) { cfx += 100;  }
    if (joint_values[1] < 0)    { cfx += 10;   }

    pose_config.confdata.push_back(axis_1_config);
    pose_config.confdata.push_back(axis_4_config);
    pose_config.confdata.push_back(axis_6_config);
    pose_config.confdata.push_back(cfx);

    pose_config.external_axis_position = joint_values[2];

    if (debug) {
        ROS_INFO("_____ (debug) Axis configuration for provided joint positions _____");
        ROS_INFO("Configuration: [%d, %d, %d, %d] | External Axis Position: [%0.5f]", 
            pose_config.confdata[0], pose_config.confdata[1], pose_config.confdata[2], pose_config.confdata[3], 
            pose_config.external_axis_position);
    }

    return pose_config;
}

/* ------------------------------------------------------------ */
/* ------------ YUMI MODULE MANIPULATION FUNCTIONS ------------ */
/* ------------------------------------------------------------ */

RAPIDModuleData getYuMiLeadThroughData(std::string file_name, planningInterface::MoveGroup& group, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-07

    PURPOSE: The purpose of this funciton is to open the provided RAPID module, store all the robtargets at the beginning of the 
             file, get the order to executing the robtargets from the main function, then reconstructing the final trajectory. A
             simple RAPID module and program flow explaination is below along with a good convention to follow when creating these
             modules.

    INSTRUCTIONS: The RAPID module file name should be provided without extentions (.txt, etc.) or file path (/home/user). This 
                  function will search for that file within the yummi_scripts package in a folder called "demo". Make sure the 
                  RAPID module is located within that folder. The move group for this file should also be provided in case that
                  the first command within the RAPID module is OpenHand or CloseHand. In this case, the current pose of the robot
                  for the provided group will be used as the first pose for the trajectory. If debug is set to true, the user will
                  be notified about the current status of the function, the robtargets parsed from the file, the order of 
                  robtargets from the main function, and the final reconstructed trajectory.

    NOTE: If the data retrival is not successful, the module will be returned with an empty group name indicating that the retieval
          was not successful.


    -------------------- OTHER INFORMATION --------------------

    SIMPLE ABB MODULE EXAMPLE:
        1) MODULE module_name
        2) LOCAL CONST string YuMi_App_Program_Version:="1.0.1";
        3) LOCAL VAR robtarget s1 := [[316.65,45.52,21.47],[0.0502929,0.702983,-0.635561,0.315196],[-1,0,-1,1010],[-169.577,9E+09,9E+09,9E+09,9E+09,9E+09]];
        4) PROC main()
        5) MoveSync s1;
        6) OpenHand;
        7) CloseHand;
        8) ENDPROC
        9) ENDMODULE

    PROGRAM FLOW:
        > The module name is retrieved and stored
        > Second line is skipped
        > All robtargets are stored until reaching line 4 above
        > All points and gripper movements are stored in the same order as in the RAPID module
        > When reaching line 8 above, data retrieval is finished
        > The trajectory is reconstructed into a trajectory by matching the first trajectory point name in the main function with the stored robtarget with the same name
        > The reconstructed trajectory is returned

    GOOD CONVENTION: The first trajectory point is not an OpenHand or CloseHand command but rather a pose. If the first command 
                     is one of these commands, the first pose is assumed to be the current position and configuration of the robot.

    ASSUMPTIONS: Assuming grippers are being used on each arms when creating the RAPID modules.

    NOTE: If the functions runs into an OpenHand or CloseHand command, the pose is assumed to the same as the last pose retrieved
          from the main function. The case where an OpenHand or CloseHand command is the first command in the main function is 
          described above in the "GOOD CONVENTION" section.
*/
    ROS_INFO(">--------------------");

    // INITIALIZE VARIABLES
    std::string line;
    std::string empty;
    std::string data_type;
    std::string robtarget;

    double gripper_position = 0;
    std::vector<double> gripper_positions;
    bool gripper_warning_stated = false;
    /* If gripper is not attached to the provided group, but the file includes gripper movements, then the user will be 
       notified of this. This boolean indicates if this message has already been displayed in order to refrain from 
       displaying this issue multiple times. */

    poseConfig robtarget_pose_config;
    std::vector<poseConfig> robtarget_pose_configs;
    RAPIDModuleData module;
    module.group_name = group.getName();
    
    std::string function_point_name;
    std::string robtarget_point_name;
    std::vector<std::string> robtarget_point_names;

    int line_index = 0;
    int main_function_index = 0;

    bool robtarget_end = 0;
    bool success = true;

    std::string input_file = yumi_scripts_directory + "modules/" + file_name + ".mod";

    // CHECK IF GROUP ATTACHED TO PROVIDED GROUP
    if (group.getActiveJoints().back().compare(0, 7, "gripper") == 0) {
        module.gripper_attached = true;
        gripper_position = group.getCurrentJointValues().back();
    }

    // NOTIFY USER FUNCTION IS ABOUT TO START
    ROS_INFO("Getting YuMi file data.");

    // OPEN RAPID MODULE
    std::ifstream text_file(input_file.c_str());
    tic();

    if (text_file.is_open()) {
        
        if (std::getline(text_file, line)) {

            line_index++;
            if (debug) { ROS_INFO("(debug) Pulling line: %d",line_index); }

            // GET MODULE NAME
            std::istringstream first_line(line);
            first_line >> empty >> module.module_name;
            ROS_INFO("Module name: %s",module.module_name.c_str());

            // RUN THROUGH THE REMAINING LINES OF THE RAPID MODULE
            while (std::getline(text_file, line)) {
                
                line_index++;
                if (debug) { ROS_INFO("(debug) Pulling line: %d",line_index); }
                
                // GET FIRST WORD FROM LINE
                std::istringstream current_line(line);
                current_line >> data_type;

                // DETERMINE WHETHER TO STORE A ROBTARGET, STORE TRAJECTORY POINT, OR IF REACHED END OF FILE
                if (data_type.compare(0, 1, "P") == 0) {
                /* If the current line reads "PROC main()", then set a flag to indicate that all robtargets have been stored and 
                   to begin reading the main function */
                    robtarget_end = 1;

                    if (debug) { ROS_INFO("(debug) Reading from Main function."); }

                } else if (!(robtarget_end)) {
                /* If there are still robtargets to store */
                    current_line >> empty >> data_type;
                    if (data_type.compare(0, 1, "r") == 0) {
                        current_line >> robtarget_point_name >> empty >> robtarget;
                        robtarget_point_names.push_back(robtarget_point_name);

                        robtarget_pose_config = getRobtargetData(robtarget, debug);
                        robtarget_pose_configs.push_back(robtarget_pose_config);

                        if (debug) { ROS_INFO("(debug) Pulled robtarget for position: %s. Stored data.", robtarget_point_name.c_str()); }
                    }

                } else if (data_type.compare(0, 1, "E") == 0) {
                /* If the current line reads "ENDPROC", then the end of file has been reached and exit the while loop */
                    ROS_INFO("End of file reached.");
                    break;

                } else {
                /* If all robtargets have been stored and the end of the file has not been reached yet */
                    main_function_index++;
                    if (data_type.compare("MoveSync") == 0) {
                        current_line >> function_point_name;
                        function_point_name = function_point_name.substr(0, function_point_name.length()-1);
                    } else if (data_type.compare("OpenHand;") == 0) {
                    /* If the hand is to be opened at this point, use the previous pose (function_point_name wasn't overwritten)
                       and update the gripper position to indicate the gripper should be open. */
                        if (main_function_index == 1) {
                        /* If the first instruction is to open the gripper, there is no reference to where the pose of the robot
                           should be before closing the gripper. In this case, pose used is assumed to be the current pose of
                           the robot. To do this, first the current pose configuration of the robot is retrieved and then added 
                           to the stored robtargets list with the name "pStart". The function_point_name is then also assigned 
                           with the name "pStart", which will then be used later in order to find the current pose configuration 
                           of the robot when reconstructing the final pose trajectory. */
                            poseConfig pose_config = getCurrentPoseConfig(group, debug);
                            robtarget_pose_configs.push_back(pose_config);

                            robtarget_point_names.push_back("pStart");
                            function_point_name = "pStart";
                        }
                        if (module.gripper_attached) {
                            gripper_position = gripper_open_position;
                        } else if (!gripper_warning_stated) {
                            ROS_WARN("Module contains gripper command but a gripper is not attached to the provided group.");
                            ROS_WARN("Module name: %s", module.module_name.c_str());
                            ROS_WARN("Provided group: %s", group.getName().c_str());
                            gripper_warning_stated = true;
                        }
                    } else if (data_type.compare("CloseHand;") == 0) {
                    /* If the hand is to be closed at this point, use the previous pose (function_point_name wasn't overwritten)
                       and update the gripper position to indicate the gripper should be closed. */
                        if (main_function_index == 1) {
                        /* If the first instruction is to close the gripper, there is no reference to where the pose of the robot
                           should be before closing the gripper. In this case, pose used is assumed to be the current pose of
                           the robot. To do this, first the current pose configuration of the robot is retrieved and then added 
                           to the stored robtargets list with the name "pStart". The function_point_name is then also assigned 
                           with the name "pStart", which will then be used later in order to find the current pose configuration 
                           of the robot when reconstructing the final pose trajectory. */
                            poseConfig pose_config = getCurrentPoseConfig(group, debug);
                            robtarget_pose_configs.push_back(pose_config);

                            robtarget_point_names.push_back("pStart");
                            function_point_name = "pStart";
                        }
                        if (module.gripper_attached) {
                            gripper_position = gripper_closed_position;
                        } else if (!gripper_warning_stated) {
                            ROS_WARN("Module contains gripper command but a gripper is not attached to the provided group.");
                            ROS_WARN("Module name: %s", module.module_name.c_str());
                            ROS_WARN("Provided group: %s", group.getName().c_str());
                            gripper_warning_stated = true;
                        }
                    }

                    module.pose_names.push_back(function_point_name);
                    gripper_positions.push_back(gripper_position);

                    if (debug) { ROS_INFO("(debug) Main function line: %d | Pose name: %s | Gripper position: %.5f", main_function_index, function_point_name.c_str(), gripper_position); }
                }
            }
        } else {
            ROS_ERROR("From: getYuMiLeadThroughData(file_name, group, debug)");
            ROS_ERROR("The provided file is empty.");
            success = false;
        }
        text_file.close();
    } else {
        ROS_ERROR("From: getYuMiLeadThroughData(file_name, group, debug)");
        ROS_ERROR("The provided file name could not be opened or does not exist.");
        ROS_WARN("File: %s",input_file.c_str());
        success = false;
    }

    if (success) {
        if (debug) { ROS_INFO("...................."); }

        module.total_points = module.pose_names.size();
        ROS_INFO("Storing trajectory. Total trajectory points to store: %lu", module.pose_names.size());

        // CONSTRUCTION THE FINAL POSE TRAJECTORY
        int stored_location;
        int total_stored_points = robtarget_point_names.size();
        for (int point = 0; point < module.total_points; point++) {

            std::string point_name = module.pose_names[point];
            for (int robtarget_index = 0; robtarget_index < total_stored_points; robtarget_index++) {
                if (robtarget_point_names[robtarget_index].compare(point_name) == 0) {
                    stored_location = robtarget_index;
                    break; 
                }
            }

            module.pose_configs.push_back(robtarget_pose_configs[stored_location]);
            module.pose_configs[point].gripper_position = gripper_positions[point];

            if (debug) { ROS_INFO("(debug) Trajectory point %d was stored successfully", point+1); }
        }
    } else {
        ROS_ERROR("Not able to parse YuMi lead through file.");
        module.group_name = "";
    }

    if (debug) {
        ROS_INFO("....................");
        ROS_INFO("(debug) Module Name: %s", module.module_name.c_str());

        std::string current_point_name;
        std::string previous_point_name = "";

        for (int point = 0; point < module.total_points; point++) {
            current_point_name = module.pose_names[point].c_str();
            if (previous_point_name.compare(current_point_name) == 0) {
                if (module.pose_configs[point].gripper_position == gripper_open_position) {
                    ROS_INFO("(debug) Open Hand");
                } else {
                    ROS_INFO("(debug) Close Hand");
                }
            } else {
                ROS_INFO("(debug) Name: %s | Position: %.5f, %.5f, %.5f | Orientation: %.5f, %.5f, %.5f, %.5f | Configuration: %d, %d, %d, %d | External Axis: %.5f", 
                    module.pose_names[point].c_str(), 
                    module.pose_configs[point].pose.position.x, module.pose_configs[point].pose.position.y, module.pose_configs[point].pose.position.z, 
                    module.pose_configs[point].pose.orientation.w, module.pose_configs[point].pose.orientation.x, module.pose_configs[point].pose.orientation.y, module.pose_configs[point].pose.orientation.z, 
                    module.pose_configs[point].confdata[0], module.pose_configs[point].confdata[1], module.pose_configs[point].confdata[2], module.pose_configs[point].confdata[3], 
                    module.pose_configs[point].external_axis_position);
            }
            previous_point_name = current_point_name;
        }
    }

    ROS_INFO("Processing time: %.5f",toc());
    return module;
}

poseConfig getRobtargetData(std::string robtarget, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-07

    PURPOSE: The purpose of this function is to parse an inputted robtarget into the robtarget psoition, orientation, axis
             configuration, and external axis position. This is done by finding the first character in the rob target that
             is either a number or a negative. The end of the number is then found, which a substring can then be
             constructed and converted into a double. Below is an explaination of the unit conversion between ABB and ROS
             unit convention along with the ABB convention for a robtarget. This information is taken from the ABB RAPID
             Reference Manual.

    INSTRUCTIONS: Input a robtarget and a poseConfig structure will be returned containing the position, orientation, axis
                  configuration, and the external axis position (axis 7). If debug is set to true, the position,
                  orientation, configuration data, and external axis position will be displayed for the privded robtarget.


    -------------------- OTHER INFORMATION --------------------

    ABB UNITS: milimeters and degrees
    ROS UNITS: meters and radians

    ABB ROBTARGET CONVENTION: [[position.x,position.y,position.z],[orientation.w,orientation.x,orientation.y,orientation.z],[cf1,cf4,cf6,cfx],[eax1,eax2,eax3,eax4,eax5,eax6]]
        > cf1 represents the configuration data for axis 1
        > cf4 represents the configuration data for axis 4
        > cf6 represents the configuration data for axis 6
        > cfx represents the configuration data for a combination of axis 5, 3, 2, and a compatibility bit
        > eax1 represents the arm angle
        > eax2 to eax6 represents 5 other external axis values, but these are not used on YuMi and thus are all set to 9E+09. This data is ignored.
*/
    // INITIALIZE VARIABLES
    const double PI = 3.1415927;
    const double mm2m = 1.0/1000.0;
    const double deg2rad = PI / 180;
    std::string robtarget_search = "0123456789E.-+";

    poseConfig pose_config;
    std::vector<int> confdata(4);

    bool pose_pulled = 0;
    std::size_t start_char = 2;
    std::size_t end_char = 0;
    std::size_t E_location;
    int element = 0;

    // PARSE ROBTARGET
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

        if (element == 1) { pose_config.pose.position.x = current_val * mm2m; }
        else if (element == 2) { pose_config.pose.position.y = current_val * mm2m; }
        else if (element == 3) { pose_config.pose.position.z = current_val * mm2m; }
        else if (element == 4) { pose_config.pose.orientation.w = current_val; }
        else if (element == 5) { pose_config.pose.orientation.x = current_val; }
        else if (element == 6) { pose_config.pose.orientation.y = current_val; }
        else if (element == 7) { pose_config.pose.orientation.z = current_val; }
        else if (element == 8) { confdata[0] = ((int)current_val); }
        else if (element == 9) { confdata[1] = ((int)current_val); }
        else if (element == 10) { confdata[2] = ((int)current_val); }
        else if (element == 11) { confdata[3] = ((int)current_val); }
        else if (element == 12) { pose_config.external_axis_position = current_val * deg2rad; }
        else if (element >= 13) { break; }

        start_char = robtarget.find_first_of(robtarget_search, end_char);
    }

    pose_config.confdata = confdata;
 
    if (debug) {
        ROS_INFO("(debug) Position: %.5f, %.5f, %.5f | Orientation: %.5f, %.5f, %.5f, %.5f | Configuration: %d, %d, %d, %d | External Axis: %.5f",
            pose_config.pose.position.x, pose_config.pose.position.y, pose_config.pose.position.z, 
            pose_config.pose.orientation.w, pose_config.pose.orientation.x, pose_config.pose.orientation.y, pose_config.pose.orientation.z,
            pose_config.confdata[0], pose_config.confdata[1], pose_config.confdata[2], pose_config.confdata[3],
            pose_config.external_axis_position);
    }

    return pose_config;
}

/* ------------------------------------------------------------ */
/* ------------------- CONVERSION FUNCTIONS ------------------- */
/* ------------------------------------------------------------ */

trajectoryPoses combineModules(RAPIDModuleData& module_1, RAPIDModuleData& module_2, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-25 

    PURPOSE: The purpose of this function is to combine two RAPID module data structure into a pose trajectory. This function
             will only work properly or will produce and error if the two modules were meant to work together. These modules
             will be combined in a way that all MoveSync motions will be lined up. If any of the arms is moving independently
             (including closing and opening the grippers), the other arm will wait at it's current position until the next 
             MoveSync task is reached. This function assumes that grippers are on each arm.

    INSTRUCTIONS: This function requires two different modules to be supplied where one is meant for the left arm and the 
                  other is meant for the right arm. The designated move groups for each module should have already been
                  stored into the RAPID module structure from a previous function. If debug is set to true, the provided 
                  module names will be outputted to the user, and the pose names for each arm will be displayed as the final 
                  pose trajectory is being constructed by attempting to synconize motions from each module.

    NOTE: If the modules were not able to be comined, an empty trajectory is returned.


    -------------------- OTHER INFORMATION --------------------

    ASSUMPTIONS: Assuming grippers are being used on each arms when creating the RAPID modules. Gripper data can be execluded
                 later if a gripper is not attached to one or both of the arms.
*/
    ROS_INFO(">--------------------");

    // INITIALIZE VARIABLES
    trajectoryPoses pose_trajectory;
    pose_trajectory.group_name     = "both_arms";
    pose_trajectory.intended_group = "both_arms";

    std::vector<poseConfig> pose_configs_1, pose_configs_2;
    int index_1(0), index_2(0), index(0);
    bool success = true;

    // NOTIFY THE USER THE FUNCTION IS ABOUT TO START
    ROS_INFO("Combining modules.");

    if (debug) {
        ROS_INFO("(debug) Module 1 | Name: %s | Move group: %s", module_1.module_name.c_str(), module_1.group_name.c_str());
        ROS_INFO("(debug) Module 2 | Name: %s | Move group: %s", module_2.module_name.c_str(), module_2.group_name.c_str());
    }

    // CONSTRUCT TRAJECTORY WITH SYNCRONIZED MOTION BETWEEN ARMS
    tic();
    int max_points = std::max(module_1.total_points, module_2.total_points);
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
                /* If the current point doesn't involve a gripper open or gripper close command for the second module */
                    index_2--;
                }
            } else if ((module_1.pose_names[index_1].compare(0, 1, "p") != 0) && (module_2.pose_names[index_2].compare(0, 1, "p") == 0)) {
            /* If the second module is going to a non-syncronized point (designated by p#) and the first module is going to a syncronized point */
                if (module_1.pose_names[index_1].compare(module_1.pose_names[index_1-1]) != 0) {
                /* If the current point doesn't involve a gripper open or gripper close command for the first module */
                    index_1--;
                }
            } else if (module_1.pose_names[index_1].compare(module_1.pose_names[index_1-1]) == 0) {
            /* If the first module is closing or opening a gripper */
                index_2--;
            } else if (module_2.pose_names[index_2].compare(module_2.pose_names[index_2-1]) == 0) {
            /* If the second module is closing or opening a gripper */
                index_1--;
            } else if ((module_1.pose_names[index_1].compare(0, 1, "s") == 0) && (module_2.pose_names[index_2].compare(0, 1, "s") == 0)) {
            /* If both modules are at a MoveSync command, but the robtarget names are different meaning neither of them are able to continue execution */
                ROS_ERROR("From: combineModules(module_1, module_2, debug)");
                ROS_ERROR("Cannot combine provided modules. The MoveSync target names do not match up.");
                ROS_WARN("Line %d of module 1 main function and line %d of module 2 main function.", index_1+1, index_2+1);
                ROS_WARN("Module 1: %s", module_1.module_name.c_str());
                ROS_WARN("Module 2: %s", module_2.module_name.c_str());
                success = false;
                break;
            }
        } 

        pose_configs_1.push_back(module_1.pose_configs[index_1]);
        pose_configs_2.push_back(module_2.pose_configs[index_2]);

        index_1++;
        index_2++;
        index++;

        if (debug) { ROS_INFO("(debug) Line: %d | Module 1 point: %s | Module 2 point: %s", index, module_1.pose_names[index_1-1].c_str(), module_2.pose_names[index_2-1].c_str()); }
    }

    ROS_INFO("Modules combined.");

    // STORE CONSTRUCTED TRAJECTORIES TO CORRECT ARM
    if (success) {
        if (module_1.group_name.compare("left_arm") == 0) {
            pose_trajectory.pose_configs_left = pose_configs_1;
            pose_trajectory.gripper_attached_left = module_1.gripper_attached;

            pose_trajectory.pose_configs_right = pose_configs_2;
            pose_trajectory.gripper_attached_right = module_2.gripper_attached;
        } else {
            pose_trajectory.pose_configs_left = pose_configs_2;
            pose_trajectory.gripper_attached_left = module_2.gripper_attached;

            pose_trajectory.pose_configs_right = pose_configs_1;
            pose_trajectory.gripper_attached_right = module_1.gripper_attached;
        }

        ROS_INFO("Processing time: %.5f",toc());
        return pose_trajectory;
    } else {
        ROS_WARN("Returning empty pose trajectory.");
        ROS_INFO("Processing time: %.5f",toc());
        trajectoryPoses empty_trajectory;
        return empty_trajectory;
    }
}

trajectoryPoses convertModuleToPoseTrajectory(RAPIDModuleData& module, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-28

    PURPOSE: The purpose of this function is to convert the RAPIDModuleData structure into a trajectoryPoses structure.
             Conversions from poses to joint values are only performed through trajectoryPoses structure, meaning all
             stand along modules (modules that won't be combine with another arm) need to be run through this function
             first before convertin to joint values.

    INSTRUCTIONS: Supply a module, and all of the relevent information will transfered and returned. If debug is set to
                  true, then the user will be notified when the data transferring process has started and finished.
*/
    if (debug) { ROS_INFO(">--------------------"); }

    // INTIALIZE VARIABLES
    trajectoryPoses pose_trajectory;

    if (debug) { ROS_INFO("Converting module %s for group %s to a pose trajectory structure", module.module_name.c_str(), module.group_name.c_str()); }

    // TRANSFER DATA TO NEW STRUCTURE FROM MODULE
    pose_trajectory.group_name = pose_trajectory.intended_group = module.group_name;
    pose_trajectory.total_points   = module.total_points;

    if (module.group_name.compare("left_arm") == 0) {
        pose_trajectory.pose_configs_left     = module.pose_configs;
        pose_trajectory.gripper_attached_left = module.gripper_attached;
    } else {
        pose_trajectory.pose_configs_right     = module.pose_configs;
        pose_trajectory.gripper_attached_right = module.gripper_attached;
    }

    if (debug) { ROS_INFO("Conversion from RAPID module data to pose trajectory structure successful."); }

    return pose_trajectory;
}

trajectoryJoints convertPoseTrajectoryToJointTrajectory(planningInterface::MoveGroup& group, trajectoryPoses& pose_trajectory, kinematics::KinematicsBasePtr& ik_left, kinematics::KinematicsBasePtr& ik_right, const bool BOUNDED_SOLUTION, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-29

    PURPOSE: The purpose of this function is to convert pose trajectories into joint trajectories using the IK
             services for each arm that are supplied to this function. Unless debug mode has been set to true,
             if any of the trajectory points don't produce a valid IK solution, then this function will return 
             an empty joint trajectory and notify the user of the error. The program flow for this function is
             shown below.

    INSTRUCTIONS: Input the group that is associate for the provided pose trajectory, and input pointers to the
                  kinematic services for each arm. If debug mode is set to true, then the inputs to the IK serivce
                  for each pose to joint trajectory point conversion will be displayed to the user, along with the
                  solution for each IK service call, and each point in the newly constructed joint trajectory. The
                  move group provided to this function must be the same move group the pose trajectory was intended
                  for or this function will notify the user and exit.

    NOTE: An empty joint trajectory is returned if the function fails to convert the pose trajectory to joints.


    -------------------- OTHER INFORMATION --------------------

    PROGRAM FLOW:
        Left or right arm only:
            - Get the current joint values excluding the gripper position
            - Convert all the pose trajectories into joint trajectories
            - IF successful
                - Add the gripper position for each trajectory point if there is a gripper attached
                - Return the constructed joint trajectory
            - ELSE
                - Notify the user exit the function

        Both arms:
            - Get the current joints values for each arm and exclude the gripper positions
            - Convert all the pose trajectories for the left and right arm into joint trajectories
            - IF successful
                - Combine the joint trajectories for each arm into one joint trajectory
                - Add the gripper positions for each arm at each trajectory point if either arm has a gripper
                - Return the constructed joint trajectory
            - ELSE
                - Notify the user and exit the function
*/
    ROS_INFO(">--------------------");

    // INITIALIZE VARIABLES
    trajectoryJoints joint_trajectory_left, joint_trajectory_right, joint_trajectory;
    bool success_left, success_right, success = true, wrong_group_provided = false;

    // CONVERT POSE TRAJECTORY TO JOINT TRAJECTORY
    if (pose_trajectory.group_name.compare("left_arm") == 0) {
    /* If the provided pose trajectory is intended for the left arm */
        if (group.getName().compare("left_arm") == 0) {
        /* If the provided group is also for manipulating the left arm*/
            ROS_INFO("Converting pose trajectory to joint trajectory for the left arm.");

            std::vector<std::string> joint_names = group.getActiveJoints();
            std::vector<double>     joint_values = group.getCurrentJointValues();

            if (debug) {
                ROS_INFO("....................");
                ROS_INFO("(debug) Solving IK for left arm");
                if (BOUNDED_SOLUTION) {
                    ROS_INFO("(debug) Forcing bounded solutions to IK solver.");
                } else {
                    ROS_INFO("(debug) Not bounding solutions to IK solver");
                }
            }
            success_left = convertPoseConfigToJointTrajectory(joint_trajectory_left, ik_left, pose_trajectory.pose_configs_left, BOUNDED_SOLUTION, debug);
            

            if (success_left) {
                ROS_INFO("Successfully converted pose trajectory to joint trajectory for left arm.");
                ROS_INFO("Adding gripper values to trajectory if neccessary.");

                joint_trajectory_left.total_joints = joint_values.size();

                if (pose_trajectory.gripper_attached_left) {
                    for (int point = 0; point < joint_trajectory_left.total_points; point++) {
                    /* Override gripper positions producesd from the IK with the gripper data from the pose trajectory structure */
                        joint_trajectory_left.joints[point].push_back(pose_trajectory.pose_configs_left[point].gripper_position);

                        if (debug) {
                            ROS_INFO("_____ (debug) Joint Trajectory Point %d Joint Values _____", point+1);
                            for (int joint = 0; joint < joint_trajectory_left.total_joints; joint++) {
                                ROS_INFO("%s joint value: %.5f", joint_names[joint].c_str(), joint_trajectory_left.joints[point][joint]);
                            }
                        }
                    }
                } else {
                    for (int point = 0; point < joint_trajectory_left.total_points; point++) {
                        ROS_INFO("_____ (debug) Joint Trajectory Point %d Joint Values _____", point+1);
                        for (int joint = 0; joint < joint_trajectory_left.total_joints; joint++) {
                            ROS_INFO("%s joint value: %.5f", joint_names[joint].c_str(), joint_trajectory_left.joints[point][joint]);
                        }
                    }
                }
                joint_trajectory_left.group_name = joint_trajectory_left.intended_group = "left_arm";

                ROS_INFO("Pose trajectory to joint trajectory completed.");

                return joint_trajectory_left;
            } else {
                ROS_INFO("....................");
                ROS_ERROR("From: convertPoseTrajectoryToJointTrajectory(group, pose_trajectory, ik_left, ik_right, debug)");
                ROS_ERROR("Conversion from pose trajectory to joint trajectory was not successful for the left arm.");
                success = false;
            }
        } else {
            wrong_group_provided = true;
            success = false;
        }
    } else if (pose_trajectory.group_name.compare("right_arm") == 0) {
    /* If the provided pose trajectory is intended for the right arm */
        if (group.getName().compare("right_arm") == 0) {
        /* If the provided group is also for manipulating the right arm*/
            ROS_INFO("Converting pose trajectory to joint trajectory for the right arm.");

            std::vector<std::string> joint_names = group.getActiveJoints();
            std::vector<double>     joint_values = group.getCurrentJointValues();

            if (debug) {
                ROS_INFO("....................");
                ROS_INFO("(debug) Solving IK for right arm");
                if (BOUNDED_SOLUTION) {
                    ROS_INFO("(debug) Forcing bounded solutions to IK solver.");
                } else {
                    ROS_INFO("(debug) Not bounding solutions to IK solver");
                }
            }
            success_right = convertPoseConfigToJointTrajectory(joint_trajectory_right, ik_right, pose_trajectory.pose_configs_right, BOUNDED_SOLUTION, debug);

            if (success_right) {
                ROS_INFO("Successfully converted pose trajectory to joint trajectory for right arm.");
                ROS_INFO("Adding gripper values to trajectory if neccessary.");

                joint_trajectory_right.total_joints = joint_values.size();

                if (pose_trajectory.gripper_attached_right) {
                    for (int point = 0; point < joint_trajectory_right.total_points; point++) {
                    /* Override gripper positions producesd from the IK with the gripper data from the pose trajectory structure */
                        joint_trajectory_right.joints[point].push_back(pose_trajectory.pose_configs_right[point].gripper_position);

                        if (debug) {
                            ROS_INFO("_____ (debug) Joint Trajectory Point %d Joint Values _____", point+1);
                            for (int joint = 0; joint < joint_trajectory_right.total_joints; joint++) {
                                ROS_INFO("%s joint value: %.5f", joint_names[joint].c_str(), joint_trajectory_right.joints[point][joint]);
                            }
                        }
                    }
                } else {
                    for (int point = 0; point < joint_trajectory_right.total_points; point++) {
                        ROS_INFO("_____ (debug) Joint Trajectory Point %d Joint Values _____", point+1);
                        for (int joint = 0; joint < joint_trajectory_right.total_joints; joint++) {
                            ROS_INFO("%s joint value: %.5f", joint_names[joint].c_str(), joint_trajectory_right.joints[point][joint]);
                        }
                    }
                }
                joint_trajectory_right.group_name = joint_trajectory_right.intended_group = "right_arm";

                ROS_INFO("Pose trajectory to joint trajectory completed.");

                return joint_trajectory_right;
            } else {
                ROS_INFO("....................");
                ROS_ERROR("From: convertPoseTrajectoryToJointTrajectory(group, pose_trajectory, ik_left, ik_right, debug)");
                ROS_ERROR("Conversion from pose trajectory to joint trajectory was not successful for the right arm.");
                success = false;
            }
        } else {
            wrong_group_provided = true;
            success = false;
        }
    } else if (pose_trajectory.group_name.compare("both_arms") == 0) {
    /* If the provided pose trajectory is intended for both arms */
        if (group.getName().compare("both_arms") == 0) {
        /* If the provided group is also for manipulating both arms*/
            ROS_INFO("Converting pose trajectory to joint trajectory for both arms.");

            std::vector<std::string> joint_names = group.getActiveJoints();
            std::vector<double>     joint_values = group.getCurrentJointValues();

            if (debug) {
                ROS_INFO("....................");
                ROS_INFO("(debug) Solving IK for left arm");
                if (BOUNDED_SOLUTION) {
                    ROS_INFO("(debug) Forcing bounded solutions to IK solver.");
                } else {
                    ROS_INFO("(debug) Not bounding solutions to IK solver");
                }
            }
            success_left  = convertPoseConfigToJointTrajectory(joint_trajectory_left, ik_left, pose_trajectory.pose_configs_left, BOUNDED_SOLUTION, debug);
            if (debug) {
                ROS_INFO("....................");
                ROS_INFO("(debug) Solving IK for right arm");
                if (BOUNDED_SOLUTION) {
                    ROS_INFO("(debug) Forcing bounded solutions to IK solver.");
                } else {
                    ROS_INFO("(debug) Not bounding solutions to IK solver");
                }
            }
            success_right = convertPoseConfigToJointTrajectory(joint_trajectory_right, ik_right, pose_trajectory.pose_configs_right, BOUNDED_SOLUTION, debug);

            if (joint_trajectory_left.total_points != joint_trajectory_right.total_points) {
                ROS_INFO("....................");
                ROS_ERROR("From: convertPoseTrajectoryToJointTrajectory(group, pose_trajectory, ik_left, ik_right, debug)");
                ROS_ERROR("The total trajectory points from the provided pose trajectories for the right and left arm do not match up.");
                ROS_WARN("Left joint trajectory size: %d | Right joint trajectory size: %d", joint_trajectory_left.total_points, joint_trajectory_right.total_points);
                success_left = success_right = success = false;
            }

            if (((success_left) && (success_right)) && joint_trajectory_left.total_points > 0) {
                if (debug) {  ROS_INFO("...................."); }
                ROS_INFO("Conversion from pose trajectory to joint trajectory successful."); 
                ROS_INFO("Combining joint trajectories for the left and right arm.");

                joint_trajectory.total_joints = joint_values.size();
                joint_trajectory.total_points = joint_trajectory_left.total_points;

                for (int point = 0; point < joint_trajectory_right.total_points; point++) {
                /* Combine joint trajectories and override gripper positions producesd from the IK with the gripper data from the pose trajectory structure */
                    for (int joint = 0; joint < joint_trajectory.total_joints; joint++) {
                        if (joint < joint_trajectory_left.total_joints) {
                            joint_values[joint] = joint_trajectory_left.joints[point][joint];
                        } else if (joint >= (joint_trajectory_left.total_joints + pose_trajectory.gripper_attached_left) ) { 
                        /* If a gripper is attached to the left arm, skip joint value 8 (index 7) since that is reserved for the left gripper value */
                            joint_values[joint] = joint_trajectory_right.joints[point][joint-(joint_trajectory_left.total_joints+pose_trajectory.gripper_attached_left)];
                        }
                    }
                    joint_trajectory.joints.push_back(joint_values);

                    if (pose_trajectory.gripper_attached_left) {
                        joint_trajectory.joints[point][7] = pose_trajectory.pose_configs_left[point].gripper_position;
                        /* Gripper position for the left arm is index 7 since: 7 (joints left arm) + 1 (gripper left_arm) - 1 (index starts from 0) = 7 */
                        if (pose_trajectory.gripper_attached_right) {
                            joint_trajectory.joints[point][15] = pose_trajectory.pose_configs_right[point].gripper_position;
                            /* Gripper position for the right arm is index 15 since the left arm has a gripper meaning: 7 (joints left arm) + 1 (gripper left arm) + 7 (joints right arm) + 1 (gripper right arm) - 1 (index starts from 0) = 15 */
                        }
                    } else if (pose_trajectory.gripper_attached_right) {
                    /* If there is no gripper attached to the left arm but there is a gripper attached to the right arm */
                        joint_trajectory.joints[point][14] = pose_trajectory.pose_configs_right[point].gripper_position;
                        /* Gripper position for the right arm is index 14 since the left arm does not have a gripper meaning: 7 (joints left arm) + 7 (joints right arm) + 1 (gripper right arm) - 1 (index starts from 0) = 14 */
                    }

                    if (debug) {
                        ROS_INFO("_____ (debug) Joint Trajectory Point %d Joint Values _____", point+1);
                        for (int joint = 0; joint < joint_values.size(); joint++) {
                            ROS_INFO("%s joint value: %.5f", joint_names[joint].c_str(), joint_values[joint]);
                        }
                    }
                }
                joint_trajectory.group_name = joint_trajectory.intended_group = "both_arms";

                if (debug) { ROS_INFO("...................."); }
                ROS_INFO("Pose trajectory to joint trajectory completed."); 

                return joint_trajectory;
            } else {
                if (success) {
                /* If this line has not already been displayed previously */
                    ROS_INFO("....................");
                    ROS_ERROR("From: convertPoseTrajectoryToJointTrajectory(group, pose_trajectory, ik_left, ik_right, debug)");
                }
                if ((!success_left) && (!success_right)) {
                    ROS_ERROR("Conversion from pose trajectory to joint trajectory was not successful for both arms.");
                } else if (!success_left) {
                    ROS_ERROR("Conversion from pose trajectory to joint trajectory was not successful for the left arm.");
                } else if (!success_right) {
                    ROS_ERROR("Conversion from pose trajectory to joint trajectory was not successful for the right arm.");
                }

                if ((joint_trajectory.total_points == 0) && (success)) {
                    ROS_ERROR("The joint trajectory is empty. Not able to combine empty trajectories.");
                }

                success = false;
            }
        } else {
            wrong_group_provided = true;
            success = false;
        }
    }

    if (!success) {
        if (wrong_group_provided) {
            ROS_ERROR("From: convertPoseTrajectoryToJointTrajectory(group, pose_trajectory, ik_left, ik_right, debug)");
            ROS_ERROR("Provided group does not match with the group that the provided pose_trajectory was intended for.");
            ROS_WARN("Expected group: %s", pose_trajectory.group_name.c_str());
            ROS_WARN("Provided group: %s", group.getName().c_str());
        }

        ROS_WARN("Returning empty joint trajectory.");
        trajectoryJoints empty_trajectory;
        return empty_trajectory;
    }
}

bool convertPoseConfigToJointTrajectory(trajectoryJoints& joint_trajectory, kinematics::KinematicsBasePtr& ik, std::vector<poseConfig>& pose_configs, const bool BOUNDED_SOLUTION, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-28

    PURPOSE: The purpose of this function is to call the inverse kinematics service to conver from coordinate space to
             joint space. This is done by bounding the solution based on the configuration data and external axis
             position stored within the pose_configs parameter. In the case that debug mode has been set to true, then
             if an IK for a given point fails, the function will try finding a solution without bounds. If a solution is
             found, then that solution is stored. Otherwise the funciton will skip that trajectory point and move onto 
             the next trajectory point. If debug was not set to true and the IK service failed for any of the trajectory 
             points, then this function will return indicated the pose to joint conversion failed. For each iteration 
             of the IK service loop, the seed will be updated with the most recent successful solutions generated from 
             the IK service.

    INSTRUCTIONS: This function required an input of a joint trajectory in which this function will add the solutions
                  from the IK service, and will return a boolean to indicate of the conversion was successful or not. 
                  The inputted IK service pointer is expected to be the intended for the same group as the pose_configs.
                  If debug is set to true, the user will be notified of all contraints and values supplied to the IK 
                  service for each trajectory point, and in the case that the IK service fails, the current trajectory
                  point will be skipped and the function will preceed on to the next trajectory point. If debug is set
                  to false, then the function will stop executing whenever the IK solver failed for a trjaectory point.
                  The BOUNDED_SOLUTION input variable can be used if the user would not like to bound the solution
                  according to the axis configurations.


    -------------------- OTHER INFORMATION --------------------

    ASSUMPTIONS: The gripper position is not included in the initial_seed parameter regardless of whether a gripper is
                 attached to the desired group or not. In that case, the initial_seed is expected to have a size of 7
                 for the 7 revolute joints on each arm of the YuMi. The IK solver is also expected to be for the same 
                 move group that the pose_configs variable was intended for. 

    CONSISTENCY: The consistency parameter passed into the function provides the bounds for the solution based on the
                 respective seed value. For example, if the consistency value and respective seed value were 2 and 4
                 for a given joint, then the minimum and maximum values that can be returned from the IK service are
                 2 and 6 respectively.

    ABB YuMi AXIS NAMING CONVENTION (from YuMi base to end effector): [1, 2, 7, 3, 4, 5, 6]

    ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

    CONFIGURATION SETS: -4 -> [-360, -270)   -3 -> [-270, -180)   -2 -> [-180, -90)   -1 -> [-90,   0)
                         0 -> [0,      90)    1 -> [  90,  180)    2 -> [180,  270)    3 -> [270, 360)
        - The configuration values above are for joints 1, 4, and 6

    CONFIGURATION EQUATION FOR SEED: (config * PI/2) + PI/4 (radians) or (config * 90) + 45 (degrees)
        - The above equation is used to convert a configuration value for an axis into a seed position. The seed 
          position is located in the center of the quadrant according the the ABB convention for configuration 
          values as shown above in CONFIGURATION SETS. The bounds for the IK solver (if used) is then set for PI/4
          or 45 degrees to go from the max of the quadrant to the minimum of the quadrant

    CFX: ABCD
        - A represents configuration for axis 5 and can be either (1) or (0)
            > (0) if axis 5 position >= 0 degrees, (1) if axis 5 position < 0 degrees
        - B represents configuration for axis 3 and can be either (1) or (0)
            > (0) if axis 3 position >= -90 degrees, (1) if axis 3 position < -90 degrees
        - C represents configuration for axis 2 and can be either (1) or (0)
            > (0) if axis 2 position >= 0 degrees, (1) if axis 2 position < 0 degrees
        - D represents the compatability bit, particulary used for linear movements
            > This value is not used for the IK solver

    EXTERNAL AXIS POSITION CONVENTION: [arm_angle]

    ERROR CODE DESCRIPTIONS: http://docs.ros.org/indigo/api/moveit_msgs/html/msg/MoveItErrorCodes.html
*/
    ROS_INFO("....................");

    // INITIALIZE VARIABLES
    const double PI = 3.14159;
    const double TOLERANCE = PI/72.0; // 2.5 degrees tolerance
    const std::vector<int> confdata_axis_146 = { 0, 4, 6 }; // index for axis 1, 4, and 6
    const int total_joints = 7;
    int cfx, axis_2_config(0), axis_3_config(0), axis_5_config(0);

    double timeout = 1;
    std::vector<double> seed(total_joints);
    std::vector<double> consistency = {PI/4.0+TOLERANCE, PI+TOLERANCE, 2.0*PI, PI+TOLERANCE, PI/4.0+TOLERANCE, PI+TOLERANCE, PI/4.0+TOLERANCE}; // tolerance for seeded values (min: seed[i]-consistency[i] | max: seed[i]+consistency[i])
    /* An addition tolerance of 2.5 degrees is added to each bound in the case that the solution is on some interval of PI/2 */
    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error_code;

    int solutions_stored = 0; // indicates how many IK solutions were found and it is not incremented if BOUNDED_SOLUTION is set to true and a solution was only found without bounds
    int previous_axis_7_solution;
    bool previous_solution_unbounded = false;
    bool previous_solution_failed    = true; // initially set to true in order to ensure the first if statement in the for loop below is not executed
    bool success;

    tic();

    // PERFORM IK
    for (int pose = 0; pose < pose_configs.size(); pose++) {
        if (debug) { ROS_INFO("(debug) Performing IK calculation for index %d.", pose+1); }

        // CHECK IF CURRENT POSE CONFIGURATION IS SIMILAR ENOUGH TO THE PREVIOUS ONE
        if (!previous_solution_failed) {
        /* If the previous solution did not fail */
            if (isClose(pose_configs[pose-1], pose_configs[pose], debug)) {
            /* If the current pose configuration is close enough to the previous one, then use the previous solution */
                ROS_INFO("(Index %d) Previous pose configuration similar to current pose configuration. Using previous solution.", pose+1);
                joint_trajectory.joints.push_back(solution);
                if ((BOUNDED_SOLUTION) && (!previous_solution_unbounded)) {
                /* If solutions are to be bounded and the previous solution was not unbounded due to a 
                   failed bounded solution. If debug mode is set to true and bounded solutions are set 
                   to true, then if the IK fails for the bounded solution, IK is reattempted using an 
                   unbounded solution. */
                    solutions_stored++;
                }
                if (debug) { ROS_INFO("...................."); }

                continue;
            }
        }

        // GET AXIS CONFIGURATIONS FOR AXIS 2, 3, and 5
        cfx = pose_configs[pose].confdata.back();
        axis_2_config = axis_3_config = axis_5_config = 0;
        success = parseCFX(cfx, axis_2_config, axis_3_config, axis_5_config);

        // ADJUST SEED BASED ON AXIS CONFIGURATIONS
        for (int config = 0; config < confdata_axis_146.size(); config++) {
            seed[confdata_axis_146[config]] = (pose_configs[pose].confdata[config] * PI/2) + PI/4;
        }

        if (axis_2_config) { seed[1] = -PI; }
        else { seed[1] = PI; }
        if (axis_5_config) { seed[5] = -PI; }
        else { seed[5] = PI; }
        if (axis_3_config) { 
            seed[3] = ((5.0*PI)/4.0); // -225 degree seed
            consistency[3] = ((3.0*PI)/4.0) + TOLERANCE; // 135 degree bound with 2.5 degree tolerance
        } else { 
            seed[3] = ((3.0*PI)/4.0); // 135 degree seed
            consistency[3] = ((5.0*PI)/4.0) + TOLERANCE; // 225 degree bound with 2.5 degree tolerance
        }

        if (solutions_stored > 0) {
            seed[2] = previous_axis_7_solution;
        } else {
            seed[2] = pose_configs[pose].external_axis_position;
        }

        if (debug) {
            ROS_INFO("_____ (debug) IK Inputs for Index %d _____", pose+1);
            ROS_INFO(" *Note: for orientation, the w value is provided first*");
            ROS_INFO("Pose        | Position: [%.5f, %.5f, %.5f] | Orientation: [%.5f, %.5f, %.5f, %.5f]", 
                pose_configs[pose].pose.position.x, pose_configs[pose].pose.position.y, pose_configs[pose].pose.position.z,
                pose_configs[pose].pose.orientation.w, pose_configs[pose].pose.orientation.x, pose_configs[pose].pose.orientation.y, pose_configs[pose].pose.orientation.z);
            ROS_INFO("Config Data | Values: [%d, %d, %d, %d] | External Axis: [%.5f]", pose_configs[pose].confdata[0], pose_configs[pose].confdata[1], pose_configs[pose].confdata[2], 
                pose_configs[pose].confdata[3], pose_configs[pose].external_axis_position);
            ROS_INFO("Seed        | Joint values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6]);
            if (BOUNDED_SOLUTION) {
                ROS_INFO("Consistency | Values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", consistency[0], consistency[1], consistency[2], consistency[3], 
                    consistency[4], consistency[5], consistency[6]);

                ROS_INFO("_____ (debug) Solution Bounds for Index %d _____", pose+1);
                ROS_INFO("Axis 1: %.5f to %.5f", seed[0]-consistency[0], seed[0]+consistency[0]);
                ROS_INFO("Axis 2: %.5f to %.5f", seed[1]-consistency[1], seed[1]+consistency[1]);
                ROS_INFO("Axis 7: %.5f to %.5f", seed[2]-consistency[2], seed[2]+consistency[2]);
                ROS_INFO("Axis 3: %.5f to %.5f", seed[3]-consistency[3], seed[3]+consistency[3]);
                ROS_INFO("Axis 4: %.5f to %.5f", seed[4]-consistency[4], seed[4]+consistency[4]);
                ROS_INFO("Axis 5: %.5f to %.5f", seed[5]-consistency[5], seed[5]+consistency[5]);
                ROS_INFO("Axis 6: %.5f to %.5f", seed[6]-consistency[6], seed[6]+consistency[6]);
            }
        }

        // COMPUTE IK
        if (BOUNDED_SOLUTION) {
            success = ik->searchPositionIK(pose_configs[pose].pose, (const std::vector<double>)seed, timeout, (const std::vector<double>)consistency, solution, error_code);
        } else {
            success = ik->searchPositionIK(pose_configs[pose].pose, (const std::vector<double>)seed, timeout, solution, error_code);
        }

        if (debug) {
            ROS_INFO("____ (debug) Joint Value Solution for Index %d _____", pose+1);
            if (error_code.val != 1) {
                previous_solution_failed = true;
                ROS_WARN("Solution not found. Error code: %d", error_code.val);
            } else {
                previous_solution_unbounded = previous_solution_failed = false;

                int joint_7_index_adjust = 0;
                for (int joint = 0; joint < solution.size(); joint++) {
                    if (joint == 2) {
                    /* The position for joint 7 is the third value in the solution */
                        ROS_INFO("Joint %d value: %.5f", 7, solution[joint]);
                        joint_7_index_adjust = -1; // decement the displayed joint number for the remaining joints since the third value in the solution is joint 7
                    } else {
                        ROS_INFO("Joint %d value: %.5f", joint+1+joint_7_index_adjust, solution[joint]);
                    }
                }
            }
        }

        if (success) {
            ROS_INFO("(Index %d) IK calculation was successful.", pose+1);
            joint_trajectory.joints.push_back(solution);
            previous_axis_7_solution = solution[2];
            solutions_stored++;
        } else {
            ROS_WARN("(Index %d) IK calculation failed.", pose+1);

            if (debug) {
                if (BOUNDED_SOLUTION) {
                    ROS_WARN("(debug) Trying again without solution bounds.");
                    success = ik->searchPositionIK(pose_configs[pose].pose, (const std::vector<double>)seed, timeout, solution, error_code);

                    ROS_INFO("____ (debug) Joint Value Solution for Index %d _____", pose+1);
                    if (error_code.val != 1) {
                        ROS_WARN("Solution not found. Error code: %d", error_code.val);
                    } else {
                        previous_solution_unbounded = true;
                        previous_solution_failed    = false;

                        int joint_7_index_adjust = 0;
                        for (int joint = 0; joint < solution.size(); joint++) {
                            if (joint == 2) {
                            /* The position for joint 7 is the third value in the solution */
                                ROS_INFO("Joint %d value: %.5f", 7, solution[joint]);
                                joint_7_index_adjust = -1; // decement the displayed joint number for the remaining joints since the third value in the solution is joint 7
                            } else {
                                ROS_INFO("Joint %d value: %.5f", joint+1+joint_7_index_adjust, solution[joint]);
                            }
                        }
                    }
                } else {
                    success = false;
                }

                if (success) {
                    ROS_WARN("(debug) Solution found without bounds. Storing unbounded solution");
                    joint_trajectory.joints.push_back(solution);
                } else {
                    ROS_WARN("(debug) Skipping trajectory point.");
                }
            } else {
                ROS_ERROR("From: convertPoseConfigToJointTrajectory(joint_trajectory, ik, pose_configs, BOUNDED_SOLUTION, debug)");
                ROS_ERROR("Not able to find a solution for trajectory point %d.", pose+1);
                ROS_WARN("Exiting function.");
                return false;
            }
        }

        if (debug) { ROS_INFO("...................."); }
    }

    joint_trajectory.total_points = joint_trajectory.joints.size();
    joint_trajectory.total_joints = total_joints;

    if ((debug) && (BOUNDED_SOLUTION)) {
        ROS_INFO("Able to convert %d of %lu poses to joints with bounded solutions.", solutions_stored, pose_configs.size());
        ROS_INFO("Able to convert %d of %lu poses to joints with unbounded/bounded solutions.", joint_trajectory.total_points, pose_configs.size());
    } else {
        ROS_INFO("Able to convert %d of %lu poses to joint trajectory points.", solutions_stored, pose_configs.size());
    }
    ROS_INFO("Processing time: %.5f",toc());
    return true;
}

bool parseCFX(int& cfx, int& axis_2_config, int& axis_3_config, int& axis_5_config, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-04

    PURPOSE: The purpose of this function is to parse the CFX configuraiton value from the ABB
             confdata convention in order to determine the configurations for axes 2, 3, and 5.
             The ABB convention for confdata is shown below. These cafinguration values can be
             either 1 or 0 indicating bounds of where the joint angle solution for the given
             axis lies according to the cutoffs shown below.

    INSTRUCTIONS: Supply the CFX value to this function, as well as variables for storing the
                  config values for axis 2, 3, and 5. the config variables are passed by
                  reference and this function will update their values according to the given
                  CFX according to the cenvention showed below. If debug is set to true, then
                  the configruation values will be displayed after parsing the CFX value.


    -------------------- OTHER INFORMATION --------------------

    ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

    CFX: ABCD
        - A represents configuration for axis 5 and can be either (1) or (0)
            > (0) if axis 5 position >= 0 degrees, (1) if axis 5 position < 0 degrees
        - B represents configuration for axis 3 and can be either (1) or (0)
            > (0) if axis 3 position >= -90 degrees, (1) if axis 3 position < -90 degrees
        - C represents configuration for axis 2 and can be either (1) or (0)
            > (0) if axis 2 position >= 0 degrees, (1) if axis 2 position < 0 degrees
        - D represents the compatability bit, particulary used for linear movements
            > This value is not used for the IK solver

    EXAMPLES:
        - CFX Value: 0 (Note: leading zeros are removed)
            - Axis 5 position: >= 0 degrees   -> 0
            - Axis 3 position: >= -90 degress -> 0
            - Axis 2 position: >= 0 degrees   -> 0
            - Compatability bit: 0            -> 0
        - CFX Value: 1010
            - Axis 5 position: <  0 degrees   -> 1
            - Axis 3 position: >= -90 degress -> 0
            - Axis 2 position: <  0 degrees   -> 1
            - Compatability bit: 0            -> 0
        - CFX Value: 110
            - Axis 5 position: >= 0 degrees   -> 0
            - Axis 3 position: <  -90 degress -> 1
            - Axis 2 position: <  0 degrees   -> 1
            - Compatability bit: 0            -> 0
*/
    // INITIALIZE VARIABLES
    int digits;

    // DETERMINE TOTAL DIGITS IN CFX
    if (cfx != 0) {
        digits = floor(log10((double)cfx)) + 1;
    } else {
        digits = 1;
    }

    // GET CONFIGURATION DATA FOR AXIS 2, 3, and 5
    if (digits > 1) {
        if (digits == 4) {
            axis_5_config = 1;
            cfx -= 1000; // take out the axis_4_config value
            if (cfx >= 100) {
            /* If the third value from the right is a 1 */
                axis_3_config = 1;
                cfx -= 100; // take out the axis_3_config value
            }
            if (cfx >= 10) {
            /* If the second value from the right is a 1 */
                axis_2_config = 1;
            }
        } else if (digits == 3) {
            axis_3_config = 1;
            cfx -= 100; // take out the axis_3_config value
            if (cfx >= 10) {
            /* If the second value from the right is a 1 */
                axis_2_config = 1;
            }
        } else if (digits == 2) {
            axis_2_config = 1;
        } else {
            ROS_FATAL("From: parseCFX(cfx, axis_2_config, axis_3_config, axis_5_config)");
            ROS_FATAL("CFX (4th axis config value) for current pose has more than the total expected digits.");
            ROS_ERROR("Inputted file does not have the proper ABB convention.");
            ROS_WARN("Max expected digits: 4");
            ROS_WARN("Digits: %d", digits);
            ROS_WARN("Exiting function.");
            return false;
        }
    }

    if (debug) {
        ROS_INFO("CFX Value: %d", cfx);
        ROS_INFO("Axis 5 Config: %d | Axis 3 Config: %d | Axis 2 Config: %d", axis_5_config, axis_3_config, axis_2_config);
    }

    return true;
}

bool isClose(poseConfig& pose_config_1, poseConfig& pose_config_2, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-05

    PURPOSE: The purpose of this function is to determine whether the two inputted pose configrations
             are similar enough to be considered the same point. This is done by comparing first the
             configuration data for axis 1 through 7, comparing the difference in position for x, y, 
             and z, comparing the difference in orientation for w, x, y, and z, and finally comparing
             the external axis values for each pose configuration. The positions, orientations, and 
             external axis values are considered close enough based on the tolerances set at the
             beginning of the function. If they are considered close enough based on the tolerances,
             the function returns true, otherwise it returns false.

    INSTRUCTIONS: Input two pose configurations and this function will return true if they are 
                  considered close enough based on the tolerances set at the beginning of this function,
                  otherwise the function returns false. If debug is set to true, then the tolerances,
                  both poses, and each step in the tolerance tests are displayed to the user. A final
                  output is displayed to the user saying whether the pose configurations were determined
                  to be close enough or not based on the set tolerances. 
*/
    // INITIALIZE VARIABLES
    const double TOL_POSITION = 0.002; // 2 mm tolerance for position
    const double TOL_ORIENT   = 0.01; // 0.01 tolerance for orientation (quaternion) and about 0.5 degree tolerance for arm angle

    double x_tol_position = std::abs(pose_config_1.pose.position.x - pose_config_2.pose.position.x);
    double y_tol_position = std::abs(pose_config_1.pose.position.y - pose_config_2.pose.position.y);
    double z_tol_position = std::abs(pose_config_1.pose.position.z - pose_config_2.pose.position.z);

    double w_tol_orientation = std::abs(pose_config_1.pose.orientation.w - pose_config_2.pose.orientation.w);
    double x_tol_orientation = std::abs(pose_config_1.pose.orientation.x - pose_config_2.pose.orientation.x);
    double y_tol_orientation = std::abs(pose_config_1.pose.orientation.y - pose_config_2.pose.orientation.y);
    double z_tol_orientation = std::abs(pose_config_1.pose.orientation.z - pose_config_2.pose.orientation.z);

    double arm_angle_tol = abs(pose_config_1.external_axis_position - pose_config_2.external_axis_position);

    bool similar = true;

    if (debug) {
        ROS_INFO("_____ (debug) Tolerances Allowed for Similarity Comparison _____");
        ROS_INFO("   Position tolerance: %.5f", TOL_POSITION);
        ROS_INFO("Orientation tolerance: %.5f", TOL_ORIENT);
        ROS_INFO("  Arm angle tolerance: %.5f", TOL_ORIENT);

        ROS_INFO("_____ (debug) Pose Configurations Being Compared _____");
        ROS_INFO(" *Convention: [[position],[orientation (w is first)],[configuration_data],[arm_angle]]");
        ROS_INFO("Pose 1: [[%.5f, %.5f, %.5f],[%.5f, %.5f, %.5f, %.5f],[%d, %d, %d, %d],[%.5f]]",
            pose_config_1.pose.position.x, pose_config_1.pose.position.y, pose_config_1.pose.position.z,
            pose_config_1.pose.orientation.w, pose_config_1.pose.orientation.x, pose_config_1.pose.orientation.y, pose_config_1.pose.orientation.z,
            pose_config_1.confdata[0], pose_config_1.confdata[1], pose_config_1.confdata[2], pose_config_1.confdata[3],
            pose_config_1.external_axis_position);
        ROS_INFO("Pose 2: [[%.5f, %.5f, %.5f],[%.5f, %.5f, %.5f, %.5f],[%d, %d, %d, %d],[%.5f]]",
            pose_config_2.pose.position.x, pose_config_2.pose.position.y, pose_config_2.pose.position.z,
            pose_config_2.pose.orientation.w, pose_config_2.pose.orientation.x, pose_config_2.pose.orientation.y, pose_config_2.pose.orientation.z,
            pose_config_2.confdata[0], pose_config_2.confdata[1], pose_config_2.confdata[2], pose_config_2.confdata[3],
            pose_config_2.external_axis_position);
    }

    // DETERMINE IS POSE CONFIGURATIONS ARE CLOSE
    if (debug) { ROS_INFO("_____ (debug) Tolerance Test Feedback _____"); }
    if (pose_config_1.confdata == pose_config_2.confdata) {
    /* If the previous and current pose configs have the same configuration data */
        if (debug) { ROS_INFO("(debug) The pose configurations have the same axis configurations."); }
        if ((x_tol_position < TOL_POSITION) && (y_tol_position < TOL_POSITION) && (z_tol_position < TOL_POSITION)) {
        /* If the previous and current pose configs have similar position values */
            if (debug) { ROS_INFO("(debug) The pose configurations have a similar position."); }
            if ((w_tol_orientation < TOL_ORIENT) && (x_tol_orientation < TOL_ORIENT) && (y_tol_orientation < TOL_ORIENT) && (z_tol_orientation < TOL_ORIENT)) {
            /* If the previous and current pose configs have similar orientation values */
                if (debug) { ROS_INFO("(debug) The pose configurations have a similar orientation."); }
                if (arm_angle_tol < TOL_ORIENT) {
                /* If the previous and current pose configs have similar arm angles */
                    if (debug) { ROS_INFO("(debug) The pose configurations have a similar arm angle."); }
                } else {
                    if (debug) { ROS_INFO("(debug) The pose configurations do not have a similar arm angle."); }
                    similar = false;
                }
            } else {
                if (debug) { ROS_INFO("(debug) The pose configurations do not have a similar orientation."); }
                similar = false;
            }
        } else {
            if (debug) { ROS_INFO("(debug) The pose configurations do not have a similar position."); }
            similar = false;
        }
    } else {
        if (debug) { ROS_INFO("(debug) The pose configurations do not have the same axis configurations."); }
        similar = false;
    }

    if (debug) {
        if (similar) { ROS_INFO("(debug) Pose configurations are considered to be the same position according to tolerances."); }
        else { ROS_INFO("(debug) Pose configurations are not considered to be the same position according to tolerances."); }
    }
    
    return similar;
}

/* ------------------------------------------------------------ */
/* ------------- PLANNING AND EXECUTION FUNCTIONS ------------- */
/* ------------------------------------------------------------ */
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
    ROS_INFO(">--------------------");

    // CHECK INPUTS
    if (joint_trajectory.group_name.compare(group.getName()) != 0) {
        ROS_INFO("....................");
        ROS_ERROR("From: generatePlans(group, joint_trajectory, debug)");
        ROS_ERROR("Provided joint trajectory is not meant for the provided group.");
        if (joint_trajectory.group_name.compare("") == 0) {
            ROS_WARN("Group name for joint trajectory is empty.");
            ROS_WARN("This is likely due to a failure when converting to a joint trajectory.");
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
    ROS_INFO("Creating plans for provided joint trajectory.");

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

            ROS_INFO("Plan created for trajectory point: %d of %d", plan+1, joint_trajectory.total_points);
        } else {
            ROS_WARN("Plan failed for trajectory point: %d of %d", plan+1, joint_trajectory.total_points);

            if (debug) {
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
    ROS_INFO("Finished planning. Total plans created: %d of %d", plans.total_plans, joint_trajectory.total_points);
    
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


