// INCLUDES
#include <sstream>
#include <fstream>
#include <string>
#include <stack>
#include <ctime>
#include <math.h>

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
    std::string groupName; // include group name
    std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to retrieve planned paths for trajectory points
    int totalPlans; // include count of total plans
    bool success; // include boolean to retrieve whether the planner was successful or not
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
/* Configuration Functions */
poseConfig getCurrentPoseConfig(planningInterface::MoveGroup&, bool debug = false);
poseConfig getAxisConfigurations(planningInterface::MoveGroup&, bool debug = false);
poseConfig getAxisConfigurations(std::vector<double>, bool debug = false);

/* RAPID Module Retrieval/Conversion Functions */
RAPIDModuleData getYuMiLeadThroughData(std::string, planningInterface::MoveGroup&, bool debug = false);
poseConfig getRobtargetData(std::string, bool debug = false);
trajectoryPoses combineModules(RAPIDModuleData&, RAPIDModuleData&, bool debug = false);
trajectoryPoses convertModuleToPoseTrajectory(RAPIDModuleData&, bool debug = false);

/* Pose Trajectory to Joint Trajectory Functions */
trajectoryJoints convertPoseTrajectoryToJointTrajectory(planningInterface::MoveGroup&, trajectoryPoses&, kinematics::KinematicsBasePtr&, kinematics::KinematicsBasePtr&, bool debug = false);
bool convertPoseConfigToJointTrajectory(trajectoryJoints&, kinematics::KinematicsBasePtr&, std::vector<poseConfig>&, std::vector<double>, bool debug = false);

// MAIN FUNCTION
int main(int argc, char **argv) {

    ros::init (argc, argv, "test");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // INITIALIZE VARIABLES
    bool debug = true;
    std::string end_effector_left    = "yumi_link_7_l";
    std::string end_effector_right   = "yumi_link_7_r";
    std::string pose_reference_frame = "yumi_body";

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
    success_ik_left  = ik_left->initialize("/robot_description", "left_arm_ik", pose_reference_frame, end_effector_left, 0.001);
    success_ik_right = ik_right->initialize("/robot_description", "right_arm_ik", pose_reference_frame, end_effector_right, 0.001);
    // success_ik_left  = ik_left->initialize("/robot_description", "left_arm", pose_reference_frame, left_arm.getEndEffectorLink(), 0.001);
    // success_ik_right = ik_right->initialize("/robot_description", "right_arm", pose_reference_frame, right_arm.getEndEffectorLink(), 0.001);

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

    RAPIDModuleData module_left  = getYuMiLeadThroughData("Left_ASL_Demo", left_arm, debug);
    RAPIDModuleData module_right = getYuMiLeadThroughData("Right_ASL_Demo", right_arm, debug);

    trajectoryPoses pose_trajectory = combineModules(module_left, module_right, debug);
    trajectoryJoints joint_trajectory = convertPoseTrajectoryToJointTrajectory(both_arms, pose_trajectory, ik_left, ik_right, debug);


    // geometry_msgs::Pose poser;
    // poser.position.x = 0.353766;
    // poser.position.y = -0.116135;
    // poser.position.z = 0.26376;
    // poser.orientation.x = 0.0420612;
    // poser.orientation.y = 0.862708;
    // poser.orientation.z = 0.421826;
    // poser.orientation.w = 0.275734;

    // // INITIALIZE VARIABLES
    // const double PI = 3.14159;
    // const double TOLERANCE = 0.01;
    // const std::vector<int> confdata_joints = { 0, 4, 6 };

    // double timeout = 10.0;
    // std::vector<double> seed = right_arm.getCurrentJointValues();
    // std::vector<double> consistency; // tolerance from seed
    // std::vector<double> solution;
    // std::vector<double> previous_solution = seed;
    // moveit_msgs::MoveItErrorCodes error_code;

    // bool gripper_attached = false;
    // bool success;

    // std::vector<poseConfig> pose_configs;
    // pose_configs = pose_trajectory.pose_configs_right;
    // trajectoryJoints joint_trajectory;
    // pose_configs[0].pose = poser;

    // seed[2] = pose_configs[0].external_axis_position;

    // if (seed.size() == 8) {
    //     gripper_attached = true;
    // }

    // // TRANSFER DATA TO JOINT TRAJECTORY STRUCTURE
    // joint_trajectory.total_joints = seed.size();

    // // DETERMINE CONSISTANCY VARIABLE SIZE BASED ON GRIPPER ATTACHED
    // if (gripper_attached) {
    //     consistency = {PI/4, PI, PI/32, PI, PI/4, PI, PI/4, PI};
    // } else {
    //     consistency = {PI/4, PI, PI/32, PI, PI/4, PI, PI/4};
    // }

    // int pose = 0;

    // // SEED AXIS CONFIGURATIONS
    // for (int config = 0; config < confdata_joints.size(); config++) {
    //     seed[confdata_joints[config]] = (pose_configs[pose].confdata[config] * PI/2) + PI/4;
    // }

    // if (debug) {
    //     ROS_INFO("_____ (debug) IK Inputs for Index %d _____", pose+1);
    //     ROS_INFO(" *Note: for orientation, the w value is provided first*");
    //     ROS_INFO("Pose        | Position: [%.5f, %.5f, %.5f] | Orientation: [%.5f, %.5f, %.5f, %.5f]", 
    //         pose_configs[pose].pose.position.x, pose_configs[pose].pose.position.y, pose_configs[pose].pose.position.z,
    //         pose_configs[pose].pose.orientation.w, pose_configs[pose].pose.orientation.x, pose_configs[pose].pose.orientation.y, pose_configs[pose].pose.orientation.z);
    //     ROS_INFO("Config Data | Values: [%d, %d, %d, %d]", pose_configs[pose].confdata[0], pose_configs[pose].confdata[1], pose_configs[pose].confdata[2], pose_configs[pose].confdata[3]);
    //     if (gripper_attached) {
    //         ROS_INFO("Seed        | Joint values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6], seed[7]);
    //         ROS_INFO("Consistency | Values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", consistency[0], consistency[1], consistency[2], consistency[3], consistency[4], 
    //             consistency[5], consistency[6], consistency[7]);
    //     } else {
    //         ROS_INFO("Seed        | Joint values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6]);
    //         ROS_INFO("Consistency | Values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", consistency[0], consistency[1], consistency[2], consistency[3], 
    //             consistency[4], consistency[5], consistency[6]);
    //     }
    // }

    // // COMPUTE IK
    // success = ik_right->searchPositionIK(pose_configs[pose].pose, seed, timeout, consistency, solution, error_code);

    // if (debug) {
    //     ROS_INFO("____ (debug) Joint Value Solution for Index %d _____", pose+1);
    //     if (error_code.val != 1) {
    //         ROS_INFO("Solution not found.");
    //     } else {
    //         for (int joint = 0; joint < solution.size(); joint++) {
    //             ROS_INFO("Joint value: %.5f", solution[joint]);
    //         }
    //     }
    // }

    // if (success) {
    //     ROS_INFO("(Index %d) IK calculation was successful.", pose+1);
    //     joint_trajectory.joints.push_back(solution);
    // } else {
    //     ROS_WARN("IK calculation failed for index %d. Error code: %d", pose+1, error_code.val);

    //     if (debug) {
    //         ROS_WARN("(debug) Skipping trajectory point.");
    //         solution = previous_solution;
    //     } else {
    //         ROS_WARN("Exiting function.");
    //         return false;
    //     }
    // }


}

/* ============================================================
   -------------------- FINISHED FUNCTIONS --------------------
   ============================================================ */

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

    ABB AXIS CONFIGURATION CONVENTION: [axis_1_config, axis_4_config, axis_6_config, cfx]

    ABB YuMi AXIS NAMING CONVENTION (from YuMi base to end effector): [1, 2, 7, 3, 4, 5, 6]

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
            > This value is not used and is always set to (0)

    EXTERNAL AXIS POSITION CONVENTION: [axis_7_position]
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
        ROS_INFO("_____ (debug) Axis configuration for provided joint positions _____");
        ROS_INFO("Configuration: [%d, %d, %d, %d] | External Axis Position: [%0.5f]", 
            pose_config.confdata[0], pose_config.confdata[1], pose_config.confdata[2], pose_config.confdata[3], 
            pose_config.external_axis_position);
    }

    return pose_config;
}

/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */

RAPIDModuleData getYuMiLeadThroughData(std::string file_name, planningInterface::MoveGroup& group, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-07

    PURPOSE: The purpose of this funciton is to open the provided RAPID module, store all the robtargets at the beginning of the 
             file, get the order to executing the robtargets from the main function, then reconstructing the final trajectory. A
             simple RAPID module and program flow explaination is below along with a good convention to follow when creating these
             modules.

    EXPLAINATION: The RAPID module file name should be provided without extentions (.txt, etc.) or file path (/home/user). This 
                  function will search for that file within the yummi_scripts package in a folder called "demo". Make sure the 
                  RAPID module is located within that folder. The move group for this file should also be provided in case that
                  the first command within the RAPID module is OpenHand or CloseHand. In this case, the current pose of the robot
                  for the provided group will be used as the first pose for the trajectory. If debug is set to true, the user will
                  be notified about the current status of the function, the robtargets parsed from the file, the order of 
                  robtargets from the main function, and the final reconstructed trajectory.


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

    std::string input_file = yumi_scripts_directory + "paths/" + file_name + ".mod";

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
        > eax1 represents the position of the first external axis (axis 7)
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

trajectoryPoses combineModules(RAPIDModuleData& module_1, RAPIDModuleData& module_2, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-25 

    PURPOSE: The purpose of this function is to combine two RAPID module data structure into a pose trajectory. This function
             will only work properly or will produce and error if the two modules were meant to work together. These modules
             will be combined in a way that all MoveSync motions will be lined up. If any of the arms is moving independently
             (including closing and opening the grippers), the other arm will wait at it's current position until the next 
             MoveSync task is reached. This function assumes that grippers are on each arm.

    EXPLAINATION: This function requires two different modules to be supplied where one is meant for the left arm and the 
                  other is meant for the right arm. The designated move groups for each module should have already been
                  stored into the RAPID module structure from a previous function. If debug is set to true, the provided 
                  module names will be outputted to the user, and the pose names for each arm will be displayed as the final 
                  pose trajectory is being constructed by attempting to synconize motions from each module.


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

/* ------------------------------------------------------------ */
/* -------------------- UNTESTED FUNCTIONS -------------------- */
/* ------------------------------------------------------------ */

trajectoryPoses convertModuleToPoseTrajectory(RAPIDModuleData& module, bool debug) { // NEEDS TO BE TESTED
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

/* ============================================================
   -------------------- FINISHED FUNCTIONS --------------------
   ============================================================ */


trajectoryJoints convertPoseTrajectoryToJointTrajectory(planningInterface::MoveGroup& group, trajectoryPoses& pose_trajectory, kinematics::KinematicsBasePtr& ik_left, kinematics::KinematicsBasePtr& ik_right, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-29

    If the correct group and pose trajectory were provided to this function to indicate that the data is meant for 
           the left arm, then proceed to converting from the pose trajectory to joint trajectory for the left arm. If debug
           mode was set to false, then if any of the pose to joints conversion fails, the conversion function will stop
           executing and return a success of false. This function will then notify the user that the conversion failed and
           will return an empty joint trajectory. If debug is set to true, then the point that failed conversion will be 
           skipped and the conversion will continue with the next point. If the conversion was successful for all points,
           unless debug was set to true as explained above, then the correct gripper positions will be added to the joint
           trajectory and the structure for the joint trajectory will be returned.

    ASSUMPTIONS: If current joint values vector contains 7 values then there is not a gripper is attached, but if it 
                 contains 8 values then a gripper is attached. Also assumes that there are at least 7 joints for the 
                 desired group.
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
            if (debug) { ROS_INFO("Converting pose trajectory to joint trajectory for the left arm."); }

            std::vector<double> joint_values_left = group.getCurrentJointValues();

            if (debug) {
                ROS_INFO("_____ (debug) Left Arm Initial Joint Values _____");
                for (int joint = 0; joint < joint_values_left.size(); joint++) {
                    ROS_INFO("Joint value: %.5f", joint_values_left[joint]);
                }
            }

            success_left = convertPoseConfigToJointTrajectory(joint_trajectory_left, ik_left, pose_trajectory.pose_configs_left, joint_values_left, debug);
            return joint_trajectory_left;

            if (success_right) {
                if (pose_trajectory.gripper_attached_left) {
                    for (int point = 0; point < joint_trajectory_left.total_points; point++) {
                    /* Override gripper positions producesd from the IK with the gripper data from the pose trajectory structure */
                        joint_trajectory_left.joints[point][7] = pose_trajectory.pose_configs_left[point].gripper_position;
                    }
                }
                joint_trajectory_left.group_name = joint_trajectory_left.intended_group = "left_arm";

                return joint_trajectory_right;
            } else {
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
            if (debug) { ROS_INFO("Converting pose trajectory to joint trajectory for the right arm."); }

            std::vector<double> joint_values_right = group.getCurrentJointValues();

            if (debug) {
                ROS_INFO("_____ (debug) Right Arm Initial Joint Values _____");
                for (int joint = 0; joint < joint_values_right.size(); joint++) {
                    ROS_INFO("Joint value: %.5f", joint_values_right[joint]);
                }
            }

            success_right = convertPoseConfigToJointTrajectory(joint_trajectory_right, ik_right, pose_trajectory.pose_configs_right, joint_values_right, debug);

            if (success_right) {
                if (pose_trajectory.gripper_attached_right) {
                    for (int point = 0; point < joint_trajectory_right.total_points; point++) {
                    /* Override gripper positions producesd from the IK with the gripper data from the pose trajectory structure */
                        joint_trajectory_right.joints[point][7] = pose_trajectory.pose_configs_right[point].gripper_position;
                    }
                }
                joint_trajectory_right.group_name = joint_trajectory_right.intended_group = "right_arm";

                return joint_trajectory_right;
            } else {
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
            if (debug) { ROS_INFO("Converting pose trajectory to joint trajectory for both arms."); }

            std::vector<std::string> joint_names = group.getActiveJoints();
            std::vector<double>     joint_values = group.getCurrentJointValues();

            int right_arm_first_joint_index;
            for (int joint = 7; joint < joint_names.size(); joint++) {
                if (joint_names[joint].compare("yumi_joint_1_r") == 0) {
                    right_arm_first_joint_index = joint;
                    break;
                }
            }

            std::vector<double> joint_values_left(&joint_values[0], &joint_values[right_arm_first_joint_index-1]);
            std::vector<double> joint_values_right(&joint_values[right_arm_first_joint_index], &joint_values[joint_values.size()-1]);

            if (debug) {
                ROS_INFO("_____ (debug) Both Arms Initial Joint Values _____");
                for (int joint = 0; joint < joint_values.size(); joint++) {
                    ROS_INFO("Joint value: %.5f", joint_values[joint]);
                }
                ROS_INFO("_____ (debug) Left Arm Initial Joint Values _____");
                for (int joint = 0; joint < joint_values_left.size(); joint++) {
                    ROS_INFO("Joint value: %.5f", joint_values_left[joint]);
                }
                ROS_INFO("_____ (debug) Right Arm Initial Joint Values _____");
                for (int joint = 0; joint < joint_values_right.size(); joint++) {
                    ROS_INFO("Joint value: %.5f", joint_values_right[joint]);
                }
            }

            if (debug) {
                ROS_INFO("....................");
                ROS_INFO("Solving IK for left arm");
            }
            success_left  = convertPoseConfigToJointTrajectory(joint_trajectory_left, ik_left, pose_trajectory.pose_configs_left, joint_values_left, debug);
            if (debug) {
                ROS_INFO("....................");
                ROS_INFO("Solving IK for right arm");
            }
            success_right = convertPoseConfigToJointTrajectory(joint_trajectory_right, ik_right, pose_trajectory.pose_configs_right, joint_values_right, debug);

            if (joint_trajectory_left.total_points != joint_trajectory_right.total_points) {
                ROS_ERROR("From: convertPoseTrajectoryToJointTrajectory(group, pose_trajectory, ik_left, ik_right, debug)");
                ROS_ERROR("The total trajectory points from the provided pose trajectories for the right and left arm do not match up.");
                ROS_WARN("Left pose trajectory size: %d | Right pose trajectory size: %d", joint_trajectory_left.total_points, joint_trajectory_right.total_points);
                ROS_INFO("....................");
                success_left = success_right = success = false;
            }

            if (((success_left) && (success_right)) && joint_trajectory_left.total_points > 0) {
                joint_trajectory.total_joints = joint_trajectory_left.total_joints + joint_trajectory_right.total_joints;
                joint_trajectory.total_points = joint_trajectory_left.total_points;

                for (int point = 0; point < joint_trajectory_right.total_points; point++) {
                /* Combine joint trajectories and override gripper positions producesd from the IK with the gripper data from the pose trajectory structure */
                    for (int joint = 0; joint < joint_trajectory.total_joints; joint++) {
                        if (joint < joint_trajectory_left.total_joints) { joint_values[joint] = joint_trajectory_left.joints[point][joint]; }
                        else { joint_values[joint] = joint_trajectory_right.joints[point][joint-joint_trajectory_left.total_joints]; }
                    }
                    joint_trajectory.joints.push_back(joint_values);

                    if (pose_trajectory.gripper_attached_left) {
                        joint_trajectory.joints[point][7] = pose_trajectory.pose_configs_left[point].gripper_position;
                        /* Gripper position for the left arm is index 7 since: 7 (joints left arm) + 1 (gripper left_arm) - 1 (index starts from 0) = 7 */
                        if (pose_trajectory.gripper_attached_right) {
                            joint_trajectory.joints[point][15] = pose_trajectory.pose_configs_right[point].gripper_position;
                            /* Gripper position for the right arm is index 15 since the left arm has a gripper meaning: 7 (joints left arm) + 1 (gripper left_arm) + 7 (joints right_arm) + 1 (gripper right_arm) - 1 (index starts from 0) = 15 */
                        }
                    } else if (pose_trajectory.gripper_attached_right) {
                    /* If there is no gripper attached to the left arm but there is a gripper attached to the right arm */
                        joint_trajectory.joints[point][14] = pose_trajectory.pose_configs_right[point].gripper_position;
                        /* Gripper position for the right arm is index 14 since the left arm does not have a gripper meaning: 7 (joints left arm) + 7 (joints right_arm) + 1 (gripper right_arm) - 1 (index starts from 0) = 14 */
                    }
                }
                joint_trajectory.group_name = joint_trajectory.intended_group = "both_arms";

                return joint_trajectory;
            } else {
                if (success) {
                /* If this line has not already been displayed previously */
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

bool convertPoseConfigToJointTrajectory(trajectoryJoints& joint_trajectory, kinematics::KinematicsBasePtr& ik, std::vector<poseConfig>& pose_configs, std::vector<double> initial_seed, bool debug) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-07-28


    ASSUMPTIONS: If the initial_seed contains 7 values then there is not a gripper is attached, but if it contains 8 values 
                 then a gripper is attached. Also assumes that there are at least 7 joints for the desired group.
*/
    ROS_INFO("....................");

    // INITIALIZE VARIABLES
    const double PI = 3.14159;
    const std::vector<int> confdata_joints = { 0, 4, 6 };

    double timeout = 1.0;
    std::vector<double> seed;
    std::vector<double> consistency; // tolerance from seed
    std::vector<double> solution;
    std::vector<double> previous_solution = initial_seed;
    moveit_msgs::MoveItErrorCodes error_code;

    bool gripper_attached = false;
    bool success;

    if (initial_seed.size() == 8) {
        gripper_attached = true;
    }

    // TRANSFER DATA TO JOINT TRAJECTORY STRUCTURE
    joint_trajectory.total_joints = initial_seed.size();

    // DETERMINE CONSISTANCY VARIABLE SIZE BASED ON GRIPPER ATTACHED
    if (gripper_attached) {
        consistency = {PI/4, PI, PI/32, PI, PI/4, PI, PI/4, PI};
    } else {
        consistency = {PI/4, PI, PI/32, PI, PI/4, PI, PI/4};
    }

    tic();

    // PERFORM IK
    for (int pose = 0; pose < pose_configs.size(); pose++) {

        // GET INITIAL SEED
        if (pose == 0) { seed = initial_seed; } 
        else { seed = solution; }

        // SEED AXIS CONFIGURATIONS
        for (int config = 0; config < confdata_joints.size(); config++) {
            seed[confdata_joints[config]] = (pose_configs[pose].confdata[config] * PI/2) + PI/4;
        }
        seed[2] = pose_configs[pose].external_axis_position;

        if (debug) {
            ROS_INFO("_____ (debug) IK Inputs for Index %d _____", pose+1);
            ROS_INFO(" *Note: for orientation, the w value is provided first*");
            ROS_INFO("Pose        | Position: [%.5f, %.5f, %.5f] | Orientation: [%.5f, %.5f, %.5f, %.5f]", 
                pose_configs[pose].pose.position.x, pose_configs[pose].pose.position.y, pose_configs[pose].pose.position.z,
                pose_configs[pose].pose.orientation.w, pose_configs[pose].pose.orientation.x, pose_configs[pose].pose.orientation.y, pose_configs[pose].pose.orientation.z);
            ROS_INFO("Config Data | Values: [%d, %d, %d, %d]", pose_configs[pose].confdata[0], pose_configs[pose].confdata[1], pose_configs[pose].confdata[2], pose_configs[pose].confdata[3]);
            if (gripper_attached) {
                ROS_INFO("Seed        | Joint values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6], seed[7]);
                ROS_INFO("Consistency | Values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", consistency[0], consistency[1], consistency[2], consistency[3], consistency[4], 
                    consistency[5], consistency[6], consistency[7]);
            } else {
                ROS_INFO("Seed        | Joint values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6]);
                ROS_INFO("Consistency | Values: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", consistency[0], consistency[1], consistency[2], consistency[3], 
                    consistency[4], consistency[5], consistency[6]);
            }
        }

        // COMPUTE IK
        success = ik->searchPositionIK(pose_configs[pose].pose, seed, timeout, consistency, solution, error_code);

        if (debug) {
            ROS_INFO("____ (debug) Joint Value Solution for Index %d _____", pose+1);
            if (error_code.val != 1) {
                ROS_INFO("Solution not found.");
            } else {
                for (int joint = 0; joint < solution.size(); joint++) {
                    ROS_INFO("Joint value: %.5f", solution[joint]);
                }
            }
        }

        if (success) {
            ROS_INFO("(Index %d) IK calculation was successful.", pose+1);
            joint_trajectory.joints.push_back(solution);
        } else {
            ROS_WARN("IK calculation failed for index %d. Error code: %d", pose+1, error_code.val);

            if (debug) {
                ROS_WARN("(debug) Skipping trajectory point.");
                solution = previous_solution;
            } else {
                ROS_WARN("Exiting function.");
                return false;
            }
        }

        previous_solution = solution;
    }

    joint_trajectory.total_points = joint_trajectory.joints.size();

    ROS_INFO("Processing time: %.5f",toc());
    return true;
}