// INCLUDES
#include <fstream>
#include <cmath>
#include <ctime>
#include <math.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Pose.h>
#include <leap_motion/leapros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/JointTrajectory.h>

// NAMESPACE DECLARATION
namespace planningInterface = moveit::planning_interface;

// FUNCTION PROTOTYPES
/* Constructor Functions */
bool createIKSolvers(kinematics::KinematicsBasePtr&, kinematics::KinematicsBasePtr&, std::string, std::string, std::string);
geometry_msgs::Point constructPoint(double, double, double);
geometry_msgs::Vector3 constructVector(double, double, double);
geometry_msgs::Pose constructPose(geometry_msgs::Point, geometry_msgs::Quaternion);
bool getConfigParameters();

/* Data Check/Display Functions */
bool isSamePosition(geometry_msgs::Point, geometry_msgs::Point);
void displayPalmValues(const geometry_msgs::Point&, const geometry_msgs::Quaternion&);
void displayGripperPosition(const bool, const geometry_msgs::Point&, const geometry_msgs::Point&);
void displayDeltas(const geometry_msgs::Point&, const geometry_msgs::Vector3&, const bool&, const bool&);

/* Subscriber Functions */
void leapCallback(const leap_motion::leapros::ConstPtr&);

/* Conversion Functions */
geometry_msgs::Point getRelativePosition(const geometry_msgs::Point&);
geometry_msgs::Point convertLeapToYuMiReferenceFrame(const geometry_msgs::Point&);
geometry_msgs::Vector3 convertLeapToYuMiReferenceFrame(const geometry_msgs::Vector3&);
geometry_msgs::Vector3 convertQuaternionToYPR(const geometry_msgs::Quaternion&);

/* Movement Functions */
bool getPositionDelta(geometry_msgs::Point&, const geometry_msgs::Point&);
bool getOrientationDelta(geometry_msgs::Vector3&, const geometry_msgs::Vector3&, const geometry_msgs::Vector3&);
bool getGripperPosition(const geometry_msgs::Point&, const geometry_msgs::Point&);
void updateRelativePosition(geometry_msgs::Point&, const geometry_msgs::Point&);
void updateRelativeOrientation(geometry_msgs::Quaternion&, const geometry_msgs::Vector3&);
bool addTrajectoryPoint(trajectory_msgs::JointTrajectory&, planningInterface::MoveGroup&, const geometry_msgs::Pose&, const bool&, kinematics::KinematicsBasePtr&);

/* MoveIt! Functions */
void moveArm(trajectory_msgs::JointTrajectory&, ros::Publisher&);
void moveArmDemo(planningInterface::MoveGroup&, const geometry_msgs::Pose& pose, const bool&, kinematics::KinematicsBasePtr&, sensor_msgs::JointState&, ros::Publisher&);
bool computeIK(std::vector<double>&, const geometry_msgs::Pose&, kinematics::KinematicsBasePtr&, std::vector<double>);

// GLOBAL VARIABLES
bool leap_msg_lock = false;
leap_motion::leapros global_leap_msg;

const double PI = 3.14159;
const geometry_msgs::Point ABSOLUTE_CENTER_POINT = constructPoint(0.0, 130.0, 0.0); // in leap reference frame
boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader;

/* Configuration Parameters */
double GRIPPER_OPEN_POSITION, GRIPPER_CLOSED_POSITION, CLOSED_GRIPPER_DISTANCE;
double MAX_DELTA_POSITION, MAX_DELTA_ORIENTATION;
double DEAD_ZONE_POSITION, DEAD_ZONE_ORIENTATION;
double SCALE_FACTOR_POSITION, SCALE_FACTOR_ORIENTATION = (1.0/90.0);
double COMMUNICATION_DELAY;
int TRAJECTORY_LENGTH;
bool DEMO_MODE, SHOW_DATA;

// MAIN FUNCTION
int main(int argc, char **argv) {

    // SETUP ROS VARIABLES
    ros::init(argc, argv, "leap_interface");
    ros::NodeHandle node_handle_leap("leapmotion");
    ros::NodeHandle node_handle("");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber leap_sub = node_handle_leap.subscribe("data", 1, leapCallback);
    ros::Publisher joint_state_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher joint_trajectory_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("left_arm/joint_path_command", 1);

    // INITIALIZE MOVEIT VARIABLES
    sensor_msgs::JointState joint_state;
    planningInterface::MoveGroup left_arm("left_arm");
    planningInterface::MoveGroup left_arm_ik("left_arm_ik");
    planningInterface::MoveGroup right_arm("right_arm");
    planningInterface::MoveGroup right_arm_ik("right_arm_ik");
    planningInterface::MoveGroup both_arms("both_arms");
    kinematics::KinematicsBasePtr ik_left, ik_right;

    joint_state.name = both_arms.getActiveJoints();

    left_arm.startStateMonitor();
    left_arm.getCurrentJointValues();
    left_arm_ik.startStateMonitor();
    left_arm_ik.getCurrentJointValues();
    right_arm.startStateMonitor();
    right_arm.getCurrentJointValues();
    right_arm_ik.startStateMonitor();
    right_arm_ik.getCurrentJointValues();
    both_arms.startStateMonitor();
    both_arms.getCurrentJointValues();

    std::string pose_reference_frame = "yumi_body";
    left_arm.setPoseReferenceFrame(pose_reference_frame);
    left_arm_ik.setPoseReferenceFrame(pose_reference_frame);
    right_arm.setPoseReferenceFrame(pose_reference_frame);
    right_arm_ik.setPoseReferenceFrame(pose_reference_frame);
    both_arms.setPoseReferenceFrame(pose_reference_frame);

    std::string end_effector_left  = "yumi_link_7_l";
    std::string end_effector_right = "yumi_link_7_r";
    left_arm_ik.setEndEffectorLink(end_effector_left);
    right_arm_ik.setEndEffectorLink(end_effector_right);

    bool success = createIKSolvers(ik_left, ik_right, end_effector_left, end_effector_right, pose_reference_frame);
    if (!success) {
        ROS_ERROR("Stopping execution due to error in creating IK solvers.");
        return 1;
    }

    // LOAD CONFIGURATION PARAMETERS
    if (!getConfigParameters()) {
        ROS_ERROR("None or not all of the configuration parameters have been loaded to parameter server");
        ROS_WARN("Please load the configuration parameters to the parameter server");
        ROS_WARN("Config param file location: yumi_scripts (package) /src/config.yaml (dirctory)");

        return 1;
    }

    if (DEMO_MODE) {
        // SET START POSITION AS CALC 
        std::vector<double> init_joint_values_left  = {-1.1345, -2.4435, 1.2217, 0.8727, 2.0071, 0.8727, 0.0349, 0.024}; // joint values for calc position
        std::vector<double> init_joint_values_right = {0.0, -2.2689, -2.3562, 0.5236, 0.0, 0.6981, 0.0, 0.0}; // joint values for calc position

        joint_state.position = init_joint_values_left;
        joint_state.position.insert(joint_state.position.end(), init_joint_values_right.begin(), init_joint_values_right.end());
        joint_state_pub.publish(joint_state);
    }
    

    // INITIALIZE VARIABLES
    geometry_msgs::PoseStamped pose_left = left_arm_ik.getCurrentPose();
    geometry_msgs::Point start_location = pose_left.pose.position;
    geometry_msgs::Quaternion start_orientation = pose_left.pose.orientation;

    start_location.z -= 0.11;

    geometry_msgs::Vector3 palmdir, palmnor;
    geometry_msgs::Point palmpos, prev_palmpos, thumb_tip, index_tip;
    geometry_msgs::Point palm_position = start_location;
    geometry_msgs::Quaternion palm_orientation = start_orientation;

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = left_arm.getActiveJoints();

    bool gripper_position, prev_gripper_pos;
    bool first_iteration = true;
    clock_t previous_pub_time = clock();

    gripper_position = prev_gripper_pos = 1; // start with open gripper

    int system_return = std::system("clear"); // clear the terminal window
    while (ros::ok()) {
    /*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
        DATE CREATED: 2016-08-30
    */
        if (leap_msg_lock) {
            // GET DATA
            palmpos = global_leap_msg.palmpos;
            palmdir = global_leap_msg.direction;
            palmnor = global_leap_msg.normal;
            thumb_tip = global_leap_msg.thumb_tip;
            index_tip = global_leap_msg.index_tip;

            // CHECK IF THE CURRENT POSITION IS THE SAME AS THE PREVIOUS ONE INDICATING THAT LEAP DOES NOT SEE A HAND
            if (isSamePosition(palmpos, prev_palmpos)) {
                leap_msg_lock = false;
                continue;
            } else if (first_iteration) {
                first_iteration = false;
                prev_palmpos     = palmpos;
                leap_msg_lock = false;
                continue;
            } else {
                prev_palmpos = palmpos;
            }

            // GET PALM POSITION RELATIVE TO THE ABSOLUTE CENTER POINT
            palmpos = getRelativePosition(palmpos);

            // CONVERT VALUES FROM LEAP TO YUMI FRAME
            palmpos = convertLeapToYuMiReferenceFrame(palmpos);
            palmdir = convertLeapToYuMiReferenceFrame(palmdir);
            palmnor = convertLeapToYuMiReferenceFrame(palmnor);

            // GET DELTA AND UPDATE RELATIVE POSITION AND ORIENTATION
            geometry_msgs::Point delta_position = constructPoint(0.0, 0.0, 0.0); // give position delta a default value incase current position is within the dead zone
            bool dead_zone_pos = getPositionDelta(delta_position, palmpos);
            geometry_msgs::Vector3 delta_orienation = constructVector(0.0, 0.0, 0.0); // give orientation delta a default value incase current orientation is within the dead zone
            bool dead_zone_ori = getOrientationDelta(delta_orienation, palmdir, palmnor);

            updateRelativePosition(palm_position, delta_position);
            updateRelativeOrientation(palm_orientation, delta_orienation);

            // GET GRIPPER POSITION
            bool new_gripper_position = getGripperPosition(thumb_tip, index_tip);
            bool gripper_change = ((prev_gripper_pos == 1) && (new_gripper_position == 0));
            if (gripper_change) {
                gripper_position = !gripper_position;
            }

            geometry_msgs::Pose pose = constructPose(palm_position, palm_orientation);
            if ((!dead_zone_pos) || (!dead_zone_ori) || (gripper_change)) {
                addTrajectoryPoint(trajectory, left_arm, pose, gripper_position, ik_left);
            }
            prev_gripper_pos = new_gripper_position;

            // CALCULATE JOINT POSITIONS AND SHOW ARM AT POSITION AND ORIENTATION IF NOT WITHIN THE DEAD ZONE
            if (DEMO_MODE) {
                moveArmDemo(left_arm, pose, gripper_position, ik_left, joint_state, joint_state_pub);
            } else {
                double time_lapsed = (clock() - previous_pub_time) / (double) CLOCKS_PER_SEC;
                if (time_lapsed > COMMUNICATION_DELAY) {
                    if (trajectory.points.size() > 0) {
                        moveArm(trajectory, joint_trajectory_pub);
                        trajectory.points.clear();
                        previous_pub_time = clock();
                    }
                }
            }
            
            if (SHOW_DATA) {
                // DISPLAY VALUES
                displayPalmValues(palm_position, palm_orientation);
                displayGripperPosition(gripper_position, thumb_tip, index_tip);
                displayDeltas(delta_position, delta_orienation, dead_zone_pos, dead_zone_ori);
            }

            // RELEASE DATA LOCK
            leap_msg_lock = false;
        }

        ros::spinOnce();
    }

    return 0;
}


/* ------------------------------------------------------------ */
/* ------------------ CONSTRUCTOR FUNCTIONS ------------------- */
/* ------------------------------------------------------------ */
bool createIKSolvers(kinematics::KinematicsBasePtr& ik_left, kinematics::KinematicsBasePtr& ik_right, std::string end_effector_left, std::string end_effector_right, std::string pose_reference_frame) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    bool success_ik_left, success_ik_right, success = true;
    double search_discretization = 0.001;

    // LOAD KINEMATICS PLUGIN
    kinematics_loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
    std::string plugin_name = "kdl_kinematics_plugin/KDLKinematicsPlugin";
    try {
        ik_left  = kinematics_loader->createInstance(plugin_name);
        ik_right = kinematics_loader->createInstance(plugin_name);
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("The kinematics plugin failed to load. Error: %s", ex.what());
        success = false;
        return success;
    }

    // INITIALIZE IK SOLVERS
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
        success = false;
    }

    return success;
}

geometry_msgs::Point constructPoint(double x, double y, double z) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Point point;

    // CONSTRUCT OBJECt
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

geometry_msgs::Vector3 constructVector(double x, double y, double z) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Vector3 vector;

    // CONSTRUCT OBJECt
    vector.x = x;
    vector.y = y;
    vector.z = z;

    return vector;
}

geometry_msgs::Pose constructPose(geometry_msgs::Point position, geometry_msgs::Quaternion orientation) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Pose pose;

    // CONSTRUCT POSE
    pose.position = position;
    pose.orientation = orientation;

    return pose;
}

bool getConfigParameters() {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-09-06

    PURPOSE: The purpose of this function is to get al the configuration parameters that are located in the ROS parameter server
             that are used in this script.

    INSTRUCTIONS: Provide all the variables that will be used to store the configruations parameters from the ROS parameter server.
*/
    // INITIALIZE VARIABLES
    bool parameters_exist;

    // GET CONFIGURATION PARAMETERS FROM PARAMETER SERVER
    parameters_exist = ((ros::param::get("/yumi/yumi_leap/demo_mode", DEMO_MODE)) && 
                        (ros::param::get("/yumi/yumi_leap/show_data", SHOW_DATA)) &&
                        (ros::param::get("/yumi/yumi_leap/gripper_open_position", GRIPPER_OPEN_POSITION)) && 
                        (ros::param::get("/yumi/yumi_leap/gripper_closed_position", GRIPPER_CLOSED_POSITION)) && 
                        (ros::param::get("/yumi/yumi_leap/gripper_distance_toggle", CLOSED_GRIPPER_DISTANCE)) && 
                        (ros::param::get("/yumi/yumi_leap/dead_zone_position", DEAD_ZONE_POSITION)) && 
                        (ros::param::get("/yumi/yumi_leap/dead_zone_orientation", DEAD_ZONE_ORIENTATION)) && 
                        (ros::param::get("/yumi/yumi_leap/max_zone_position", MAX_DELTA_POSITION)) && 
                        (ros::param::get("/yumi/yumi_leap/max_zone_orientation", MAX_DELTA_ORIENTATION)) && 
                        (ros::param::get("/yumi/yumi_leap/scale_factor_position", SCALE_FACTOR_POSITION)) && 
                        (ros::param::get("/yumi/yumi_leap/scale_factor_orientation", SCALE_FACTOR_ORIENTATION)) && 
                        (ros::param::get("/yumi/yumi_leap/min_communication_delay", COMMUNICATION_DELAY)) && 
                        (ros::param::get("/yumi/yumi_leap/max_trajectory_length", TRAJECTORY_LENGTH)));

    return parameters_exist;
}

/* ------------------------------------------------------------ */
/* --------------- DATA CHECK/DISPLAY FUNCTIONS --------------- */
/* ------------------------------------------------------------ */
bool isSamePosition(geometry_msgs::Point position, geometry_msgs::Point prev_position) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-30
*/
    if ((position.x == prev_position.x) && (position.y == prev_position.y) && (position.z == prev_position.z)) {
        return true;
    } else {
        return false;
    }
}

void displayPalmValues(const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Vector3 orientation_ypr = convertQuaternionToYPR(orientation);

    ROS_INFO(">--------------------");
    ROS_INFO("YuMi Pose: %.5f %.5f %.5f", position.x, position.y, position.z);
    ROS_INFO(" YuMi RPY: %.5f %.5f %.5f", orientation_ypr.x, orientation_ypr.y, orientation_ypr.z);
}

void displayGripperPosition(const bool gripper_position, const geometry_msgs::Point& thumb_tip, const geometry_msgs::Point& index_tip) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-09-01
*/
    double tip_distance = sqrt( pow((index_tip.x - thumb_tip.x), 2.0) + pow((index_tip.y - thumb_tip.y), 2.0) + pow((index_tip.z - thumb_tip.z), 2.0) );

    ROS_INFO(">--------------------");
    ROS_INFO("    Gripper Position: %s", gripper_position?"Open":"Closed");
    ROS_INFO("Thumb-Index Distance: %.5f", tip_distance);

}

void displayDeltas(const geometry_msgs::Point& delta_position, const geometry_msgs::Vector3& delta_orientation, const bool& dead_zone_pos, const bool& dead_zone_ori) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    ROS_INFO(">--------------------");
    ROS_INFO("   Within Dead Zone Position: %s", dead_zone_pos?"true":"false");
    ROS_INFO("Within Dead Zone Orientation: %s", dead_zone_ori?"true":"false");
    ROS_INFO("     Position Delta (meters): %.5f %.5f %.5f", delta_position.x, delta_position.y, delta_position.z);
    ROS_INFO(" Orientation Delta (degrees): %.5f %.5f %.5f", delta_orientation.x * (180.0/PI), delta_orientation.y * (180.0/PI), delta_orientation.z * (180.0/PI));
}

/* ------------------------------------------------------------ */
/* ------------------- SUBSCRIBER FUNCTIONS ------------------- */
/* ------------------------------------------------------------ */
void leapCallback(const leap_motion::leapros::ConstPtr& leap_msg) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-30
*/
    if (!leap_msg_lock) {
        leap_msg_lock = true;
        global_leap_msg = *leap_msg;
    }
}

/* ------------------------------------------------------------ */
/* ------------------- CONVERSION FUNCTIONS ------------------- */
/* ------------------------------------------------------------ */
geometry_msgs::Point getRelativePosition(const geometry_msgs::Point& position) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Point relative_position;

    // GET POSITION RELATIVE TO ABSOLUTE CENTER POINT IN LEAP FRAME
    relative_position.x = position.x - ABSOLUTE_CENTER_POINT.x;
    relative_position.y = position.y - ABSOLUTE_CENTER_POINT.y;
    relative_position.z = position.z - ABSOLUTE_CENTER_POINT.z;

    return relative_position;
}

geometry_msgs::Point convertLeapToYuMiReferenceFrame(const geometry_msgs::Point& position) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Point new_position;

    // PERFORM TRANSFORMATION FROM LEAP TO YUMI REFERENCE FRAME
    new_position.x = position.z;
    new_position.y = position.x;
    new_position.z = position.y;

    return new_position;
}

geometry_msgs::Vector3 convertLeapToYuMiReferenceFrame(const geometry_msgs::Vector3& vector) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Vector3 new_vector;

    // PERFORM TRANSFORMATION FROM LEAP TO YUMI REFERENCE FRAME
    new_vector.x = vector.z;
    new_vector.y = vector.x;
    new_vector.z = vector.y;

    return new_vector;
}

geometry_msgs::Vector3 convertQuaternionToYPR(const geometry_msgs::Quaternion& orientation) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    tf::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 euler_angle_matrix(quaternion);
    geometry_msgs::Vector3 ypr;

    // GET ROLL, PITCH, AND YAW
    euler_angle_matrix.getRPY(ypr.x, ypr.y, ypr.z);

    return ypr;
}

geometry_msgs::Quaternion convertYPRToQuaternion(const geometry_msgs::Vector3& ypr) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(ypr.x, ypr.y, ypr.z);

    return quaternion;
}

/* ------------------------------------------------------------ */
/* -------------------- MOVEMENT FUNCTIONS -------------------- */
/* ------------------------------------------------------------ */
bool getPositionDelta(geometry_msgs::Point& delta, const geometry_msgs::Point& position) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INTIIALIZE VARIABLES
    double distanceFromCenter = sqrt( pow(position.x, 2.0) + pow(position.y, 2.0) + pow(position.z, 2.0) );
    bool dead_zone = false;

    // CALCULATE THE DELTA
    if (distanceFromCenter < DEAD_ZONE_POSITION) {
    /* If the palm position is still within the dead zone */
        dead_zone = true;
    } else {
    /* If the palm position is not within the dead zone */
        // CALCULATE RELEVENT ANGLES
        double theta = atan2(position.y, position.x);
        double phi = acos( (position.z / distanceFromCenter) );

        // CALCULATE THE X, Y, AND Z VALUES THAT ARE OUTSIDE OF THE DEAD ZONE
        delta.x = position.x - (DEAD_ZONE_POSITION * sin(phi) * cos(theta));
        delta.y = position.y - (DEAD_ZONE_POSITION * sin(phi) * sin(theta));
        delta.z = position.z - (DEAD_ZONE_POSITION * cos(phi));

        // SET UPPER AND LOWER BOUNDS FOR DELTA
        delta.x = std::min(delta.x, MAX_DELTA_POSITION);
        delta.y = std::min(delta.y, MAX_DELTA_POSITION);
        delta.z = std::min(delta.z, MAX_DELTA_POSITION);

        delta.x = std::max(delta.x, -MAX_DELTA_POSITION);
        delta.y = std::max(delta.y, -MAX_DELTA_POSITION);
        delta.z = std::max(delta.z, -MAX_DELTA_POSITION);

        // SCALE DELTA
        delta.x = delta.x * SCALE_FACTOR_POSITION;
        delta.y = delta.y * SCALE_FACTOR_POSITION;
        delta.z = delta.z * SCALE_FACTOR_POSITION;
    }

    return dead_zone;
}

bool getOrientationDelta(geometry_msgs::Vector3& delta, const geometry_msgs::Vector3& direction, const geometry_msgs::Vector3& normal) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INTIIALIZE VARIABLES
    double yaw   = -atan2(direction.y, -direction.x);
    // double pitch = asin(normal.x);
    // double roll  = -asin(normal.y);
    double pitch = 0.0; // dont allow pitch
    double roll  = 0.0; // dont allow roll
    bool dead_zone = true;

    if (std::abs(roll) > DEAD_ZONE_ORIENTATION) {
        dead_zone = false;
        delta.x = roll;

        // CALCULATE DEGREES OUTSIDE OF DEAD ZONE
        int sign = (delta.x > 0) ? 1 : -1;
        delta.x -= (DEAD_ZONE_ORIENTATION * sign);

        // SET UPPER AND LOWER BOUNDS FOR DELTA
        delta.x = std::min(delta.x, MAX_DELTA_ORIENTATION);
        delta.x = std::max(delta.x, -MAX_DELTA_ORIENTATION);

        // SCALE DELTA
        delta.x = delta.x * SCALE_FACTOR_ORIENTATION;
    }

    if (std::abs(pitch) > DEAD_ZONE_ORIENTATION) {
        dead_zone = false;
        delta.y = pitch;

        // CALCULATE DEGREES OUTSIDE OF DEAD ZONE
        int sign = (delta.y > 0) ? 1 : -1;
        delta.y -= (DEAD_ZONE_ORIENTATION * sign);
        
        // SET UPPER AND LOWER BOUNDS FOR DELTA
        delta.y = std::min(delta.y, MAX_DELTA_ORIENTATION);
        delta.y = std::max(delta.y, -MAX_DELTA_ORIENTATION);

        // SCALE DELTA
        delta.y = delta.y * SCALE_FACTOR_ORIENTATION;
    }

    if (std::abs(yaw) > DEAD_ZONE_ORIENTATION) {
        dead_zone = false;
        delta.z = yaw;

        // CALCULATE DEGREES OUTSIDE OF DEAD ZONE
        int sign = (delta.z > 0) ? 1 : -1;
        delta.z -= (DEAD_ZONE_ORIENTATION * sign);
        
        // SET UPPER AND LOWER BOUNDS FOR DELTA
        delta.z = std::min(delta.z, MAX_DELTA_ORIENTATION);
        delta.z = std::max(delta.z, -MAX_DELTA_ORIENTATION);

        // SCALE DELTA
        delta.z = delta.z * SCALE_FACTOR_ORIENTATION;
    }

    return dead_zone;
}

bool getGripperPosition(const geometry_msgs::Point& thumb_tip, const geometry_msgs::Point& index_tip) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    double tip_distance = sqrt( pow((index_tip.x - thumb_tip.x), 2.0) + pow((index_tip.y - thumb_tip.y), 2.0) + pow((index_tip.z - thumb_tip.z), 2.0) );
    bool open_gripper = true;

    if (tip_distance < CLOSED_GRIPPER_DISTANCE) {
        open_gripper = false;
    }

    return open_gripper;
}

void updateRelativePosition(geometry_msgs::Point& position, const geometry_msgs::Point& delta) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // UPDATE RELATIVE POSITION
    position.x += delta.x;
    position.y += delta.y;
    position.z += delta.z;  
}

void updateRelativeOrientation(geometry_msgs::Quaternion& orientation, const geometry_msgs::Vector3& delta) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    geometry_msgs::Vector3 orientation_ypr = convertQuaternionToYPR(orientation);

    // UPDATE RELATIVE ORIENTATION
    orientation_ypr.x += delta.x;
    orientation_ypr.y += delta.y;
    orientation_ypr.z += delta.z;

    // CONVERT ROLL, PITCH, AND YAW BACK TO QUATERNION
    orientation = convertYPRToQuaternion(orientation_ypr);
}

bool addTrajectoryPoint(trajectory_msgs::JointTrajectory& trajectory, planningInterface::MoveGroup& group, const geometry_msgs::Pose& pose, const bool& gripper_position, kinematics::KinematicsBasePtr& ik) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    std::vector<double> solution;
    std::vector<double> initial_guess = group.getCurrentJointValues();
    trajectory_msgs::JointTrajectoryPoint point;

    // CALCULATE INVERSE KINEMATICSZ
    bool success = computeIK(solution, pose, ik, initial_guess);

    // ADJUST GRIPPER POSITION
    if (gripper_position) {
        solution.back() = GRIPPER_OPEN_POSITION;
    } else {
        solution.back() = GRIPPER_CLOSED_POSITION;
    }

    // ADD TRAJECTORY POINT
    if (success) {
        point.positions = solution;
        if (trajectory.points.size() < TRAJECTORY_LENGTH) {
            trajectory.points.push_back(point);
        } else {
            trajectory.points.back() = point; // overwrite the second trajectory point
        }
        trajectory.points.back().time_from_start = ros::Duration((COMMUNICATION_DELAY / TRAJECTORY_LENGTH) * (trajectory.points.size()-1));
    }

    return success;
}

/* ------------------------------------------------------------ */
/* --------------------- MOVEIT! FUNCTIONS -------------------- */
/* ------------------------------------------------------------ */
void moveArm(trajectory_msgs::JointTrajectory& trajectory, ros::Publisher& joint_trajectory_pub) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // PUBLISH TRAJECTORY
    joint_trajectory_pub.publish(trajectory);

    // RESET TRAJECTORY
    trajectory.points.clear();
}

void moveArmDemo(planningInterface::MoveGroup& group, const geometry_msgs::Pose& pose, const bool& gripper_position, kinematics::KinematicsBasePtr& ik, sensor_msgs::JointState& joint_state, ros::Publisher& joint_state_pub) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    std::vector<double> solution;
    std::vector<double> initial_guess = group.getCurrentJointValues();

    // CALCULATE INVERSE KINEMATICSZ
    bool success = computeIK(solution, pose, ik, initial_guess);

    // ADJUST GRIPPER POSITION
    if (gripper_position) {
        solution.back() = GRIPPER_OPEN_POSITION;
    } else {
        solution.back() = GRIPPER_CLOSED_POSITION;
    }

    // CONSTRUCT FINAL JOINT STATE
    if (success) {
        // CONSTRUCT JOINT STATE MESSAGE
        if (group.getName().compare("left_arm") == 0) {
            for (int joint = 0; joint < solution.size(); joint++) {
                joint_state.position[joint] = solution[joint];
            }
        } else {
            int first_right_joint = joint_state.position.size() - solution.size();
            for (int joint = first_right_joint; joint < joint_state.position.size(); joint++) {
                joint_state.position[joint] = solution[joint-first_right_joint];
            }
        }

        // MOVE ARM
        joint_state_pub.publish(joint_state);
    }
}

bool computeIK(std::vector<double>& solution, const geometry_msgs::Pose& pose, kinematics::KinematicsBasePtr& ik, std::vector<double> initial_guess) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-31
*/
    // INITIALIZE VARIABLES
    std::vector<double> consistency(7, PI/3.0); // 60 degree tolerance in solution for each direction
    std::vector<double> joint_values_seed;
    moveit_msgs::MoveItErrorCodes error_code;
    double timeout = 0.25;

    bool gripper_attached = (initial_guess.size() == 8);
    double gripper_position;

    // CHECK IF GRIPPER VALUE WAS PROVIDED IN INITIAL GUESS
    if (gripper_attached) {
        gripper_position = initial_guess.back();
        initial_guess.pop_back();
    }

    // CALCULATE IK
    bool success = ik->searchPositionIK(pose, initial_guess, timeout, (const std::vector<double>)consistency, solution, error_code);
    if (!success) {
        ROS_ERROR("IK Failed.");
    } else {
        if (gripper_attached) {
            solution.push_back(gripper_position);
        }
    }

    return success;
}


