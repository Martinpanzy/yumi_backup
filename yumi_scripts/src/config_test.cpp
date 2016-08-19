// INCLUDES
#include <string>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

// NAMESPACE DECLARATION
using namespace ros;

// GLOBAL VARIABLES
const std::string yumi_scripts_directory = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/";

// STRUCTURES
struct parameters {
    bool debug            = false;
    bool bounded_solution = true;
    bool save_as_rosbag   = false;
    bool dont_execute     = false;
    std::string module_name_left   = "";
    std::string module_name_right  = "";
    std::string end_effector_left  = "";
    std::string end_effector_right = "";
    std::string active_arms        = "";
};

// OPERATOR FUNCTIONS
void operator >> (const YAML::Node& node, parameters& params) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-16
*/
    params.debug            = node["debug"].as<bool>();
    params.bounded_solution = node["bounded_solution"].as<bool>();
    params.save_as_rosbag   = node["save_as_rosbag"].as<bool>();
    params.dont_execute     = node["dont_execute"].as<bool>();

    params.module_name_left   = node["module_name_left"].as<std::string>();
    params.module_name_right  = node["module_name_right"].as<std::string>();
    params.end_effector_left  = node["end_effector_left"].as<std::string>();
    params.end_effector_right = node["end_effector_right"].as<std::string>();
    params.active_arms        = node["active_arms"].as<std::string>();
}

// FUNCTION PROTOTYPS
void displayParameters(parameters);

// ====================================================================================
// ========== IF USING CONFIG FILE, NEED TO ADD YAML REPO'S TO SETUP WS FILE ==========
// ====================================================================================

// MAIN FUNCTION
int main(int argc, char **argv) {

    init(argc, argv, "config_test");
    NodeHandle node_handle;

    // INITIALIZE VARIABLES
    parameters params;

    // GET CONFIGURATION DATA FROM CONFIG FILE
    YAML::Node config = YAML::LoadFile(yumi_scripts_directory + "src/config.yaml");
    config >> params;

    // DIPLAY CONFIGURATION DATA RETRIEVED FROM CONFIG FILE
    displayParameters(params);

    return 0;
}

void displayParameters(parameters params) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-16
*/
    ROS_INFO("             Debug Mode: %s", params.debug?"true":"false");
    ROS_INFO("      Bounded Solutions: %s", params.bounded_solution?"true":"false");
    ROS_INFO("             Save Plans: %s", params.save_as_rosbag?"true":"false");
    ROS_INFO("     Dont Execute Plans: %s", params.dont_execute?"true":"false");
    ROS_INFO("       Left Module Name: %s", params.module_name_left.c_str());
    ROS_INFO("      Right Module Name: %s", params.module_name_right.c_str());
    ROS_INFO(" Left End Effector Name: %s", params.end_effector_left.c_str());
    ROS_INFO("Right End Effector Name: %s", params.end_effector_right.c_str());
    ROS_INFO("          Active Arm(s): %s", params.active_arms.c_str());
}