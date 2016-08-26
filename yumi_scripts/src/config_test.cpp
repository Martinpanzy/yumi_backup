// INCLUDES
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <yumi_scripts/ModuleMsg.h>
#include <yumi_scripts/JointMsg.h>

// NAMESPACE DECLARATION
using namespace ros;

// GLOBAL VARIABLES
const std::string YUMI_SCRIPTS_DIRECTORY = "/home/yumi/yumi_ws/src/yumi/yumi_scripts/";

// STRUCTURES
struct Parameters {
    bool debug_node   = false;
    bool load_yumi    = true;
    bool load_rviz    = false;
    bool two_grippers = false;

    bool debug_interface  = false;
    bool bounded_solution = true;
    std::string module_name_left   = "";
    std::string module_name_right  = "";
    std::string end_effector_left  = "";
    std::string end_effector_right = "";
};

// OPERATOR FUNCTIONS
void operator >> (const YAML::Node& node, Parameters& params) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-16
*/
    params.debug_node   = node["yumi_node"]["debug"].as<bool>();
    params.load_yumi    = node["yumi_node"]["load_yumi"].as<bool>();
    params.load_rviz    = node["yumi_node"]["load_rviz"].as<bool>();
    params.two_grippers = node["yumi_node"]["two_grippers"].as<bool>();

    params.debug_interface    = node["yumi_interface"]["debug"].as<bool>();
    params.bounded_solution   = node["yumi_interface"]["bounded_solution"].as<bool>();
    params.module_name_left   = node["yumi_interface"]["module_name_left"].as<std::string>();
    params.module_name_right  = node["yumi_interface"]["module_name_right"].as<std::string>();
    params.end_effector_left  = node["yumi_interface"]["end_effector_left"].as<std::string>();
    params.end_effector_right = node["yumi_interface"]["end_effector_right"].as<std::string>();
}

// FUNCTION PROTOTYPS
void displayParameters(Parameters);
void updateConfigFile(Parameters&);

// ====================================================================================
// ========== IF USING CONFIG FILE, NEED TO ADD YAML REPO'S TO SETUP WS FILE ==========
// ====================================================================================

// MAIN FUNCTION
int main(int argc, char **argv) {

    init(argc, argv, "config_test");
    NodeHandle node_handle;

    // INITIALIZE VARIABLES
    Parameters params;

    // GET CONFIGURATION DATA FROM CONFIG FILE
    YAML::Node config = YAML::LoadFile(YUMI_SCRIPTS_DIRECTORY + "src/config.yaml");
    config["yumi_interface"]["debug"] = true;
    config >> params;

    updateConfigFile(params);

    // DIPLAY CONFIGURATION DATA RETRIEVED FROM CONFIG FILE
    displayParameters(params);
    
    return 0;
}

void displayParameters(Parameters params) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-16
*/
    ROS_INFO("      Debug Mode (Node): %s", params.debug_node?"true":"false");
    ROS_INFO("              Load YuMi: %s", params.load_yumi?"true":"false");
    ROS_INFO("              Load RViz: %s", params.load_rviz?"true":"false");
    ROS_INFO("           Two Grippers: %s", params.two_grippers?"true":"false");
    ROS_INFO(" Debug Mode (Interface): %s", params.debug_interface?"true":"false");
    ROS_INFO("      Bounded Solutions: %s", params.bounded_solution?"true":"false");
    ROS_INFO("       Left Module Name: %s", params.module_name_left.c_str());
    ROS_INFO("      Right Module Name: %s", params.module_name_right.c_str());
    ROS_INFO(" Left End Effector Name: %s", params.end_effector_left.c_str());
    ROS_INFO("Right End Effector Name: %s", params.end_effector_right.c_str());
}

void updateConfigFile(Parameters& params) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
    DATE CREATED: 2016-08-25
*/
    YAML::Emitter output;

    output << YAML::BeginMap;
    output << YAML::Key << "yumi_node";
    output << YAML::Value << YAML::BeginMap;
        output << YAML::Key << "debug";
        output << YAML::Value << params.debug_node;
        output << YAML::Key << "load_yumi";
        output << YAML::Value << params.load_yumi;
        output << YAML::Key << "load_rviz";
        output << YAML::Value << params.load_rviz;
        output << YAML::Key << "two_grippers";
        output << YAML::Value << params.two_grippers;
        output << YAML::EndMap;
    output << YAML::Key << "yumi_interface";
    output << YAML::Value << YAML::BeginMap;
        output << YAML::Key << "debug";
        output << YAML::Value << params.debug_interface;
        output << YAML::Key << "bounded_solution";
        output << YAML::Value << params.bounded_solution;
        output << YAML::Key << "module_name_left";
        output << YAML::Value << params.module_name_left;
        output << YAML::Key << "module_name_right";
        output << YAML::Value << params.module_name_right;
        output << YAML::Key << "end_effector_left";
        output << YAML::Value << params.end_effector_left;
        output << YAML::Key << "end_effector_right";
        output << YAML::Value << params.end_effector_right;
        output << YAML::EndMap;
    output << YAML::EndMap;

    std::ofstream output_file(YUMI_SCRIPTS_DIRECTORY + "src/config.yaml"); 
    output_file << output.c_str();
}


