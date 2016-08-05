struct poseConfig {
	geometry_msgs::Pose pose;
	double gripper_position;
	std::vector<int> confdata;
	double external_axis_position;
};

struct trajectoryPoses {
	std::string group_name;
	std::string intended_group;
	std::vector<poseConfig> poses_left;
	std::vector<poseConfig> poses_right;
	bool gripper_attached_left  = false;
	bool gripper_attached_right = false; 
	int total_points;
};

struct RAPIDModuleData {
	std::string module_name;
	std::vector<std::string> pose_names;
	std::vector<poseConfig> poses;
	int total_points;
};

struct trajectoryJoints {
	std::string group_name;
	std::string intended_group;
	int total_joints;
	std::vector<std::vector<double>> joints;
	int total_points;
};

struct planner {
	std::string groupName; // include group name
	std::vector<planningInterface::MoveGroup::Plan> plans; // include vector to retrieve planned paths for trajectory points
	int totalPlans; // include count of total plans
	bool success; // include boolean to retrieve whether the planner was successful or not
};