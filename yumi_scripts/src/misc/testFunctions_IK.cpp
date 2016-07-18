robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
geometry_msgs::PoseStamped pose;

std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);

req.group_name = "right_arm";
moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(group.getEndEffector(), pose, tolerance_pose, tolerance_angle);
req.goal_constraints.push_back(pose_goal);

planning_pipeline->generatePlan(planning_scene, req, res);
if(res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}

void preplanTrajectory(planningInterface::MoveGroup& group, trajectoryPoses& pose_trajectory, std::vector<std::vector<int>> confdata) {

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

	planning_interface::MotionPlanRequest motion_request;
	planning_interface::MotionPlanResponse motion_response;
	geometry_msgs::PoseStamped poses_left  = pose_trajectory.pose_left;
	geometry_msgs::PoseStamped poses_right = pose_trajectory.pose_right;

	std::vector<double> tolerance_pose(3, 0.001);
	std::vector<double> tolerance_angle(3, 0.001);
	motion_request.group_name = group.getName();

	moveit_msgs::Constraints pose_constraints  = kinematic_constraints::constructGoalConstraints(group.getEndEffector(), pose, tolerance_pose, tolerance_angle);
	moveit_msgs::Constraints joint_constraints = setAxisConfigurations(confdata, planningInterface::MoveGroup& group);
	moveit_msgs::Constraints goal_constraints  = kinematic_constraints::mergeConstraints(pose_constraints, joint_constraints);
	motion_request.goal_constraints.push_back(pose_goal);

	planning_pipeline->generatePlan(planning_scene, motion_request, motion_response);
	if (motion_response.error_code_.val != motion_response.error_code_.SUCCESS) {
		ROS_ERROR("Could not compute plan successfully");
		ROS_WARN("Error code: %d", motion_response.error_code_.val);
	}

	moveit_msgs::MotionPlanResponse motion_plan;
  	motion_response.getMessage(motion_plan);
}


moveit_msgs::Constraints setAxisConfigurations(std::vector<int> confdata(3), planningInterface::MoveGroup& group) {
/*  PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com

	INPUT(S):
	  > confdata - ABB naming convention for axis configurations for a specified target
*/
	// Initialize Variables
	const double PI = 3.1416;
	const double TOLERANCE = PI/4; // 45 degree tolerance for max and min of joint position constraint
	const double WEIGHT = 1;
	const std::vector<int> config_joints = { 0, 3, 5 }; // axis configuration joints (joint 1, 4, and 6 starting with index 0)

	std::string group_name = group.getName(); // get the name of the provided group
	std::vector<std::string> active_joint_names = group.getActiveJoints(); // get the names of all the active joints

	moveit_msgs::Constraints goal_constraints;
	for (int constraint = 0; constraint < joint_constraints.size(); constraint++) {
		std::vector<moveit_msgs::JointConstraint> joint_constraint;
		joint_constraint.joint_name      = active_joints_names[config_joints[constraint]];
		joint_constraint.position        = (confdata[constraint] * PI/2) + TOLERANCE;
		joint_constraint.tolerance_above = TOLERANCE;
		joint_constraint.tolerance_below = TOLERANCE;
		joint_constraint.weight          = WEIGHT; 

		goal_constraints.joint_constraints.push_back(joint_constraint[constraint]);
	}

	return goal_constraints;
}

std::vector<double> computeIK(ServiceClient serviceIK, trajectoryPoses& pose_trajectory, planningInterface::MoveGroup& group) {
	
	moveit_msgs::GetPositionIK::Request serviceRequest;
	moveit_msgs::GetPositionIK::Response serviceResponse;
	
	std::vector<geometry_msgs::PoseStamped> poseStamped(2);
	std::string left_arm_eef  = left_arm.getEndEffector();
	std::string right_arm_eef = right_arm.getEndEffector();
	
	geometry_msgs::PoseStamped poseStamped_left;
	geometry_msgs::PoseStamped poseStamped_right;
	
	poseStamped_left.header.frame_id  = left_arm_eef;
	poseStamped_right.header.frame_id = right_arm_eef;
	poseStamped_left.pose  = pose_trajectory.pose_left[0];
	poseStamped_right.pose = pose_trajectory.pose_right[0];
	
	poseStamped[0] = poseStamped_left;
	poseStamped[1] = poseStamped_right;
	
	std::vector<std::string> joint_names = group.getActiveJoints();
	
	ROS_INFO("Working 1");
	
	serviceRequest.ik_request.group_name       = "both_arms";
	serviceRequest.ik_request.avoid_collisions = true;
	serviceRequest.ik_request.attempts         = 10;
	serviceRequest.ik_request.constraints      = setAxisConfigurations(pose_trajectory.confdata_left, planningInterface::MoveGroup& group, joint_values);
	serviceRequest.ik_request.robot_state.joint_state.name     = joint_names;
	serviceRequest.ik_request.robot_state.joint_state.position = currentJointValues;
	serviceRequest.ik_request.pose_stamped_vector = poseStamped;
	
	ROS_INFO("Working 2");
	
	serviceIK.call(serviceRequest,serviceResponse);
	
	ROS_INFO("Working 3");
	
	std::vector<double> jointValues     = serviceResponse.solution.joint_state.position;
	std::vector<std::string> jointNames = serviceResponse.solution.joint_state.name;
	for (int joint = 0; joint < jointValues.size(); joint++) {
		ROS_INFO("Joint %s: %.4f",jointNames[joint].c_str(),jointValues[joint]);
	}
	
	ROS_INFO("Joint value size %lu",jointValues.size());
	ROS_INFO("Error code: %d",serviceResponse.error_code);
	
	return jointValues;
}


