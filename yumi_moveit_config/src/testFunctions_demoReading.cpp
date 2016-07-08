#include <algorithm> // allow std::find functionality

struct leadThroughRAPID {
	std::string module_name;
	std::vector<geometry_msgs::Pose> poses;
	std::vector<std::vector<int, 3>> confdata;
	std::vector<bool> gripper_positiions;
};

leadThroughRAPID getYuMiLeadThroughData(std::string);
void getRobtargetData(std::string, geometry_msgs::Pose&, std::vector<int>);

leadThroughRAPID getYuMiLeadThroughData(std::string fileName) {
	// Initialize Variables
	std::string line; // variable to retrieve each line of text from the input file
	std::string empty; // variable to store unwanted data from file
	std::string module_name; // variable to store the module name
	std::string data_type; // variable to store the data type from the provided input file
	std::string robtarget;

	geometry_msgs::Pose pose;
	std::vector<geometry_msgs::Pose> poses;
	std::vector<int> confdata(3);
	std::vector<std::vector<int>> confdatas;
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
 
	ROS_INFO("Getting file data type."); // notify the user that the file data type for the provided input file is being retrived

	// Get Provided File Data Type
	std::ifstream text_file(input_file.c_str()); // open provided text file
	if (text_file.is_open()) { // if the file was able to successfully open
		// Get Module Name for Provided File
		if (std::getline(text_file, line)) { // if the file exists and contains a first line
			line_index++; // increment the current line counter
			ROS_INFO("Pulling line: %d",line_index);

			std::istringstream first_line(line); // create a string steam element for delimiting data by spaces
			first_line >> empty >> module_name; // get the module name
			while (std::getline(text_file, line)) {
				line_index++; // increment the current line counter
				ROS_INFO("Pulling line: %d",line_index);

				std::istringstream current_line(line); // create a string steam element for delimiting data by spaces
				current_line >> data_type;

				if (data_type.compare(0, 1, "P") == 0) {
					robtarget_end = 1;
					ROS_INFO("Reading from Main function.");
				} else if (!(robtarget_end)) {
					current_line >> empty >> data_type;
					if (data_type.compare(0, 1, "r")) {
						current_line >> store_point_name >> empty >> robtarget;
						getRobtargetData(robtarget, pose, confdata);

						poses.push_back(pose);
						confdatas.push_back(confdata);
						store_point_names.push_back(store_point_name);
					} else if (data_type.compare(0, 1, "s")) {
						std::string yumi_app;
						current_line >> yumi_app;
						if (yumi_app.compare(0, 8, "YuMi_App") != 0) {
							ROS_ERROR("The provided file was not generated from the YuMi Windows 10 App.");
							success = false;
							break;
						}
					}
				} else {
					execution_line++;
					if (data_type.compare("MoveSync") == 0) {
						current_line >> move_point_name;
					} else if (data_type.compare("OpenHand;") == 0) {
						gripper_position = 1;
					} else if (data_type.compare("CloseHand;") == 0) {
						gripper_position = 0;
					}
					move_point_names.push_back(move_point_name);
					gripper_positions.push_back(gripper_position);
					ROS_INFO("Main function line: %d, pose name: %s, gripper position (open is 1): %d", execution_line, move_point_name.c_str(), gripper_position);
				}
			}
		} else { // if the provided file is empty
			ROS_ERROR("The provided file is empty."); // notify the user that the provided file is empty
			success = false; // indicate that there was an error
		}
		text_file.close(); // close the text file
	} else { // if the file was not open successfully
		ROS_ERROR("The provided file name could not be opened or does not exist."); // notify user of failure to open file
		ROS_WARN("File: %s",inputFile.c_str()); // notify user of the supplied file
		success = false; // indicate that there was an error
	}

	leadThroughRAPID lead_through_RAPID;
	if (success) {
		lead_through_RAPID.module_name = module_name;

		int store_location;
		for (int point = 0; point < move_point_names.size(); point++) {
			lead_through_RAPID.gripper_position = gripper_positions[point];

			store_location = std::find(store_point_names.begin(), store_point_names.end(), move_point_names[point]);
			lead_through_RAPID.poses = poses[store_location];
			lead_through_RAPID.confdata = confdata[store_location];
		}
	} else {
		ROS_ERROR("Not able to parse YuMi lead through file.");
	}
	return lead_through_RAPID;
}

void getRobtargetData(std::string robtarget, geometry_msgs::Pose& pose, std::vector<int> confdata(3)) {

	std::string robtarget_search = "0123456789.";

	bool pose_pulled = 0;
	std::size_t start_char = 2;
	std::size_t end_char = 0;
	int element = 0;

	while (!(pose_pulled)) {
		element++;

		end_char = robtarget.find_first_not_of(robtarget_search, start_char);
		int current_val = stoi(robtarget.substr(start_char, (end_char-start_char), nullptr);

		if (element == 1) { pose.position.x = current_val; }
		else if (element == 2) { pose.position.y = current_val; }
		else if (element == 3) { pose.position.z = current_val; }
		else if (element == 4) { pose.orientation.w = current_val; }
		else if (element == 5) { pose.orientation.x = current_val; }
		else if (element == 6) { pose.orientation.y = current_val; }
		else if (element == 7) { pose.orientation.z = current_val; }
		else if (element == 8) { confdata[0] = current_val; }
		else if (element == 9) { confdata[1] = current_val; }
		else if (element == 10) { confdata[2] = current_val; }

		start_char = robtarget.find_first_of(robtarget_search, end_char);
	}
}