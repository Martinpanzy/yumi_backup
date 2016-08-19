cmake_minimum_required(VERSION 2.8.3)
project(yumi_scripts)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_ros_planning_interface
	message_generation
	geometry_msgs
	std_msgs
	rosbag
	yaml_cpp_catkin
)

add_message_files(FILES
	JointMsg.msg
	TrajectoryJointMsg.msg
	PlannerMsg.msg
)

generate_messages(DEPENDENCIES
	std_msgs
	moveit_msgs
	geometry_msgs
)

catkin_package(CATKIN_DEPENDS 
	message_runtime
)

install(
	DIRECTORY launch config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(config_test src/config_test.cpp)
target_link_libraries(config_test ${catkin_LIBRARIES})

add_executable(yumi_interface src/yumi_interface.cpp)
target_link_libraries(yumi_interface ${catkin_LIBRARIES})

add_executable(moveit_interface src/moveit_interface.cpp)
target_link_libraries(moveit_interface ${catkin_LIBRARIES})

add_executable(lead_through src/lead_through.cpp)
target_link_libraries(lead_through ${catkin_LIBRARIES})

# Check c++11 / c++0x
# REFERENCE: http://answers.ros.org/question/152276/is-there-a-way-to-enable-c11-support-for-catkin-packages/
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
