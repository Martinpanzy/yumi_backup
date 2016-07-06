cmake_minimum_required(VERSION 2.8.3)
project(yumi_moveit_config)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_ros_planning_interface
	geometry_msgs
	std_msgs
	message_generation
	rosbag
)

add_message_files(FILES
	PrePlan.msg
)

generate_messages(DEPENDENCIES
	moveit_msgs
)

catkin_package(CATKIN_DEPENDS 
	message_runtime
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(demo_script src/demo_script.cpp)
target_link_libraries(demo_script ${catkin_LIBRARIES})
add_executable(lead_through src/lead_through_script_generator.cpp)
target_link_libraries(lead_through ${catkin_LIBRARIES})
add_executable(run_lead_through src/run_lead_through_script.cpp)
target_link_libraries(run_lead_through ${catkin_LIBRARIES})

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
