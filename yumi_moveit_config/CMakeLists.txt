cmake_minimum_required(VERSION 2.8.3)
project(yumi_moveit_config)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_ros_planning_interface
	geometry_msgs
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(demo_script src/demo_script.cpp)
target_link_libraries(demo_script ${catkin_LIBRARIES})


