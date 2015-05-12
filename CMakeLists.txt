cmake_minimum_required(VERSION 2.8.3)

project(cloud_fusion)

find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	tf
)

find_package(
	PCL REQUIRED COMPONENTS
	common
	io
	search
	features
	registration
	filters
	ros
)

catkin_package(
	LIBRARIES cloud_fusion
	CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
	DEPENDS system_lib PCL
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable (cloud_fusion src/cloud_fusion.cpp)

target_link_libraries (cloud_fusion
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_dependencies(cloud_fusion cloud_fusion_cpp)
