cmake_minimum_required(VERSION 2.8.3)
project(splines)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	geometry_msgs
	tf
	rosbag
	roslib
	nav_msgs
	)

## Generate messages in the 'msg' folder
add_message_files( DIRECTORY msg FILES Coeff.msg State.msg )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs )


catkin_package( CATKIN_DEPENDS message_runtime std_msgs )

add_executable( polyTraj src/polyTraj.cpp )
add_dependencies( polyTraj ${catkin_LIBRARIES} )