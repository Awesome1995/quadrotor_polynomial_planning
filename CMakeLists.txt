cmake_minimum_required(VERSION 2.8.3)
project(splines)

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	std_msgs
	geometry_msgs
	tf
	nav_msgs
	message_generation
	)

find_package( Eigen3 REQUIRED )

## Generate messages in the 'msg' folder
add_message_files( DIRECTORY msg FILES Coeff.msg State.msg )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs nav_msgs)

catkin_package(

)

include_directories ( src )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${CATKIN_DEVEL_PREFIX}/../external/pronto-distro/build/include) #for eigen_utils


add_executable( polyTraj src/polyTraj.cpp )
target_link_libraries( polyTraj ${catkin_LIBRARIES} )

add_library( poly src/poly.cpp)
target_link_libraries(polyTraj poly)
add_executable( polyTest tests/poly-test.cpp )
target_link_libraries(polyTest poly)
