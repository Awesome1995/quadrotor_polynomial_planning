cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(splines)

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	std_msgs
	geometry_msgs
	tf
	nav_msgs
	acl_fsw
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


add_executable( plan_minimum_snap_node src/plan_minimum_snap_node.cpp)
target_link_libraries( plan_minimum_snap_node ${catkin_LIBRARIES} )

add_library( Polynomial src/Polynomial.cpp)
add_library( PolysAndQuadSplines src/PiecewisePolynomial.cpp src/QuadSpline.cpp src/OptimalPiecewisePolynomialGenerator.cpp src/TimeOptimizedQuadSplineGenerator.cpp)
add_library( WaypointsAndSequencing src/WaypointInterpolator.cpp src/QuadSplineSequencer.cpp)


add_executable( polyTest tests/poly-test.cpp )
target_link_libraries(polyTest Polynomial)
target_link_libraries(polyTest PolysAndQuadSplines)
target_link_libraries( PolysAndQuadSplines Polynomial )
target_link_libraries( WaypointsAndSequencing PolysAndQuadSplines )
target_link_libraries(plan_minimum_snap_node WaypointsAndSequencing)

catkin_add_gtest( SplinesTests tests/Splines_gtests.cpp)
target_link_libraries( SplinesTests PolysAndQuadSplines WaypointsAndSequencing)

add_executable( TimeOptimizedQuadSplineTest tests/time_optimized_test.cpp )
target_link_libraries( TimeOptimizedQuadSplineTest PolysAndQuadSplines)