#include <ros/ros.h>

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "polyTraj_node");
	std::cout << "Poly traj :)" << std::endl;
	ros::spinOnce();
}