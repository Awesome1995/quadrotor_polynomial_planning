#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PolyTrajNode {
public:

	PolyTrajNode(ros::NodeHandle & nh, std::string const& waypoint_topic) {
		waypoints_sub = nh.subscribe(waypoint_topic, 10, &PolyTrajNode::OnWaypoints, this);
	}

private:

	void OnWaypoints(nav_msgs::Path const& waypoints) {
		std::cout << "Got waypoints" << std::endl;
	}

	ros::Subscriber waypoints_sub;
};



int main(int argc, char* argv[]) {
	ros::init(argc, argv, "polyTraj_node");
	ros::NodeHandle nh;

	std::cout << "Poly traj :)" << std::endl;

	PolyTrajNode poly_traj_node(nh, "/waypoint_list");

	ros::spin();
}