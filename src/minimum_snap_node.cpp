#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "Polynomial.hpp"
#include <std_msgs/String.h>

class MinimumSnapNode {
public:

	MinimumSnapNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& samples_topic) {
		waypoints_sub = nh.subscribe(waypoint_topic, 10, &MinimumSnapNode::OnWaypoints, this);
		poly_samples_pub = nh.advertise<nav_msgs::Odometry>(samples_topic, 10);
		std::cout << "Sleeping" << std::endl;
		sleep(2);
		std::cout << "Done sleeping" << std::endl;

	}

	static Eigen::MatrixXd SamplePoly(Polynomial & poly, double t0, double tf, int num_samples) {

		// 1 should be dimension of polynomial
		Eigen::MatrixXd samples (1,num_samples);

		double dt = (tf - t0) / (num_samples - 1);
		double t = 0;
		for (int i = 0; i < num_samples; i++) {
			t =  i * dt;
			samples.col(i) << poly.HornersEval(t);
			//std::cout << "Time is " << t << std::endl;
		}

		std::cout << samples << std::endl;
		return samples;

	}

	void publishOdomPoints(Eigen::MatrixXd & poly_samples) {

		for (int i = 0 ; i < poly_samples.cols(); i++ ) {
			nav_msgs::Odometry poly_samples_msg;
			Eigen::VectorXd point = poly_samples.col(i);
			poly_samples_msg.pose.pose.position.x = point[0];
			poly_samples_msg.header.frame_id = "map";
			poly_samples_msg.header.stamp = ros::Time::now();

			poly_samples_pub.publish(poly_samples_msg);
			ros::spinOnce();
		}
	}

private:

	void OnWaypoints(nav_msgs::Path const& waypoints) {
		std::cout << "Got waypoints" << std::endl;
	}

	ros::Subscriber waypoints_sub;
	ros::Publisher poly_samples_pub;
};



int main(int argc, char* argv[]) {
	std::cout << "Initializing poly traj node" << std::endl;

	ros::init(argc, argv, "polyTraj_node");
	ros::NodeHandle nh;

	MinimumSnapNode minimum_snap_node(nh, "/waypoint_list", "/poly_samples");

	//minimum_snap_node.publishOdomPoints(samples);

	ros::spin();
}