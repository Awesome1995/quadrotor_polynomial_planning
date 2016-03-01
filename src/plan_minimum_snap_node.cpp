#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "Polynomial.hpp"
#include <std_msgs/String.h>
#include "acl_fsw/QuadGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"


class PlanMinimumSnapNode {
public:

	PlanMinimumSnapNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& pose_topic, std::string const& velocity_topic, std::string const& local_goal_topic, std::string const& samples_topic) {
		waypoints_sub = nh.subscribe(waypoint_topic, 10, &PlanMinimumSnapNode::OnWaypoints, this);
		pose_sub = nh.subscribe(pose_topic, 10, &PlanMinimumSnapNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 10, &PlanMinimumSnapNode::OnVelocity, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 10);

		poly_samples_pub = nh.advertise<nav_msgs::Odometry>(samples_topic, 10);

		std::cout << "Sleeping" << std::endl;
		sleep(2);
		std::cout << "Done sleeping" << std::endl;

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

		gotWaypoints = true;
	}

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		std::cout << "Got pose " << std::endl;
		gotPose = true;
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		std::cout << "Got velocity " << std::endl;
		gotVelocity = true;
	}


	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Publisher local_goal_pub;
	ros::Publisher poly_samples_pub;


	bool gotWaypoints;
	bool gotPose;
	bool gotVelocity;

	nav_msgs::Path waypoints;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::TwistStamped twist;
};



int main(int argc, char* argv[]) {
	std::cout << "Initializing plan minimum snap node" << std::endl;

	ros::init(argc, argv, "PlanMinimumSnapNode");
	ros::NodeHandle nh;

	PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");

	//minimum_snap_node.publishOdomPoints(samples);

	ros::spin();
}