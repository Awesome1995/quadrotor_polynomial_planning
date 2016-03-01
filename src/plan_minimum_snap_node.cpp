#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "Polynomial.hpp"
#include <std_msgs/String.h>
#include "acl_fsw/QuadGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/tf.h"


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



	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		std::cout << "Got pose " << std::endl;
		// store it as Eigen vector
		pose_x_y_z_yaw << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, tf::getYaw(pose.pose.orientation);
		//std::cout << "How's my eigen vector for pose? " << pose_x_y_z_yaw << std::endl;
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		std::cout << "Got velocity " << std::endl;
		// store it as Eigen vector
		velocity_x_y_z_yaw << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, twist.twist.angular.z; // NOTE: we need to double check on if angular.z is actually what we want for yawdot
		std::cout << "How's my eigen vector for velocity? " << velocity_x_y_z_yaw << std::endl;
	}

	void OnWaypoints(nav_msgs::Path const& waypoints) {
		std::cout << "Got waypoints" << std::endl;
		// store as Eigen vector




	}


	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Publisher local_goal_pub;
	ros::Publisher poly_samples_pub;

	nav_msgs::Path waypoints;

	Eigen::Vector4d pose_x_y_z_yaw;
	Eigen::Vector4d velocity_x_y_z_yaw;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



int main(int argc, char* argv[]) {
	std::cout << "Initializing plan minimum snap node" << std::endl;

	ros::init(argc, argv, "PlanMinimumSnapNode");
	ros::NodeHandle nh;

	PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");

	//minimum_snap_node.publishOdomPoints(samples);

	ros::spin();
}