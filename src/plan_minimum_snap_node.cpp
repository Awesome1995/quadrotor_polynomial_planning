#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "Polynomial.hpp"
#include <std_msgs/String.h>
#include "acl_fsw/QuadGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/tf.h"
#include <mutex>
#include "WaypointInterpolator.h"
#include <thread>


class PlanMinimumSnapNode {
public:

	PlanMinimumSnapNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& pose_topic, std::string const& velocity_topic, std::string const& local_goal_topic, std::string const& samples_topic) {
		gotPose = gotVelocity = gotWaypoints = false;

		eval_thread = std::thread(&PlanMinimumSnapNode::some_function, this);

		waypoints_sub = nh.subscribe(waypoint_topic, 1, &PlanMinimumSnapNode::OnWaypoints, this);
		pose_sub = nh.subscribe(pose_topic, 1, &PlanMinimumSnapNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &PlanMinimumSnapNode::OnVelocity, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);

		poly_samples_pub = nh.advertise<nav_msgs::Odometry>(samples_topic, 1);



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

	bool readyToCompute() {
		return gotPose && gotVelocity && gotWaypoints;
	}

	void computeMinSnapNode() {
		waypoint_interpolator.setWayPoints(waypoints_matrix);
		waypoint_interpolator.setCurrentVelocities(velocity_x_y_z_yaw);
		waypoint_interpolator.setCurrentPositions(pose_x_y_z_yaw);
		waypoint_interpolator.setTausWithHeuristic();
		waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();

		std::cout << "Computed quad splines successfully " << std::endl;


		// put the quad spline into a thread

		mutex.lock();
		gotPose = gotVelocity = gotWaypoints = false;
		mutex.unlock();
	}

private:

	void some_function() {
		while (true) {
			std::cout << "I'm in some function " << std::endl;
			usleep(10000);
		}
	}

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		// store it as Eigen vector
		pose_x_y_z_yaw << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, tf::getYaw(pose.pose.orientation);
		//std::cout << "How's my eigen vector for pose? " << pose_x_y_z_yaw << std::endl;
		mutex.lock();
		gotPose = true;
		mutex.unlock();
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		// store it as Eigen vector
		velocity_x_y_z_yaw << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, 0.0; // NOTE: would definitely be preferable to actually use the yawdot coming from state estimator
		//std::cout << "How's my eigen vector for velocity? " << velocity_x_y_z_yaw << std::endl;
		mutex.lock();
		gotVelocity = true;
		mutex.unlock();
	}

	void OnWaypoints(nav_msgs::Path const& waypoints) {
		// store as Eigen matrix
		int counter = 0;
		size_t num_waypoints = std::min(5, (int) waypoints.poses.size());
		waypoints_matrix.resize(4,num_waypoints);
		for (int i = 0; i < num_waypoints; i++) {
			auto const& waypoint_i = waypoints.poses[i];
			waypoints_matrix.col(i) << waypoint_i.pose.position.x, waypoint_i.pose.position.y, waypoint_i.pose.position.z, 0.0; // if we want yaw poses from waypoints, use instead tf::getYaw(waypoint_i.pose.orientation)
		}
		//std::cout << waypoints_matrix << std::endl;
		mutex.lock();
		gotWaypoints = true;
		mutex.unlock();
	}


	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Publisher local_goal_pub;
	ros::Publisher poly_samples_pub;

	nav_msgs::Path waypoints;

	size_t num_waypoints;

	Eigen::Vector4d pose_x_y_z_yaw;
	Eigen::Vector4d velocity_x_y_z_yaw;
	Eigen::MatrixXd waypoints_matrix;

	bool gotPose, gotVelocity, gotWaypoints;

	std::mutex mutex;

	WaypointInterpolator waypoint_interpolator;

	std::thread eval_thread;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



int main(int argc, char* argv[]) {
	std::cout << "Initializing plan minimum snap node" << std::endl;

	ros::init(argc, argv, "PlanMinimumSnapNode");
	ros::NodeHandle nh;

	PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");



	//minimum_snap_node.publishOdomPoints(samples);

	while (ros::ok()) {
		if (!plan_minimum_snap_node.readyToCompute()) {
			ros::spinOnce();
		}
		else {
			plan_minimum_snap_node.computeMinSnapNode();
		}
	}
	ros::spin();
}