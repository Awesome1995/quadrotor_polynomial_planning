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
		quad_spline_exists = false;

		eval_thread = std::thread(&PlanMinimumSnapNode::eval_thread_function, this);

		waypoints_sub = nh.subscribe(waypoint_topic, 1, &PlanMinimumSnapNode::OnWaypoints, this);
		pose_sub = nh.subscribe(pose_topic, 1, &PlanMinimumSnapNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &PlanMinimumSnapNode::OnVelocity, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);

		poly_samples_pub = nh.advertise<nav_msgs::Path>(samples_topic, 1);

		std::cout << "Sleeping while initializing the minimum snap node" << std::endl;
		sleep(2);
		std::cout << "Done sleeping" << std::endl;

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

		quad_spline_exists = true;

		mutex.lock();
		gotPose = gotVelocity = gotWaypoints = false;
		mutex.unlock();
	}

private:

	void eval_thread_function() {

		ros::Rate spin_rate(100);

		while (ros::ok()) {
			if (quad_spline_exists) {

				Eigen::MatrixXd current_derivatives = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();

				acl_fsw::QuadGoal local_goal_msg;
				local_goal_msg.pos.x = current_derivatives(0,0);
				local_goal_msg.pos.y = current_derivatives(1,0);
				local_goal_msg.pos.z = current_derivatives(2,0);
				local_goal_msg.yaw   = current_derivatives(3,0);

				local_goal_msg.vel.x = current_derivatives(0,1);
				local_goal_msg.vel.y = current_derivatives(1,1);
				local_goal_msg.vel.z = current_derivatives(2,1);
				local_goal_msg.dyaw  = current_derivatives(3,1);

				local_goal_msg.accel.x = current_derivatives(0,2);
				local_goal_msg.accel.y = current_derivatives(1,2);

				local_goal_msg.jerk.x = current_derivatives(0,3);
				local_goal_msg.jerk.y = current_derivatives(1,3);

				local_goal_pub.publish(local_goal_msg);

				nav_msgs::Path poly_samples_msg;

				size_t samples = 100;
				double dt = 1/(samples-1);

				poly_samples_msg.poses[0].pose.position.x =  current_derivatives(0,0);
				poly_samples_msg.poses[0].pose.position.x = current_derivatives(1,0);
				poly_samples_msg.poses[0].pose.position.x = current_derivatives(2,0);

				poly_samples_msg.header.frame_id = "world";
				poly_samples_msg.header.stamp = ros::Time::now();

				poly_samples_pub.publish(poly_samples_msg);
				ros::spinOnce();

			}
			spin_rate.sleep();
		}

	}

	void OnPose( geometry_msgs::PoseStamped const& pose ) {
		pose_x_y_z_yaw << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, tf::getYaw(pose.pose.orientation);
		mutex.lock();
		gotPose = true;
		mutex.unlock();
	}

	void OnVelocity( geometry_msgs::TwistStamped const& twist) {
		velocity_x_y_z_yaw << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z, 0.0; // NOTE: would definitely be preferable to actually use the yawdot coming from state estimator
		mutex.lock();
		gotVelocity = true;
		mutex.unlock();
	}

	void OnWaypoints(nav_msgs::Path const& waypoints) {
		int max_waypoints = 10;															// Currently hard-coded to look at up to next 10 waypoints.  Will be better to switch this to a distance-cutoff receding horizon
		size_t num_waypoints = std::min(max_waypoints, (int) waypoints.poses.size());
		waypoints_matrix.resize(4,num_waypoints);
		for (int i = 0; i < num_waypoints; i++) {
			auto const& waypoint_i = waypoints.poses[i];
			waypoints_matrix.col(i) << waypoint_i.pose.position.x, waypoint_i.pose.position.y, waypoint_i.pose.position.z, 0.0; // if we want yaw poses from waypoints, use instead tf::getYaw(waypoint_i.pose.orientation)
		}
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
	bool quad_spline_exists;

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

	// PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");
	PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/RQ01/pose", "/RQ01/vel", "/goal_passthrough", "/poly_samples");


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