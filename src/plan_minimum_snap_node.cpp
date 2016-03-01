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
		counter=0;

		eval_thread = std::thread(&PlanMinimumSnapNode::eval_thread_function, this);

		waypoints_sub = nh.subscribe(waypoint_topic, 1, &PlanMinimumSnapNode::OnWaypoints, this);
		pose_sub = nh.subscribe(pose_topic, 1, &PlanMinimumSnapNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &PlanMinimumSnapNode::OnVelocity, this);

		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);

		poly_samples_pub = nh.advertise<geometry_msgs::PoseStamped>(samples_topic, 1);

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
		quad_spline_exists = true;
		std::cout << counter << " evals before last spline" << std::endl;
		counter=0;


		// put the quad spline into a thread

		mutex.lock();
		gotPose = gotVelocity = gotWaypoints = false;
		mutex.unlock();
	}

private:

	void eval_thread_function() {

		ros::Rate spin_rate(100);

		while (ros::ok()) {
			if (quad_spline_exists) {
				//std::cout << "I'm in some function and counter is " << counter << std::endl;
				std::cout << "also the current derivs of quad splines is" <<
				waypoint_interpolator.getCurrentDerivativesOfQuadSpline() << std::endl;
				
				Eigen::MatrixXd current_derivatives = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
				

				// create a acl_fsw::QuadGoal local_goal_msg
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

				counter++;

				geometry_msgs::PoseStamped poly_samples_msg;
				poly_samples_msg.pose.position.x = current_derivatives(0,0);
				poly_samples_msg.pose.position.y = current_derivatives(1,0);
				poly_samples_msg.pose.position.z = current_derivatives(2,0);
				poly_samples_msg.header.frame_id = "world";
				poly_samples_msg.header.stamp = ros::Time::now();

				poly_samples_pub.publish(poly_samples_msg);
				ros::spinOnce();

			}
			spin_rate.sleep();
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
		size_t num_waypoints = std::min(10, (int) waypoints.poses.size());
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
	bool quad_spline_exists;

	std::mutex mutex;

	WaypointInterpolator waypoint_interpolator;

	std::thread eval_thread;
	int counter;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



int main(int argc, char* argv[]) {
	std::cout << "Initializing plan minimum snap node" << std::endl;

	ros::init(argc, argv, "PlanMinimumSnapNode");
	ros::NodeHandle nh;

	// PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");
	PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/RQ01/pose", "/RQ01/vel", "/goal_passthrough", "/poly_samples");


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