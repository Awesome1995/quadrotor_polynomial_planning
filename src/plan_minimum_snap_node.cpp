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
#include <std_srvs/Empty.h>


class PlanMinimumSnapNode {
public:

	PlanMinimumSnapNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& pose_topic, std::string const& velocity_topic, std::string const& local_goal_topic, std::string const& samples_topic) {
		gotPose = gotVelocity = gotWaypoints = false;
		quad_spline_exists = false;

		nh.getParam("spline_horizon_distance", spline_horizon_distance);
		nh.getParam("derivative_to_minimize", derivative_to_minimize);
		nh.getParam("spline_eval_rate", spline_eval_rate);
		nh.getParam("publish_spline_path", publish_spline_path);
		nh.getParam("max_waypoints", max_waypoints);

		waypoint_interpolator_building.setDerivativeToMinimize(derivative_to_minimize);


		pose_sub = nh.subscribe(pose_topic, 1, &PlanMinimumSnapNode::OnPose, this);
		velocity_sub = nh.subscribe(velocity_topic, 1, &PlanMinimumSnapNode::OnVelocity, this);


		waypoints_sub = nh.subscribe(waypoint_topic, 1, &PlanMinimumSnapNode::OnWaypoints, this);


		local_goal_pub = nh.advertise<acl_fsw::QuadGoal> (local_goal_topic, 1);

		poly_samples_pub = nh.advertise<nav_msgs::Path>(samples_topic, 1);


		reset_sequencer_server = nh.advertiseService("min_snap/ResetSequencer", &PlanMinimumSnapNode::ResetSequencerHandler, this);

		std::cout << "Finished constructing the plan min snap node, waiting for waypoints" << std::endl;

	}

	bool ResetSequencerHandler(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
		ROS_INFO("Got reset sequence");
		return true;
	}

	bool readyToCompute() {
		return gotPose && gotVelocity && gotWaypoints;
	}

	void StartEvalThread() {
		eval_thread = std::thread(&PlanMinimumSnapNode::eval_thread_function, this);
	};

	void PublishSplinePath(size_t samples = 100) {
		nav_msgs::Path poly_samples_msg;
		poly_samples_msg.header.frame_id = "world";
		poly_samples_msg.header.stamp = ros::Time::now();
		double final_time = waypoint_interpolator_building.getTotalTime();

		double dt = final_time/(samples-1);
		double t;

		for (size_t i = 0; i < samples; i++) {
			t = i * dt;
			Eigen::MatrixXd derivatives = waypoint_interpolator_building.getDerivativesOfQuadSplineAtTime(t);
			poly_samples_msg.poses.push_back(PoseFromDerivativeMatrix(derivatives));
		}
		poly_samples_pub.publish(poly_samples_msg);
	}

	void computeMinSnapNode() {


		waypoint_interpolator_building.setWayPoints(waypoints_matrix);
		waypoint_interpolator_building.setCurrentVelocities(velocity_x_y_z_yaw);
		waypoint_interpolator_building.setCurrentPositions(pose_x_y_z_yaw);
		waypoint_interpolator_building.setTausWithHeuristic();
		waypoint_interpolator_building.computeQuadSplineWithFixedTimeSegments();
		ROS_INFO("Computing spline trajectory for %d waypoints", (int) waypoints_matrix.cols() );

		if (publish_spline_path) {
			PublishSplinePath();
		};

		quad_spline_exists = true;

		mutex.lock();
		waypoint_interpolator_built = waypoint_interpolator_building;
		gotPose = gotVelocity = gotWaypoints = false;
		mutex.unlock();
	}

private:

	void eval_thread_function() {

		ros::Rate spin_rate(spline_eval_rate);

		while (ros::ok()) {
			if (quad_spline_exists) {

				mutex.lock();

				Eigen::MatrixXd current_derivatives = waypoint_interpolator_built.getCurrentDerivativesOfQuadSpline();

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

				mutex.unlock();

				ros::spinOnce();

			}
			spin_rate.sleep();
		}

	}

	geometry_msgs::PoseStamped PoseFromDerivativeMatrix(Eigen::MatrixXd const& derivatives) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = derivatives(0,0);
		pose.pose.position.y = derivatives(1,0);
		pose.pose.position.z = derivatives(2,0);
		pose.header.frame_id = "world";
		pose.header.stamp = ros::Time::now();
		return pose;
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

	Eigen::Vector3d VectorFromPose(geometry_msgs::PoseStamped const& pose) {
		return Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	}


	void OnWaypoints(nav_msgs::Path const& waypoints) {

		int max_waypoints = 10;
		int waypoints_to_check = std::min((int) waypoints.poses.size(), max_waypoints);

		waypoints_matrix.resize(4, waypoints_to_check);
		waypoints_matrix.col(0) << VectorFromPose(waypoints.poses[0]), 0.0;  // yaw is currently hard set to be 0
		double distance_so_far = 0.0;
		double distance_to_add;
		double distance_left;
		Eigen::Vector3d truncated_waypoint;
		Eigen::Vector3d p1, p2;
		int i;
		for (i = 0; i < waypoints_to_check - 1; i++){
			p1 = VectorFromPose(waypoints.poses[i]);
			p2 = VectorFromPose(waypoints.poses[i+1]);
			distance_to_add = (p2-p1).norm();
			if ((distance_to_add + distance_so_far) < spline_horizon_distance) {
				distance_so_far += distance_to_add;
				waypoints_matrix.col(i + 1) << p2, 0.0; // yaw is currently hard set to be 0
			}
			else {
				distance_left = spline_horizon_distance - distance_so_far;
				truncated_waypoint = p1 + (p2-p1) / distance_to_add * distance_left;
				distance_so_far = distance_so_far + distance_left;
				waypoints_matrix.col(i + 1) << truncated_waypoint, 0.0; // yaw is currently hard set to be 0
				i++;
				break;

			}
		}


		waypoints_matrix.conservativeResize(4, i+1);

		mutex.lock();
		gotWaypoints = true;
		mutex.unlock();

	}

	ros::ServiceServer reset_sequencer_server;
	ros::Subscriber waypoints_sub;
	ros::Subscriber pose_sub;
	ros::Subscriber velocity_sub;
	ros::Publisher local_goal_pub;
	ros::Publisher poly_samples_pub;

	nav_msgs::Path waypoints;
	nav_msgs::Path previous_waypoints;

	size_t num_waypoints;

	int derivative_to_minimize;
	double spline_eval_rate;
	bool publish_spline_path;
	int max_waypoints;

	Eigen::Vector4d pose_x_y_z_yaw;
	Eigen::Vector4d velocity_x_y_z_yaw;
	double spline_horizon_distance;
	Eigen::Matrix<double, 4, Eigen::Dynamic> waypoints_matrix;


	bool gotPose, gotVelocity, gotWaypoints;
	bool quad_spline_exists;

	std::mutex mutex;

	WaypointInterpolator waypoint_interpolator_building;
	WaypointInterpolator waypoint_interpolator_built;

	std::thread eval_thread;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


int main(int argc, char* argv[]) {
	std::cout << "Initializing plan minimum snap node" << std::endl;

	ros::init(argc, argv, "PlanMinimumSnapNode");
	ros::NodeHandle nh;

	PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/FLA_ACL02/pose", "/FLA_ACL02/vel", "/goal_passthrough", "/poly_samples");
	//PlanMinimumSnapNode plan_minimum_snap_node(nh, "/waypoint_list", "/RQ01/pose", "/RQ01/vel", "/goal_passthrough", "/poly_samples");
	plan_minimum_snap_node.StartEvalThread();

	while (ros::ok()) {
		if (plan_minimum_snap_node.readyToCompute()) {
			plan_minimum_snap_node.computeMinSnapNode();
		}
		ros::spinOnce();
	}
}