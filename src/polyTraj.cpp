#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <poly.hpp>
#include <std_msgs/String.h>

class PolyTrajNode {
public:

	PolyTrajNode(ros::NodeHandle & nh, std::string const& waypoint_topic, std::string const& samples_topic) {
		waypoints_sub = nh.subscribe(waypoint_topic, 10, &PolyTrajNode::OnWaypoints, this);
		poly_samples_pub = nh.advertise<nav_msgs::Path>(samples_topic, 10);
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
			std::cout << "Time is " << t << std::endl;
		}

		std::cout << samples << std::endl;
		return samples;

	}

	void publishPath(Eigen::MatrixXd & poly_samples) {

		nav_msgs::Path poly_samples_msg;

		for (int i = 0 ; i < poly_samples.cols(); i++ ) {
			geometry_msgs::PoseStamped pose;
			Eigen::VectorXd point = poly_samples.col(i);
			pose.pose.position.x = point[0];
			poly_samples_msg.poses.push_back(pose);
		}

		poly_samples_msg.header.frame_id = "map";
		poly_samples_msg.header.stamp = ros::Time::now();

		poly_samples_pub.publish(poly_samples_msg);

	}

private:

	void OnWaypoints(nav_msgs::Path const& waypoints) {
		std::cout << "Got waypoints" << std::endl;
	}

	ros::Subscriber waypoints_sub;
	ros::Publisher poly_samples_pub;
};



Eigen::MatrixXd quad_traj_test() {

	using namespace Eigen;

	Polynomial p0, p1, p2;
	Polynomial * polys_constrained[3];
	polys_constrained[0] = &p0;
	polys_constrained[1] = &p1;
	polys_constrained[2] = &p2;

	int n_segments = 3;
	int n_derivatives_specified = 5;


	VectorXd der_initial(n_derivatives_specified);
	der_initial << 0, 0, 0, 0, 0;

	VectorXd der_final(n_derivatives_specified);
	der_final << 1, 0, 0, 0, 0;

	VectorXd der_costs(10);
	der_costs << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;

	VectorXd taus(n_segments);
	taus << 0.75, 0.5, 1; // (spend .75 seconds on the first segment, .5 seconds on the second segment...)

	int n_fixed = 1;

	MatrixXd intermediate_ders(n_derivatives_specified, n_segments - 1);

	VectorXd waypoint_1(n_derivatives_specified);
	waypoint_1 << .4, 0, 0, 0, 0; // Position of .4 for waypoint 1
	intermediate_ders.col(0) << waypoint_1;

	VectorXd waypoint_2(n_derivatives_specified);
	waypoint_2 << .5, 0, 0, 0, 0; // Position of .5 for waypoint 2
	intermediate_ders.col(1) << waypoint_2;

	double cost;

	// Compute the solution using the SPARSE implementation of the unconstrained QP (Richter, Bry & Roy, ISRR 2013)
	// For this one, the solver will allocate the Polynomial object for us
	Polynomial * polys_unconstrained_sparse[3];
	Eigen::MatrixXd opt_ders_unconstrained; // Optimal derivatives
	Eigen::VectorXd opt_costs; // Return costs for each segment
	polyOptPiecewiseDersSparse(taus, der_initial, der_final, der_costs, intermediate_ders, polys_unconstrained_sparse,
							   opt_ders_unconstrained, opt_costs, n_fixed);



	Polynomial p0_unC_sparse, p1_unC_sparse, p2_unC_sparse;
	p0_unC_sparse = *polys_unconstrained_sparse[0];
	p1_unC_sparse = *polys_unconstrained_sparse[1];
	p2_unC_sparse = *polys_unconstrained_sparse[2];

	// Print the resulting coefficients to the console
	eigen_matlab_dump(p0_unC_sparse);
	eigen_matlab_dump(p1_unC_sparse);
	eigen_matlab_dump(p2_unC_sparse);

	return PolyTrajNode::SamplePoly(p0_unC_sparse, 0, 0.75, 101);

}







int main(int argc, char* argv[]) {
	ros::init(argc, argv, "polyTraj_node");
	ros::NodeHandle nh;

	std::cout << "Poly traj :)" << std::endl;

	PolyTrajNode poly_traj_node(nh, "/waypoint_list", "/poly_samples");

	Eigen::MatrixXd samples = quad_traj_test();

	poly_traj_node.publishPath(samples);

	ros::spin();
}