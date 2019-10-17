#ifndef __JOINT_CONTROLLER_H__
#define __JOINT_CONTROLLER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
//#include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/PointStamped.h"

#include <vector>


// #include <Eigen/Dense>
// #include <Eigen3/Eigen/Dense>

// #include "eigen3/Eigen/Core"
// #include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

using namespace Eigen;

using namespace Eigen;

typedef Matrix<double, 3, 1> Vector3d;
// typedef Matrix<double, 6, 1> Vector6d;
// typedef Matrix<double, 6, 6> Matrix6d;





class JointCarryController {


private:

	// DS variables

	// int K_gmm_;
	// int dim_;
	// std::vector<double> Priors_;
	// std::vector<double> Mu_;
	// std::vector<double> Sigma_;
	// std::vector<double> attractor_;
	double dt_;

	// double max_desired_vel_;

	// // Filter variables
	// std::unique_ptr<CDDynamics> CCDyn_filter_;

	// double Wn_;
	// MathLib::Vector accLimits_;
	// MathLib::Vector velLimits_;


	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	ros::Subscriber sub_right_robot_pose_;
	ros::Subscriber sub_left_robot_pose_;
	// ros::Publisher pub_desired_twist_;
	// ros::Publisher pub_desired_twist_filtered_;
	// ros::Publisher pub_target_;
	// ros::Publisher pub_DesiredPath_;

	std::string topic_name_right_robot_pose_;
	std::string topic_name_left_robot_pose_;
	std::string output_topic_name_;

	// geometry_msgs::Pose msg_real_pose_;
	// geometry_msgs::TwistStamped msg_desired_velocity_;
	// geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

	// nav_msgs::Path msg_DesiredPath_;
	// int MAX_FRAME = 200;



	Vector3d right_robot_position_;
	Vector3d left_robot_position_;
	// MathLib::Vector real_pose_;
	// MathLib::Vector target_pose_;
	// MathLib::Vector target_offset_;


	// MathLib::Vector desired_velocity_;
	// MathLib::Vector desired_velocity_filtered_;

	// double scaling_factor_;
	// double ds_vel_limit_;



public:
	JointCarryController(ros::NodeHandle &n,
	                  double frequency,
	                  std::string topic_name_right_robot_pose,
	                  std::string topic_name_left_robot_pose,
	                  std::string output_topic_name);

	bool Init();

	void Run();

private:



	void UpdateRightRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg);
	void UpdateLeftRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg);



};


#endif