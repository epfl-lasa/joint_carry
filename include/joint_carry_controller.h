#ifndef __JOINT_CONTROLLER_H__
#define __JOINT_CONTROLLER_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
// #include "geometry_msgs/Twist.h"
//#include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/PointStamped.h"

#include <vector>


// #include <qb_interface/cubeRef.h>
#include <qb_interface/handRef.h>

// #include <qb_interface/cubeEq_Preset.h>
// #include <qb_interface/cubePos.h>
// #include <qb_interface/handPos.h>

// #include <qb_interface/cubeCurrent.h>
// #include <qb_interface/handCurrent.h>


#include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>

// #include <tf_conversions/tf_eigen.h>


// #include <Eigen/Dense>
// #include <Eigen3/Eigen/Dense>

// #include "eigen3/Eigen/Core"
// #include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

using namespace Eigen;

typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
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

	ros::Publisher pub_right_hand_command_;
	ros::Publisher pub_left_hand_command_;

	ros::Publisher pub_right_robot_command_vel_;
	ros::Publisher pub_left_robot_command_vel_ ;

	ros::Publisher pub_right_robot_command_orient_ ;
	ros::Publisher pub_left_robot_command_orient_ ;

	ros::Publisher pub_right_grasp_pose_;		
	ros::Publisher pub_left_grasp_pose_;	

	// ros::Publisher pub_desired_twist_;
	// ros::Publisher pub_desired_twist_filtered_;
	// ros::Publisher pub_target_;
	// ros::Publisher pub_DesiredPath_;

	std::string topic_name_right_robot_pose_;
	std::string topic_name_left_robot_pose_;
	std::string topic_name_right_hand_command_;
    std::string topic_name_left_hand_command_;
    std::string topic_name_right_robot_command_vel_;
	std::string topic_name_left_robot_command_vel_;
	std::string topic_name_right_robot_command_orient_;
	std::string topic_name_left_robot_command_orient_;

  	std::string topic_name_right_grasp_pose_;
  	std::string topic_name_left_grasp_pose_;

	std::string output_topic_name_;

	// geometry_msgs::Pose msg_real_pose_;
	// geometry_msgs::TwistStamped msg_desired_velocity_;
	// geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

	// nav_msgs::Path msg_DesiredPath_;
	// int MAX_FRAME = 200;

	qb_interface::handRef right_hand_closure_;
	qb_interface::handRef left_hand_closure_;


	Vector3d right_robot_position_;
	Vector3d left_robot_position_;
	// MathLib::Vector real_pose_;
	// MathLib::Vector target_pose_;
	// MathLib::Vector target_offset_;


	// MathLib::Vector desired_velocity_;
	// MathLib::Vector desired_velocity_filtered_;

	// double scaling_factor_;
	// double ds_vel_limit_;



tf::TransformListener tf_listener_;

  Vector7d right_grasp_pose_;
  Vector7d left_grasp_pose_;



public:
	JointCarryController(ros::NodeHandle &n,
	                  double frequency,
	                  std::string topic_name_right_robot_pose,
	                  std::string topic_name_left_robot_pose,
	                  std::string topic_name_right_hand_command,
	                  std::string topic_name_left_hand_command,
                      std::string topic_name_right_robot_command_vel,
                      std::string topic_name_left_robot_command_vel,
                      std::string topic_name_right_robot_command_orient,
                      std::string topic_name_left_robot_command_orient,
                      std::string topic_name_right_grasp_pose,
                      std::string topic_name_left_grasp_pose,
	                  std::string output_topic_name);

	bool Init();

	void Run();

private:



	void UpdateRightRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg);
	void UpdateLeftRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg);


	void UpdateRightRobotTask();


	void wait_for_transformtaions();
	bool update_grasp_tfs();




};


#endif