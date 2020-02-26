#ifndef __JOINT_CONTROLLER_H__
#define __JOINT_CONTROLLER_H__

#include <signal.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


#include <vector>

#include <qb_interface/handRef.h>



#include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>

// #include <tf_conversions/tf_eigen.h>


// #include <Eigen/Dense>
// #include <Eigen3/Eigen/Dense>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"



using namespace Eigen;

typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
// typedef Matrix<double, 6, 6> Matrix6d;





class JointCarryController {


private:

	static JointCarryController* thisNodePtr;

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

	ros::Publisher pub_right_robot_command_orient_;
	ros::Publisher pub_left_robot_command_orient_;

	ros::Publisher pub_right_robot_command_wrench_;
	ros::Publisher pub_left_robot_command_wrench_;

	ros::Publisher pub_right_robot_command_damping_;
	ros::Publisher pub_left_robot_command_damping_;

	ros::Publisher pub_guard_pose_;
	ros::Publisher pub_guard_twist_;

	ros::Publisher pub_right_grasp_pose_;
	ros::Publisher pub_left_grasp_pose_;

	ros::Subscriber sub_right_ds_vel_;
	ros::Subscriber sub_left_ds_vel_;

	ros::Subscriber sub_right_ft_sensor_;
	ros::Subscriber sub_left_ft_sensor_;

	ros::Subscriber sub_guard_desired_velocity_;
	ros::Subscriber sub_guard_modulated_difference_;

	ros::Publisher pub_guard_disturbance_;
	ros::Publisher pub_tank_disturbance_;

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
	std::string topic_name_right_robot_command_wrench_;
	std::string topic_name_left_robot_command_wrench_;
	std::string topic_name_right_robot_command_damping_;
	std::string topic_name_left_robot_command_damping_;
	std::string topic_name_right_ft_sensor_;
	std::string topic_name_left_ft_sensor_;

	std::string topic_name_right_ds_vel_;
	std::string topic_name_left_ds_vel_;

	std::string topic_name_guard_desired_velocity_;
	std::string topic_name_guard_modulated_difference_;


	std::string topic_name_guard_pose_;
	std::string topic_name_guard_twist_;

	std::string topic_name_right_grasp_pose_;
	std::string topic_name_left_grasp_pose_;




	// geometry_msgs::Pose msg_real_pose_;
	// geometry_msgs::TwistStamped msg_desired_velocity_;
	// geometry_msgs::TwistStamped msg_desired_velocity_filtered_;

	// nav_msgs::Path msg_DesiredPath_;
	// int MAX_FRAME = 200;

	qb_interface::handRef right_hand_closure_;
	qb_interface::handRef left_hand_closure_;


	Vector3d right_robot_position_;
	Vector3d left_robot_position_;

	Quaterniond right_robot_orientation_;
	Quaterniond left_robot_orientation_;

	// MathLib::Vector real_pose_;
	// MathLib::Vector target_pose_;
	// MathLib::Vector target_offset_;


	// MathLib::Vector desired_velocity_;
	// MathLib::Vector desired_velocity_filtered_;

	// double scaling_factor_;
	// double ds_vel_limit_;



	tf::TransformListener tf_listener_;

	Vector7d guard_pose_; // pose of the guard in the world frame
	Vector3d guard_vel_; // in the world frame as well

	ros::Time time_guard_last_position_;
	Vector3d guard_last_position_;


	Vector7d right_grasp_pose_;
	Vector7d left_grasp_pose_;

	Vector3d guard_to_right_ee_in_world_;
	Vector3d guard_to_left_ee_in_world_;

	Quaterniond guard_desired_orientation_;
	Vector3d guard_desired_velocity_;



	Vector3d right_ds_vel_;
	Vector3d left_ds_vel_;
	Vector3d guard_desired_vel_;


	double hand_max_closure_;
	double hand_grasp_trigger_dist_;
	double hand_grasp_complete_dist_;

	double right_palm_guard_distance_;
	double left_palm_guard_distance_;

	bool flag_left_grasp_compelete_;
	bool flag_right_grasp_compelete_;

	double guard_weight_;
	geometry_msgs::Wrench left_lwr_wrench_msg_;
	geometry_msgs::Wrench right_lwr_wrench_msg_;

	double guard_ori_damp_;

	double filter_time_constant_;
	double filter_ratio_;

	Vector3d right_ft_force_;
	Vector3d left_ft_force_;

	Vector3d right_ft_force_last_;
	Vector3d left_ft_force_last_;

	double guardPowHighFreq_;
	double tank_disturbance_;

	ros::Time last_time_dist_;
	bool flag_dist_occured_;

	bool flag_obstacle_;


	bool flagNodeStop_;


	Vector3d guard_rot_stiffness_follower_;
	Vector3d guard_rot_stiffness_obstacle_;
	Vector3d guard_rot_stiffness_disturbance_;

	Vector3d guard_rot_stiffness_;

	double modulation_threshold_;


	double disturbance_min_pow_;
	double disturbance_max_pow_;
	double disturbance_energy_threshold_;


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
	                     std::string topic_name_right_robot_command_wrench,
	                     std::string topic_name_left_robot_command_wrench,
	                     std::string topic_name_right_robot_command_damping,
	                     std::string topic_name_left_robot_command_damping,
	                     std::string topic_name_right_ft_sensor,
	                     std::string topic_name_left_ft_sensor,
	                     std::string topic_name_guard_pose,
	                     std::string topic_name_guard_twist,
	                     std::string topic_name_right_grasp_pose,
	                     std::string topic_name_left_grasp_pose,
	                     std::string topic_name_right_ds_vel,
	                     std::string topic_name_left_ds_vel,
	                     std::string topic_name_guard_desired_velocity,
	                     std::string topic_name_guard_modulated_difference,
	                     double hand_max_closure,
	                     double hand_grasp_trigger_dist,
	                     double hand_grasp_complete_dist,
	                     double guard_weight,
	                     double guard_ori_damp,
	                     double filter_time_constant);

	bool Init();

	void Run();

private:



	void UpdateRightRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg);
	void UpdateLeftRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg);


	void RightLwrReachToGrasp();
	void LeftLwrReachToGrasp();

	void UpdateRightQBHandControl();
	void UpdateLeftQBHandControl();


	void wait_for_transformtaions();
	bool update_right_grasp_point();
	bool update_left_grasp_point();
	bool UpdateGuardCenterPose();

	void UpdateLwrsOrientatinControl();

	void ComputeGuardDesiredDynamics();



	void UpdateRightDSVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void UpdateLeftDSVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);

	void UpdateGuardDesiredVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void ObserveObstacleModulation(const geometry_msgs::TwistStamped::ConstPtr& msg);


	void UpdateGuardStiffness();
	void UpdateGuardWeightCancelation();


	int distance_to_colusre(double distance_to_goal);  // int since qbhand in TICK mode


	Eigen::Quaterniond clamp_quat(Eigen::Quaterniond qd, Eigen::Quaterniond qr , double max_angle );

	void CommandDamping(double Dx, double Dy, double Dz);



	void UpdateRightFTsensor(const geometry_msgs::WrenchStamped::ConstPtr& msg);
	void UpdateLeftFTsensor(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	void ComputeGuardDisturbance();

	static void SigIntHandler(int sig);


	void ShutDownController();






};


#endif