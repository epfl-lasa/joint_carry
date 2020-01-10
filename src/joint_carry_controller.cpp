#include "joint_carry_controller.h"


//create pointer to self for CTRL + C catching
JointCarryController* JointCarryController::thisNodePtr = NULL;



JointCarryController::JointCarryController(ros::NodeHandle &n,
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
        double filter_time_constant)

	: nh_(n),
	  loop_rate_(frequency),
	  topic_name_right_robot_pose_(topic_name_right_robot_pose),
	  topic_name_left_robot_pose_(topic_name_left_robot_pose),
	  topic_name_right_hand_command_(topic_name_right_hand_command),
	  topic_name_left_hand_command_(topic_name_left_hand_command),
	  topic_name_right_robot_command_vel_(topic_name_right_robot_command_vel),
	  topic_name_left_robot_command_vel_(topic_name_left_robot_command_vel),
	  topic_name_right_robot_command_orient_(topic_name_right_robot_command_orient),
	  topic_name_left_robot_command_orient_(topic_name_left_robot_command_orient),
	  topic_name_right_robot_command_wrench_(topic_name_right_robot_command_wrench),
	  topic_name_left_robot_command_wrench_(topic_name_left_robot_command_wrench),
	  topic_name_right_robot_command_damping_(topic_name_right_robot_command_damping),
	  topic_name_left_robot_command_damping_(topic_name_left_robot_command_damping),
	  topic_name_right_ft_sensor_(topic_name_right_ft_sensor),
	  topic_name_left_ft_sensor_(topic_name_left_ft_sensor),
	  topic_name_guard_pose_(topic_name_guard_pose),
	  topic_name_guard_twist_(topic_name_guard_twist),
	  topic_name_right_grasp_pose_(topic_name_right_grasp_pose),
	  topic_name_left_grasp_pose_(topic_name_left_grasp_pose),
	  topic_name_right_ds_vel_(topic_name_right_ds_vel),
	  topic_name_left_ds_vel_(topic_name_left_ds_vel),
	  topic_name_guard_desired_velocity_(topic_name_guard_desired_velocity),
	  topic_name_guard_modulated_difference_(topic_name_guard_modulated_difference),
	  hand_max_closure_(hand_max_closure),
	  hand_grasp_trigger_dist_(hand_grasp_trigger_dist),
	  hand_grasp_complete_dist_(hand_grasp_complete_dist),
	  guard_weight_(guard_weight),
	  guard_ori_damp_(guard_ori_damp),
	  filter_time_constant_(filter_time_constant),
	  dt_(1 / frequency)
	  // scaling_factor_(1),
{

	ROS_INFO_STREAM("joint_carry_controller node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool JointCarryController::Init() {

	thisNodePtr = this;
	flagNodeStop_ = false;
	signal(SIGINT, JointCarryController::SigIntHandler);



	// topics to communicate with the two LWRs
	sub_right_robot_pose_ = nh_.subscribe(topic_name_right_robot_pose_ , 1,
	                                      &JointCarryController::UpdateRightRobotEEPose,
	                                      this, ros::TransportHints().reliable().tcpNoDelay());
	sub_left_robot_pose_ = nh_.subscribe(topic_name_left_robot_pose_ , 1,
	                                     &JointCarryController::UpdateLeftRobotEEPose,
	                                     this, ros::TransportHints().reliable().tcpNoDelay());


	sub_right_ft_sensor_ = nh_.subscribe(topic_name_right_ft_sensor_ , 1,
	                                     &JointCarryController::UpdateRightFTsensor,
	                                     this, ros::TransportHints().reliable().tcpNoDelay());
	sub_left_ft_sensor_ = nh_.subscribe(topic_name_left_ft_sensor_ , 1,
	                                    &JointCarryController::UpdateLeftFTsensor,
	                                    this, ros::TransportHints().reliable().tcpNoDelay());



	pub_right_robot_command_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_name_right_robot_command_vel_, 1);
	pub_left_robot_command_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_name_left_robot_command_vel_, 1);

	pub_right_robot_command_orient_ = nh_.advertise<geometry_msgs::Quaternion>(topic_name_right_robot_command_orient_, 1);
	pub_left_robot_command_orient_ = nh_.advertise<geometry_msgs::Quaternion>(topic_name_left_robot_command_orient_, 1);

	pub_right_robot_command_wrench_ = nh_.advertise<geometry_msgs::Wrench>(topic_name_right_robot_command_wrench_, 1);
	pub_left_robot_command_wrench_ = nh_.advertise<geometry_msgs::Wrench>(topic_name_left_robot_command_wrench_, 1);

	pub_right_robot_command_damping_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_name_right_robot_command_damping_, 1);
	pub_left_robot_command_damping_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_name_left_robot_command_damping_, 1);



	// pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);


	// topics to communicate with the two QBhands
	pub_right_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_right_hand_command_, 1);
	pub_left_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_left_hand_command_, 1);


	// topis to communicate with the Dynamical Systems
	pub_guard_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_guard_pose_, 1);
	pub_guard_twist_ = nh_.advertise<geometry_msgs::Twist>(topic_name_guard_twist_, 1);

	pub_right_grasp_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_right_grasp_pose_, 1);
	pub_left_grasp_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_left_grasp_pose_, 1);

	sub_right_ds_vel_ = nh_.subscribe(topic_name_right_ds_vel_, 1,
	                                  &JointCarryController::UpdateRightDSVelocity,
	                                  this, ros::TransportHints().reliable().tcpNoDelay());

	sub_left_ds_vel_ = nh_.subscribe(topic_name_left_ds_vel_, 1,
	                                 &JointCarryController::UpdateLeftDSVelocity,
	                                 this, ros::TransportHints().reliable().tcpNoDelay());

	sub_guard_desired_velocity_ = nh_.subscribe(topic_name_guard_desired_velocity_, 1,
	                              &JointCarryController::UpdateGuardDesiredVelocity,
	                              this, ros::TransportHints().reliable().tcpNoDelay());

	sub_guard_modulated_difference_ = nh_.subscribe(topic_name_guard_modulated_difference_, 1,
	                                  &JointCarryController::ObserveObstacleModulation,
	                                  this, ros::TransportHints().reliable().tcpNoDelay());





	pub_guard_disturbance_ = nh_.advertise<std_msgs::Float32>("guard_disturbance", 1);
	pub_tank_disturbance_ = nh_.advertise<std_msgs::Float32>("tank_disturbance", 1);




	std::vector<double> temp_vec;
	if (!nh_.getParam("guard_rot_stiffness_follower", temp_vec))   {
		ROS_ERROR("Couldn't retrieve the guard rotation sitffness in the follower condition. ");
		// return -1;
	} else {
		guard_rot_stiffness_follower_ << temp_vec[0] , temp_vec[1] , temp_vec[2];
		guard_rot_stiffness_ << temp_vec[0] , temp_vec[1] , temp_vec[2];
	}



	if (!nh_.getParam("guard_rot_stiffness_obstacle", temp_vec))   {
		ROS_ERROR("Couldn't retrieve the guard rotation sitffness in the obstacle condition. ");
		// return -1;
	} else {
		guard_rot_stiffness_obstacle_ << temp_vec[0] , temp_vec[1] , temp_vec[2];
	}


	if (!nh_.getParam("guard_rot_stiffness_disturbance", temp_vec))   {
		ROS_ERROR("Couldn't retrieve the guard rotation sitffness in the disturbance condition. ");
		// return -1;
	} else {
		guard_rot_stiffness_disturbance_ << temp_vec[0] , temp_vec[1] , temp_vec[2];

	}


	if (!nh_.getParam("modulation_threshold", modulation_threshold_))   {
		ROS_ERROR("Couldn't retrieve the modulation threshod for detecting presence of an obstacle. ");
		modulation_threshold_ = 0.2;
		// return -1;
	}


	




	// float32 closure[1];
	// closure[0] = 19000.0;

	right_hand_closure_.closure.clear();
	right_hand_closure_.closure.push_back(0.0);

	left_hand_closure_.closure.clear();
	left_hand_closure_.closure.push_back(0.0);

	flag_left_grasp_compelete_ = false;
	flag_right_grasp_compelete_ = false;



	right_ds_vel_.setZero();
	left_ds_vel_.setZero();

	right_ft_force_.setZero();
	left_ft_force_.setZero();

	right_ft_force_last_.setZero();
	left_ft_force_last_.setZero();

	guardPowHighFreq_ = 0;
	tank_disturbance_ = 0;

	guard_vel_.setZero();
	guard_last_position_.setZero();
	time_guard_last_position_ = ros::Time::now();

	filter_ratio_ = dt_ / (filter_time_constant_ + dt_);

	flag_obstacle_ = false;
	flag_dist_occured_ = false;
	last_time_dist_ = ros::Time::now();


	guard_desired_vel_.setZero();


	right_robot_orientation_.setIdentity();
	left_robot_orientation_.setIdentity();

	//preparing the wrench messages
	left_lwr_wrench_msg_.force.x = 0;
	left_lwr_wrench_msg_.force.y = 0;
	left_lwr_wrench_msg_.force.z = 0;
	left_lwr_wrench_msg_.torque.x = 0;
	left_lwr_wrench_msg_.torque.y = 0;
	left_lwr_wrench_msg_.torque.z = 0;

	right_lwr_wrench_msg_.force.x = 0;
	right_lwr_wrench_msg_.force.y = 0;
	right_lwr_wrench_msg_.force.z = 0;
	right_lwr_wrench_msg_.torque.x = 0;
	right_lwr_wrench_msg_.torque.y = 0;
	right_lwr_wrench_msg_.torque.z = 0;


	if (guard_ori_damp_ < 0) {
		ROS_WARN_STREAM("The dampinng gain for the guard is set to a negative value (" << guard_ori_damp_ << ") " <<
		                "Setting it to a default value (2)" );
		guard_ori_damp_ = 2;
	}



	if (nh_.ok() && !flagNodeStop_) { // Wait for poses being published
		ros::spinOnce();
		// starting the clock
		ros::Time::now();

		ROS_INFO_STREAM("Waiting 3 second for QBhands nodes");
		ros::Duration(3).sleep();
		pub_right_hand_command_.publish(right_hand_closure_);
		pub_left_hand_command_.publish(left_hand_closure_);

		ROS_INFO_STREAM("Setting kuka damping to 200");
		CommandDamping(200, 200, 200);


		ros::spinOnce();
		ros::Duration(1).sleep();

		wait_for_transformtaions();




		ROS_INFO("The controller is initialized.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		ShutDownController();

		return false;
	}






}






void JointCarryController::Run() {

	while (nh_.ok() && !flagNodeStop_) {


		// ROS_WARN_STREAM_THROTTLE(1, "flag: " << flag_right_grasp_compelete_ << " closure: " << right_hand_closure_.closure[0]);
		UpdateGuardCenterPose();

		UpdateRightQBHandControl();
		UpdateLeftQBHandControl();

		UpdateLwrsOrientatinControl();


		if (!flag_right_grasp_compelete_ || !flag_left_grasp_compelete_) {
			RightLwrReachToGrasp();
			LeftLwrReachToGrasp();
		}
		else {
			ROS_INFO_STREAM_THROTTLE(2, "Control the guard dynamics");
			UpdateGuardStiffness();
			ComputeGuardDesiredDynamics();
		}


		UpdateGuardWeightCancelation();

		if (1) {
			ROS_INFO_STREAM_THROTTLE(2, "---------------------------------------------\n" <<
			                         "Distance to right grasp point: " << right_palm_guard_distance_ << " (" <<  right_hand_closure_.closure[0] << ")" << "\n" <<
			                         "Distance to left grasp point:  " <<   left_palm_guard_distance_ << " (" <<  left_hand_closure_.closure[0] << ")"  << "\n" <<
			                         "DS vel for guard center: " << guard_desired_vel_.transpose()    << "\n" <<
			                         "Guard stiffness " << guard_rot_stiffness_.transpose() << "\n" <<
			                         "Flag for obstacle" << flag_obstacle_);
		}

		ComputeGuardDisturbance();

		ros::spinOnce();
		loop_rate_.sleep();
	}

	ShutDownController();

}


void JointCarryController::ComputeGuardDesiredDynamics() {


	// UpdateGuardCenterPose(); // doing this int main loop

	// for now let's just assume we want to keep the guard balanced

	Eigen::Quaterniond qd;
	Eigen::Quaterniond qr;

	qd = guard_desired_orientation_;
	qr.coeffs() << guard_pose_.bottomRows(4);
	qr.normalize();

	// ROS_INFO_STREAM_THROTTLE(2, "Real quat " << qr.coeffs() );
	// ROS_INFO_STREAM_THROTTLE(2, "desired quat " << qd.coeffs() );


	if (qd.coeffs().dot(qr.coeffs()) < 0.0) {
		qr.coeffs() << -qr.coeffs();
	}

	// qd = clamp_quat(qd , qr , 0.2);


	Eigen::Quaterniond q_err = qr * qd.inverse();
	Eigen::AngleAxisd err_axang(q_err);


	// in case of a disturbance, we forget about the linear motion and only try to keep the guard balanced.
	Vector3d guard_angular_err = err_axang.axis() * err_axang.angle();
	Vector3d guard_desired_angular_vel = -1 *  guard_rot_stiffness_.array() * guard_angular_err.array();


	Vector3d right_lwr_vel;
	Vector3d left_lwr_vel;

	if (!flag_dist_occured_) {

		right_lwr_vel = guard_desired_angular_vel.cross(guard_to_right_ee_in_world_) + guard_desired_vel_;
		left_lwr_vel  = guard_desired_angular_vel.cross(guard_to_left_ee_in_world_) + guard_desired_vel_;

	} else {

		right_lwr_vel = guard_desired_angular_vel.cross(guard_to_right_ee_in_world_);
		left_lwr_vel  = guard_desired_angular_vel.cross(guard_to_left_ee_in_world_);

	}

	// ROS_INFO_STREAM_THROTTLE(2, "Desired angular velocity for the guard center " << guard_desired_angular_vel );



	double vel_limit = 0.2;

	if (right_lwr_vel.norm() > vel_limit) {
		right_lwr_vel *= (vel_limit / right_lwr_vel.norm() );
	}

	if (left_lwr_vel.norm() > vel_limit) {
		left_lwr_vel *= (vel_limit / left_lwr_vel.norm() );
	}

	ROS_INFO_STREAM_THROTTLE(2, "DS vel for right lwr: (" <<  right_lwr_vel.norm() << ")" << right_lwr_vel.transpose() );
	ROS_INFO_STREAM_THROTTLE(2, "DS vel for left lwr:  (" <<  left_lwr_vel.norm()  <<  ")" << left_lwr_vel.transpose() );


	// sending the DS vels to the robot
	geometry_msgs::Twist twist_msg;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;

	twist_msg.linear.x = right_lwr_vel(0);
	twist_msg.linear.y = right_lwr_vel(1);
	twist_msg.linear.z = right_lwr_vel(2);


	pub_right_robot_command_vel_.publish(twist_msg);


	twist_msg.linear.x = left_lwr_vel(0);
	twist_msg.linear.y = left_lwr_vel(1);
	twist_msg.linear.z = left_lwr_vel(2);

	pub_left_robot_command_vel_.publish(twist_msg);



}

void JointCarryController::UpdateGuardStiffness() {

	if (flag_dist_occured_) {
		guard_rot_stiffness_ = (1 - filter_ratio_) * guard_rot_stiffness_
		                       + filter_ratio_ * guard_rot_stiffness_disturbance_;

	} else if (flag_obstacle_) {
		guard_rot_stiffness_ =  (1 - filter_ratio_) * guard_rot_stiffness_
		                        + filter_ratio_ * guard_rot_stiffness_obstacle_;

	} else {
		guard_rot_stiffness_ =  (1 - filter_ratio_) * guard_rot_stiffness_
		                        + filter_ratio_ * guard_rot_stiffness_follower_;

	}




}


void JointCarryController::UpdateGuardWeightCancelation() {

	left_lwr_wrench_msg_.force.z = 0;
	right_lwr_wrench_msg_.force.z = 0;

	if (flag_right_grasp_compelete_) {
		right_lwr_wrench_msg_.force.z = guard_weight_ / 2;
	}

	if (flag_left_grasp_compelete_) {
		left_lwr_wrench_msg_.force.z = guard_weight_ / 2;
	}

	pub_right_robot_command_wrench_.publish(right_lwr_wrench_msg_);
	pub_left_robot_command_wrench_.publish(left_lwr_wrench_msg_);


}


void JointCarryController::UpdateRightQBHandControl() {

	update_right_grasp_point();

	if (!flag_right_grasp_compelete_) {

		right_hand_closure_.closure.clear();

		if (right_palm_guard_distance_ > hand_grasp_complete_dist_) {
			right_hand_closure_.closure.push_back(distance_to_colusre(right_palm_guard_distance_));
		}
		else {
			right_hand_closure_.closure.push_back(hand_max_closure_);
			flag_right_grasp_compelete_ = true;
		}

		pub_right_hand_command_.publish(right_hand_closure_);

		// ROS_INFO_STREAM_THROTTLE(2, "Distance to right grasp point: " << right_palm_guard_distance_
		//                          << " hand_clousre: " << right_hand_closure_.closure[0]);

	}
	else if (right_palm_guard_distance_ > 2 * hand_grasp_trigger_dist_) {

		right_hand_closure_.closure.clear();
		right_hand_closure_.closure.push_back(0);
		pub_right_hand_command_.publish(right_hand_closure_);

		flag_right_grasp_compelete_ = false;
		ROS_WARN("object realsed from the right hand.");
	}



}

void JointCarryController::UpdateLeftQBHandControl() {

	update_left_grasp_point();

	if (!flag_left_grasp_compelete_) {

		left_hand_closure_.closure.clear();

		if (left_palm_guard_distance_ > hand_grasp_complete_dist_) {
			left_hand_closure_.closure.push_back(distance_to_colusre(left_palm_guard_distance_));
		}
		else {
			left_hand_closure_.closure.push_back(hand_max_closure_);
			flag_left_grasp_compelete_ = true;
		}

		pub_left_hand_command_.publish(left_hand_closure_);

		// ROS_INFO_STREAM_THROTTLE(2, "Distance to left grasp point: " << left_palm_guard_distance_
		//                          << " hand_clousre: " << left_hand_closure_.closure[0]);

	}
	else if (left_palm_guard_distance_ > 2 * hand_grasp_trigger_dist_) {

		left_hand_closure_.closure.clear();

		left_hand_closure_.closure.push_back(0);
		pub_left_hand_command_.publish(left_hand_closure_);

		flag_left_grasp_compelete_ = false;
		ROS_WARN("object realsed from the left hand.");
	}

}



void JointCarryController::UpdateLwrsOrientatinControl() {

	Eigen::Quaterniond qd;
	geometry_msgs::Quaternion quat_msg;


	// orientation control of the right lwr
	qd.coeffs() << right_grasp_pose_.bottomRows(4) / right_grasp_pose_.bottomRows(4).norm();
	qd = clamp_quat(qd , right_robot_orientation_ , 0.2);

	quat_msg.x = qd.x();
	quat_msg.y = qd.y();
	quat_msg.z = qd.z();
	quat_msg.w = qd.w();

	pub_right_robot_command_orient_.publish(quat_msg);


	// orientation control of the left lwr
	qd.coeffs() << left_grasp_pose_.bottomRows(4) / left_grasp_pose_.bottomRows(4).norm();
	qd = clamp_quat(qd , left_robot_orientation_ , 0.2);

	quat_msg.x = qd.x();
	quat_msg.y = qd.y();
	quat_msg.z = qd.z();
	quat_msg.w = qd.w();

	pub_left_robot_command_orient_.publish(quat_msg);
}





void JointCarryController::RightLwrReachToGrasp() {


	// sending the DS vels to the robot
	geometry_msgs::Twist twist_msg;

	// might not be neccessary to assign zeroes
	twist_msg.linear.x = 0;
	twist_msg.linear.y = 0;
	twist_msg.linear.z = 0;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;


	if (!flag_right_grasp_compelete_) {
		twist_msg.linear.x = right_ds_vel_(0);
		twist_msg.linear.y = right_ds_vel_(1);
		twist_msg.linear.z = right_ds_vel_(2);
	}





	pub_right_robot_command_vel_.publish(twist_msg);

}


void JointCarryController::LeftLwrReachToGrasp() {

	// sending the DS vels to the robot
	geometry_msgs::Twist twist_msg;

	// might not be neccessary to assign zeroes
	twist_msg.linear.x = 0;
	twist_msg.linear.y = 0;
	twist_msg.linear.z = 0;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;

	if (!flag_left_grasp_compelete_) {
		twist_msg.linear.x = left_ds_vel_(0);
		twist_msg.linear.y = left_ds_vel_(1);
		twist_msg.linear.z = left_ds_vel_(2);
	}


	pub_left_robot_command_vel_.publish(twist_msg);

}


void JointCarryController::CommandDamping(double Dx, double Dy, double Dz) {

	std_msgs::Float64MultiArray msg;
	msg.data.resize(3);
	msg.data[0] = Dx;
	msg.data[1] = Dy;
	msg.data[2] = Dz;


	pub_right_robot_command_damping_.publish(msg);
	pub_left_robot_command_damping_.publish(msg);


}






// ########################################################
// ################ Topic Callbacks #######################
// ########################################################


void JointCarryController::UpdateRightDSVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	right_ds_vel_(0) = msg->twist.linear.x;
	right_ds_vel_(1) = msg->twist.linear.y;
	right_ds_vel_(2) = msg->twist.linear.z;

	// ROS_INFO_STREAM_THROTTLE(1, "Recieved vel from right ds: " <<
	// 	right_ds_vel_(0) << "\t" <<
	// 	right_ds_vel_(1) << "\t" <<
	// 	right_ds_vel_(2) );

}

void JointCarryController::UpdateLeftDSVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	left_ds_vel_(0) = msg->twist.linear.x;
	left_ds_vel_(1) = msg->twist.linear.y;
	left_ds_vel_(2) = msg->twist.linear.z;

	// ROS_INFO_STREAM_THROTTLE(1, "Recieved vel from right ds: " <<
	// 	left_ds_vel_(0) << "\t" <<
	// 	left_ds_vel_(1) << "\t" <<
	// 	left_ds_vel_(2) );

}




void JointCarryController::UpdateGuardDesiredVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	guard_desired_vel_(0) = (1 - filter_ratio_) * guard_desired_vel_(0) + filter_ratio_ * msg->twist.linear.x;
	guard_desired_vel_(1) = (1 - filter_ratio_) * guard_desired_vel_(1) + filter_ratio_ * msg->twist.linear.y;
	guard_desired_vel_(2) = (1 - filter_ratio_) * guard_desired_vel_(2) + filter_ratio_ * msg->twist.linear.z;


}

void JointCarryController::ObserveObstacleModulation(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	Vector3d vel_diff;

	vel_diff(0) = msg->twist.linear.x;
	vel_diff(1) = msg->twist.linear.y;
	vel_diff(2) = msg->twist.linear.z;


	if(vel_diff.norm() > modulation_threshold_){
		flag_obstacle_ = true;
	}
	else{
		flag_obstacle_ = false;
	}
};



void JointCarryController::UpdateRightRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg) {

	right_robot_position_(0) = msg->position.x;
	right_robot_position_(1) = msg->position.y;
	right_robot_position_(2) = msg->position.z;

	right_robot_orientation_.x() = msg->orientation.x;
	right_robot_orientation_.y() = msg->orientation.y;
	right_robot_orientation_.z() = msg->orientation.z;
	right_robot_orientation_.w() = msg->orientation.w;
	right_robot_orientation_.normalize();

}


void JointCarryController::UpdateLeftRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg) {

	left_robot_position_(0) = msg->position.x;
	left_robot_position_(1) = msg->position.y;
	left_robot_position_(2) = msg->position.z;

	left_robot_orientation_.x() = msg->orientation.x;
	left_robot_orientation_.y() = msg->orientation.y;
	left_robot_orientation_.z() = msg->orientation.z;
	left_robot_orientation_.w() = msg->orientation.w;
	left_robot_orientation_.normalize();

}


void JointCarryController::UpdateRightFTsensor(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	right_ft_force_last_ = right_ft_force_;

	right_ft_force_(0) = msg->wrench.force.x;
	right_ft_force_(1) = msg->wrench.force.x;
	right_ft_force_(2) = msg->wrench.force.x;



}


void JointCarryController::UpdateLeftFTsensor(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

	left_ft_force_last_ = left_ft_force_;

	left_ft_force_(0) = msg->wrench.force.x;
	left_ft_force_(1) = msg->wrench.force.x;
	left_ft_force_(2) = msg->wrench.force.x;

}


void JointCarryController::ComputeGuardDisturbance() {
	double diff_norm = (right_ft_force_ - right_ft_force_last_).squaredNorm() + (left_ft_force_ - left_ft_force_last_).squaredNorm();

	guardPowHighFreq_ = 0.99 * guardPowHighFreq_ + 0.01 * diff_norm;

	guardPowHighFreq_ = (guardPowHighFreq_ > 5) ? 5 : guardPowHighFreq_;

	// ROS_INFO_STREAM_THROTTLE(1, "High freq : " << guardPowHighFreq_);

	std_msgs::Float32 msg;
	msg.data = guardPowHighFreq_;
	pub_guard_disturbance_.publish(msg);


	if (guardPowHighFreq_ > 1) {
		tank_disturbance_ += dt_ * guardPowHighFreq_;
	}
	else {
		tank_disturbance_ = 0;
	}

	msg.data = tank_disturbance_;
	pub_tank_disturbance_.publish(msg);

	if (tank_disturbance_ > 1) {
		ROS_WARN_STREAM_THROTTLE(1, "Detecting unsual force patterns !!!");
		if (!flag_dist_occured_) {
			flag_dist_occured_ = true;
			CommandDamping(0, 0, 300);
		}
		last_time_dist_ = ros::Time::now();
	}
	else if (flag_dist_occured_ && (ros::Time::now() - last_time_dist_).toSec() > 5) {
		ROS_WARN_STREAM_THROTTLE(1, "Unusal force patterns disappeared !!!");
		flag_dist_occured_ = false;
		CommandDamping(200, 200, 200);

	}



}

// ########################################################
// ################ Handles TFs ###########################
// ########################################################

void JointCarryController::wait_for_transformtaions() {

	right_grasp_pose_.setZero();
	right_grasp_pose_(6) = 1; // quat.w = 1
	left_grasp_pose_.setZero();
	left_grasp_pose_(6) = 1; // quat.w = 1

	while ((!update_right_grasp_point() || !update_left_grasp_point() || !UpdateGuardCenterPose()) && !flagNodeStop_) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for the TFs");
		sleep(.5);
	}

}


bool JointCarryController::UpdateGuardCenterPose()
{
	tf::StampedTransform transform_guard;

	try {
		tf_listener_.lookupTransform("mocap_world", "guard",
		                             ros::Time(0), transform_guard);
		guard_pose_(0) = transform_guard.getOrigin().x();
		guard_pose_(1) = transform_guard.getOrigin().y();
		guard_pose_(2) = transform_guard.getOrigin().z();
		guard_pose_(3) = transform_guard.getRotation().x();
		guard_pose_(4) = transform_guard.getRotation().y();
		guard_pose_(5) = transform_guard.getRotation().z();
		guard_pose_(6) = transform_guard.getRotation().w();

		geometry_msgs::Pose pose_msg;

		pose_msg.position.x = guard_pose_(0);
		pose_msg.position.y = guard_pose_(1);
		pose_msg.position.z = guard_pose_(2);
		pose_msg.orientation.x = guard_pose_(3);
		pose_msg.orientation.y = guard_pose_(4);
		pose_msg.orientation.z = guard_pose_(5);
		pose_msg.orientation.w = guard_pose_(6);

		pub_guard_pose_.publish(pose_msg);

		ros::Time time_now = ros::Time::now();
		double time_diff = (time_now - time_guard_last_position_).toSec();


		guard_vel_ = (1 - filter_ratio_) * guard_vel_ + filter_ratio_ * (guard_pose_.head(3) - guard_last_position_) / time_diff;

		time_guard_last_position_ = ros::Time::now();
		guard_last_position_ = guard_pose_.head(3);


		geometry_msgs::Twist msg_guard_twist;
		msg_guard_twist.linear.x = guard_vel_(0);
		msg_guard_twist.linear.y = guard_vel_(1);
		msg_guard_twist.linear.z = guard_vel_(2);
		pub_guard_twist_.publish(msg_guard_twist);

	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from mocap_world to guard" );
		return false;
	}

	tf::StampedTransform transform;
	tf::Vector3 vec_tmp;

	try {
		tf_listener_.lookupTransform("guard", "right_lwr_7_link",
		                             ros::Time(0), transform);
		vec_tmp = transform_guard.getBasis() * transform.getOrigin();

		guard_to_right_ee_in_world_(0) = vec_tmp.getX();
		guard_to_right_ee_in_world_(1) = vec_tmp.getY();
		guard_to_right_ee_in_world_(2) = vec_tmp.getZ();

		// ROS_INFO_STREAM_THROTTLE(2, "guard_to_right_ee_in_world_:" << guard_to_right_ee_in_world_.transpose());
	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from guard to right_lwr_7_link" );
		return false;
	}


	try {
		tf_listener_.lookupTransform("guard", "left_lwr_7_link",
		                             ros::Time(0), transform);
		vec_tmp = transform_guard.getBasis() * transform.getOrigin();

		guard_to_left_ee_in_world_(0) = vec_tmp.getX();
		guard_to_left_ee_in_world_(1) = vec_tmp.getY();
		guard_to_left_ee_in_world_(2) = vec_tmp.getZ();

		// ROS_INFO_STREAM_THROTTLE(2, "guard_to_left_ee_in_world_:" << guard_to_left_ee_in_world_.transpose());

	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from guard to left_lwr_7_link" );
		return false;
	}

	try {
		tf_listener_.lookupTransform("mocap_world", "guard_desired_pose",
		                             ros::Time(0), transform);
		guard_desired_orientation_.x() = transform.getRotation().x();
		guard_desired_orientation_.y() = transform.getRotation().y();
		guard_desired_orientation_.z() = transform.getRotation().z();
		guard_desired_orientation_.w() = transform.getRotation().w();

	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from mocap_world to guard_desired_pose" );
		return false;
	}




	return true;

}





bool JointCarryController::update_right_grasp_point()
{
	tf::StampedTransform transform;

	try {
		tf_listener_.lookupTransform("right_lwr_base_link", "right_grasp",
		                             ros::Time(0), transform);
		right_grasp_pose_(0) = transform.getOrigin().x();
		right_grasp_pose_(1) = transform.getOrigin().y();
		right_grasp_pose_(2) = transform.getOrigin().z();
		right_grasp_pose_(3) = transform.getRotation().x();
		right_grasp_pose_(4) = transform.getRotation().y();
		right_grasp_pose_(5) = transform.getRotation().z();
		right_grasp_pose_(6) = transform.getRotation().w();

		geometry_msgs::Pose pose_msg;

		pose_msg.position.x = right_grasp_pose_(0);
		pose_msg.position.y = right_grasp_pose_(1);
		pose_msg.position.z = right_grasp_pose_(2);

		pose_msg.orientation.x = right_grasp_pose_(3);
		pose_msg.orientation.y = right_grasp_pose_(4);
		pose_msg.orientation.z = right_grasp_pose_(5);
		pose_msg.orientation.w = right_grasp_pose_(6);

		pub_right_grasp_pose_.publish(pose_msg);

	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from right_lwr_base_link to /right_grasp" );
		return false;
	}

	try {
		tf_listener_.lookupTransform("right_palm", "guard_right_corner",
		                             ros::Time(0), transform);
		right_palm_guard_distance_ = transform.getOrigin().length();
	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from /right_palm to /guard_right_corner" );
		return false;
	}



	return true;

}

bool JointCarryController::update_left_grasp_point()
{
	tf::StampedTransform transform;

	try {
		tf_listener_.lookupTransform("left_lwr_base_link", "left_grasp",
		                             ros::Time(0), transform);
		left_grasp_pose_(0) = transform.getOrigin().x();
		left_grasp_pose_(1) = transform.getOrigin().y();
		left_grasp_pose_(2) = transform.getOrigin().z();
		left_grasp_pose_(3) = transform.getRotation().x();
		left_grasp_pose_(4) = transform.getRotation().y();
		left_grasp_pose_(5) = transform.getRotation().z();
		left_grasp_pose_(6) = transform.getRotation().w();

		geometry_msgs::Pose pose_msg;

		pose_msg.position.x = left_grasp_pose_(0);
		pose_msg.position.y = left_grasp_pose_(1);
		pose_msg.position.z = left_grasp_pose_(2);

		pose_msg.orientation.x = left_grasp_pose_(3);
		pose_msg.orientation.y = left_grasp_pose_(4);
		pose_msg.orientation.z = left_grasp_pose_(5);
		pose_msg.orientation.w = left_grasp_pose_(6);

		pub_left_grasp_pose_.publish(pose_msg);



	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from left_lwr_base_link to /left_grasp" );
		return false;
	}


	try {
		tf_listener_.lookupTransform("left_palm", "guard_left_corner",
		                             ros::Time(0), transform);
		left_palm_guard_distance_ = transform.getOrigin().length();
	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from /left_palm to /guard_left_corner" );
		return false;
	}

	return true;
}


// ########################################################
// ################ Util functins #########################
// ########################################################


int JointCarryController::distance_to_colusre(double distance_to_goal) {

	int closure =  (int) hand_max_closure_ / (hand_grasp_complete_dist_ - hand_grasp_trigger_dist_)
	               * (distance_to_goal - hand_grasp_trigger_dist_);

	return (closure < 0 ) ? 0 : closure;

}




Eigen::Quaterniond JointCarryController::clamp_quat(Eigen::Quaterniond qd, Eigen::Quaterniond qr , double max_angle ) {

	if (qd.coeffs().dot(qr.coeffs()) < 0.0) {
		qd.coeffs() << -qd.coeffs();
	}

	Eigen::Quaterniond q_diff = qd * qr.inverse();
	Eigen::AngleAxisd diff_axang(q_diff);

	diff_axang.angle() = (diff_axang.angle() > max_angle) ?  max_angle : diff_axang.angle();
	diff_axang.angle() = (diff_axang.angle() < -max_angle) ? -max_angle : diff_axang.angle();

	Eigen::Quaterniond diff_clamped(diff_axang);

	qd = diff_clamped * qr;
	qd.normalize();

	return qd;

}



// ########################################################
// ################ shutdown      #########################
// ########################################################

void JointCarryController::SigIntHandler(int sig) {

	ROS_INFO("Catched the ctrl C");
	thisNodePtr->flagNodeStop_ = true;

}


void JointCarryController::ShutDownController() {
	ROS_INFO("Shutting down the controller");
	ros::spinOnce();
	loop_rate_.sleep();


	// Openning the qbhands:

	right_hand_closure_.closure.clear();
	right_hand_closure_.closure.push_back(0.0);
	pub_right_hand_command_.publish(right_hand_closure_);

	left_hand_closure_.closure.clear();
	left_hand_closure_.closure.push_back(0.0);
	pub_left_hand_command_.publish(left_hand_closure_);
	ros::spinOnce();
	loop_rate_.sleep();


	// Setting kuka gains to zero
	CommandDamping(0, 0, 0);

	ros::spinOnce();
	loop_rate_.sleep();
	ros::shutdown();

}











