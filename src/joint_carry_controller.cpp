#include "joint_carry_controller.h"





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
        std::string topic_name_guard_pose,
        std::string topic_name_right_grasp_pose,
        std::string topic_name_left_grasp_pose,
        std::string topic_name_right_ds_vel,
        std::string topic_name_left_ds_vel,
        double hand_max_closure,
        double hand_grasp_trigger_dist,
        double hand_grasp_complete_dist)

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
	  topic_name_guard_pose_(topic_name_guard_pose),
	  topic_name_right_grasp_pose_(topic_name_right_grasp_pose),
	  topic_name_left_grasp_pose_(topic_name_left_grasp_pose),
	  topic_name_right_ds_vel_(topic_name_right_ds_vel),
	  topic_name_left_ds_vel_(topic_name_left_ds_vel),
	  hand_max_closure_(hand_max_closure),
	  hand_grasp_trigger_dist_(hand_grasp_trigger_dist),
	  hand_grasp_complete_dist_(hand_grasp_complete_dist),
	  dt_(1 / frequency)
	  // scaling_factor_(1),
{

	ROS_INFO_STREAM("joint_carry_controller node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool JointCarryController::Init() {


	// topics to communicate with the two LWRs
	sub_right_robot_pose_ = nh_.subscribe(topic_name_right_robot_pose_ , 1000,
	                                      &JointCarryController::UpdateRightRobotEEPose,
	                                      this, ros::TransportHints().reliable().tcpNoDelay());
	sub_left_robot_pose_ = nh_.subscribe(topic_name_left_robot_pose_ , 1000,
	                                     &JointCarryController::UpdateLeftRobotEEPose,
	                                     this, ros::TransportHints().reliable().tcpNoDelay());




	pub_right_robot_command_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_name_right_robot_command_vel_, 1);
	pub_left_robot_command_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_name_left_robot_command_vel_, 1);

	pub_right_robot_command_orient_ = nh_.advertise<geometry_msgs::Quaternion>(topic_name_right_robot_command_orient_, 1);
	pub_left_robot_command_orient_ = nh_.advertise<geometry_msgs::Quaternion>(topic_name_left_robot_command_orient_, 1);


	// pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);


	// topics to communicate with the two QBhands
	pub_right_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_right_hand_command_, 1);
	pub_left_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_left_hand_command_, 1);


	// topis to communicate with the Dynamical Systems
	pub_guard_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_guard_pose_, 1);

	pub_right_grasp_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_right_grasp_pose_, 1);
	pub_left_grasp_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_left_grasp_pose_, 1);

	sub_right_ds_vel_ = nh_.subscribe(topic_name_right_ds_vel_, 1000,
	                                  &JointCarryController::UpdateRightDSVelocity,
	                                  this, ros::TransportHints().reliable().tcpNoDelay());

	sub_left_ds_vel_ = nh_.subscribe(topic_name_left_ds_vel_, 1000,
	                                 &JointCarryController::UpdateLeftDSVelocity,
	                                 this, ros::TransportHints().reliable().tcpNoDelay());

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


	right_robot_orientation_.setIdentity();
	left_robot_orientation_.setIdentity();


	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();

		ROS_INFO_STREAM("Waiting 3 second for QBhands nodes");
		ros::Duration(3).sleep();
		pub_right_hand_command_.publish(right_hand_closure_);
		pub_left_hand_command_.publish(left_hand_closure_);
		ros::spinOnce();
		ros::Duration(1).sleep();

		wait_for_transformtaions();



		ROS_INFO("The controller is initialized.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}






}






void JointCarryController::Run() {

	while (nh_.ok()) {

		UpdateRightQBHandControl();
		UpdateLeftQBHandControl();


		UpdateLwrsOrientatinControl();





		if (!flag_right_grasp_compelete_ && !flag_left_grasp_compelete_) {

			RightLwrReachToGrasp();
			LeftLwrReachToGrasp();
		}
		else
		{
			ComputeGuardDesiredDynamics();

		}



		ros::spinOnce();
		loop_rate_.sleep();
	}
}


void JointCarryController::ComputeGuardDesiredDynamics() {

	// for now let's just assume we want to keep the guard balanced

	Eigen::Quaterniond qd, qr;

	qd = guard_desired_orientation_;
	qr.coeffs() << guard_pose_.bottomRows(4);

	if (qd.coeffs().dot(qr.coeffs()) < 0.0) {
		qr.coeffs() << -qr.coeffs();
	}

	Eigen::Quaterniond q_err = qr * qd.inverse();
	Eigen::AngleAxisd err_axang(q_err);


	Vector3d guard_desired_angular_vel = -1 * err_axang.axis() * err_axang.angle();



	Vector3d right_lwr_vel = guard_desired_angular_vel.cross(guard_to_right_ee_in_world_);
	Vector3d left_lwr_vel  = guard_desired_angular_vel.cross(guard_to_left_ee_in_world_);



	// sending the DS vels to the robot
	geometry_msgs::Twist twist_msg;
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;

	twist_msg.linear.x = right_lwr_vel(0);
	twist_msg.linear.y = right_lwr_vel(1);
	twist_msg.linear.z = right_lwr_vel(2);


	// pub_right_robot_command_vel_.publish(twist_msg);


	twist_msg.linear.x = left_lwr_vel(0);
	twist_msg.linear.y = left_lwr_vel(1);
	twist_msg.linear.z = left_lwr_vel(2);

	// pub_left_robot_command_vel_.publish(twist_msg);


}


void JointCarryController::UpdateRightQBHandControl() {

	update_right_grasp_point();

	double distance_to_goal = (right_robot_position_ - right_grasp_pose_.head(3)).norm();

	if (!flag_right_grasp_compelete_) {
		right_hand_closure_.closure.clear();

		if (distance_to_goal > hand_grasp_complete_dist_) {
			right_hand_closure_.closure.push_back(distance_to_colusre(distance_to_goal));
		}
		else {
			right_hand_closure_.closure.push_back(hand_max_closure_);
			flag_right_grasp_compelete_ = true;
		}

		ROS_INFO_STREAM_THROTTLE(1, "Distance to right grasp point: " << distance_to_goal
		                         << " hand_clousre: " << right_hand_closure_.closure[0]);

		pub_right_hand_command_.publish(right_hand_closure_);

	}

	if (flag_right_grasp_compelete_ && distance_to_goal > hand_grasp_trigger_dist_) {
		ROS_WARN("object realsed from the right hand.");
		right_hand_closure_.closure.clear();
		right_hand_closure_.closure.push_back(0);
		pub_right_hand_command_.publish(right_hand_closure_);
		flag_right_grasp_compelete_ = false;
	}

}

void JointCarryController::UpdateLeftQBHandControl() {

	update_left_grasp_point();

	double distance_to_goal = (left_robot_position_ - left_grasp_pose_.head(3)).norm();

	if (!flag_left_grasp_compelete_) {
		left_hand_closure_.closure.clear();

		if (distance_to_goal > hand_grasp_complete_dist_) {
			left_hand_closure_.closure.push_back(distance_to_colusre(distance_to_goal));
		}
		else {
			left_hand_closure_.closure.push_back(hand_max_closure_);
			flag_left_grasp_compelete_ = true;
		}

		ROS_INFO_STREAM_THROTTLE(1, "Distance to left grasp point: " << distance_to_goal
		                         << " hand_clousre: " << left_hand_closure_.closure[0]);

		pub_left_hand_command_.publish(left_hand_closure_);

	}

	if (flag_left_grasp_compelete_ && distance_to_goal > hand_grasp_trigger_dist_) {
		ROS_WARN("object realsed from the left hand.");
		left_hand_closure_.closure.clear();
		left_hand_closure_.closure.push_back(0);
		pub_left_hand_command_.publish(left_hand_closure_);
		flag_left_grasp_compelete_ = false;
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
	twist_msg.linear.x = right_ds_vel_(0);
	twist_msg.linear.y = right_ds_vel_(1);
	twist_msg.linear.z = right_ds_vel_(2);
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;

	pub_right_robot_command_vel_.publish(twist_msg);

}


void JointCarryController::LeftLwrReachToGrasp() {

	// sending the DS vels to the robot
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = left_ds_vel_(0);
	twist_msg.linear.y = left_ds_vel_(1);
	twist_msg.linear.z = left_ds_vel_(2);
	twist_msg.angular.x = 0;
	twist_msg.angular.y = 0;
	twist_msg.angular.z = 0;

	pub_left_robot_command_vel_.publish(twist_msg);

}






// ########################################################
// ################ Topic Calbacks#########################
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


// ########################################################
// ################ Handles TFs ###########################
// ########################################################

void JointCarryController::wait_for_transformtaions() {

	right_grasp_pose_.setZero();
	right_grasp_pose_(6) = 1; // quat.w = 1
	left_grasp_pose_.setZero();
	left_grasp_pose_(6) = 1; // quat.w = 1

	while (!update_right_grasp_point() || !update_left_grasp_point() || !UpdateGuardCenterPose()) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for the TFs");
		sleep(1);
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
		vec_tmp = transform_guard * transform.getOrigin();

		guard_to_right_ee_in_world_(0) = vec_tmp.getX();
		guard_to_right_ee_in_world_(1) = vec_tmp.getY();
		guard_to_right_ee_in_world_(2) = vec_tmp.getZ();
	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from guard to right_lwr_7_link" );
		return false;
	}


	try {
		tf_listener_.lookupTransform("guard", "left_lwr_7_link",
		                             ros::Time(0), transform);
		vec_tmp = transform_guard * transform.getOrigin();

		guard_to_left_ee_in_world_(0) = vec_tmp.getX();
		guard_to_left_ee_in_world_(1) = vec_tmp.getY();
		guard_to_left_ee_in_world_(2) = vec_tmp.getZ();
	}
	catch (tf::TransformException ex) {
		ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: from guard to left_lwr_7_link" );
		return false;
	}

	try {
		tf_listener_.lookupTransform("mocap_world", "guard_desired_pose",
		                             ros::Time(0), transform);
		guard_desired_orientation_.x() = transform_guard.getRotation().x();
		guard_desired_orientation_.y() = transform_guard.getRotation().y();
		guard_desired_orientation_.z() = transform_guard.getRotation().z();
		guard_desired_orientation_.w() = transform_guard.getRotation().w();
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

	qd = diff_clamped * right_robot_orientation_;
	qd.normalize();

	return qd;

}













