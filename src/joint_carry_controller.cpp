#include "joint_carry_controller.h"





JointCarryController::JointCarryController(ros::NodeHandle &n,
                                     double frequency,
                                     std::string input_topic_name,
                                     std::string output_topic_name)
	: nh_(n),
	  loop_rate_(frequency),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  dt_(1 / frequency)
	  // scaling_factor_(1),
	   {

	ROS_INFO_STREAM("joint_carry_controller node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool JointCarryController::Init() {

	// sub_real_pose_ = nh_.subscribe( input_topic_name_ , 1000,
	//                                 &JointCarryController::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
	// pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name_, 1);




	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
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

		ROS_WARN_THROTTLE(1, "Doing nothing ....");

		loop_rate_.sleep();
	}
}

// void JointCarryController::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

// 	msg_real_pose_ = *msg;

// 	real_pose_(0) = msg_real_pose_.position.x;
// 	real_pose_(1) = msg_real_pose_.position.y;
// 	real_pose_(2) = msg_real_pose_.position.z;

// 	// double qtx = msg_real_pose_.orientation.x;
// 	// double qty = msg_real_pose_.orientation.y;
// 	// double qtz = msg_real_pose_.orientation.z;
// 	// double qtw = msg_real_pose_.orientation.w;

// 	// tf::Quaternion q(qtx, qty, qtz, qtw);
// 	// tf::Matrix3x3 m(q);
// 	// double roll, pitch, yaw;
// 	// m.getRPY(roll, pitch, yaw);

// 	// real_pose_(3?) = roll;
// 	// real_pose_(4?) = pitch;
// 	// real_pose_(5?) = yaw;

// }

