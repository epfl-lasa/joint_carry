#include "joint_carry_controller.h"





JointCarryController::JointCarryController(ros::NodeHandle &n,
                                     double frequency,
                                     std::string topic_name_right_robot_pose,
                                     std::string topic_name_left_robot_pose,
                                     std::string topic_name_right_hand_command,
                                     std::string topic_name_left_hand_command,
                                     std::string output_topic_name)
	: nh_(n),
	  loop_rate_(frequency),
	  topic_name_right_robot_pose_(topic_name_right_robot_pose),
	  topic_name_left_robot_pose_(topic_name_left_robot_pose),
	  topic_name_right_hand_command_(topic_name_right_hand_command),
	  topic_name_left_hand_command_(topic_name_left_hand_command),
	  output_topic_name_(output_topic_name),
	  dt_(1 / frequency)
	  // scaling_factor_(1),
	   {

	ROS_INFO_STREAM("joint_carry_controller node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool JointCarryController::Init() {

	sub_right_robot_pose_ = nh_.subscribe(topic_name_right_robot_pose_ , 1000,
	                                &JointCarryController::UpdateRightRobotEEPose, 
	                                this, ros::TransportHints().reliable().tcpNoDelay());
	sub_left_robot_pose_ = nh_.subscribe(topic_name_left_robot_pose_ , 1000,
	                                &JointCarryController::UpdateLeftRobotEEPose, 
	                                this, ros::TransportHints().reliable().tcpNoDelay());
	// pub_desired_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name_, 1);

	pub_right_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_right_hand_command_,1);
	pub_left_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_left_hand_command_,1);


	// float32 closure[1]; 
	// closure[0] = 19000.0;

	right_hand_closure_.closure.clear();
	right_hand_closure_.closure.push_back(0.0);

	left_hand_closure_.closure.clear();
	left_hand_closure_.closure.push_back(0.0);

	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The controller is initialized.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

	wait_for_transformtaions();

}






void JointCarryController::Run() {

	while (nh_.ok()) {

		ROS_WARN_THROTTLE(1, "Updating the hand commands ....");

		pub_right_hand_command_.publish(right_hand_closure_);
		pub_left_hand_command_.publish(left_hand_closure_);


		// pub_right_hand_command_.publish()

	right_hand_closure_.closure.clear();
	right_hand_closure_.closure.push_back(19000.0);

	left_hand_closure_.closure.clear();
	left_hand_closure_.closure.push_back(19000.0);


   UpdateRightRobotTask();


		loop_rate_.sleep();
	}
}

void JointCarryController::UpdateRightRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg) {

	// msg_real_pose_ = *msg;

	right_robot_position_(0) = msg->position.x;
	right_robot_position_(1) = msg->position.y;
	right_robot_position_(2) = msg->position.z;

	// double qtx = msg_real_pose_.orientation.x;
	// double qty = msg_real_pose_.orientation.y;
	// double qtz = msg_real_pose_.orientation.z;
	// double qtw = msg_real_pose_.orientation.w;

}


void JointCarryController::UpdateLeftRobotEEPose(const geometry_msgs::Pose::ConstPtr& msg) {

	// msg_real_pose_ = *msg;

	left_robot_position_(0) = msg->position.x;
	left_robot_position_(1) = msg->position.y;
	left_robot_position_(2) = msg->position.z;

	// double qtx = msg_real_pose_.orientation.x;
	// double qty = msg_real_pose_.orientation.y;
	// double qtz = msg_real_pose_.orientation.z;
	// double qtw = msg_real_pose_.orientation.w;

}


void JointCarryController::UpdateRightRobotTask(){
	
 

 get_rotation_matrix(rotation_grasp_right_, tf_listener_,
                              "right_lwr_base_link", "grasp_right");

 ROS_INFO_STREAM_THROTTLE(1, "goal is at:" << rotation_grasp_right_(0,0) << "\t" <<
 	rotation_grasp_right_(1,0));
}


void JointCarryController::wait_for_transformtaions(){

  tf::TransformListener listener;
  rotation_grasp_right_.setZero();
  rotation_grasp_left_.setZero();

  while (!get_rotation_matrix(rotation_grasp_right_, listener,
                              "right_lwr_base_link", "grasp_right")) {
	ROS_WARN_STREAM_THROTTLE(1, "Waiting for the TF of the right grasp point: ");
    sleep(1);
  }

    while (!get_rotation_matrix(rotation_grasp_left_, listener,
                              "right_lwr_base_link", "grasp_left")) {
	ROS_WARN_STREAM_THROTTLE(1, "Waiting for the TF of the left grasp point: ");
    sleep(1);
  }

}


bool JointCarryController::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}
