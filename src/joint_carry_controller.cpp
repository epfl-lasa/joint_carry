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
				                     std::string topic_name_right_grasp_pose,
                      				 std::string topic_name_left_grasp_pose,
                                     std::string output_topic_name)
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
      topic_name_right_grasp_pose_(topic_name_right_grasp_pose),
      topic_name_left_grasp_pose_(topic_name_right_grasp_pose),

	  output_topic_name_(output_topic_name),
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


	pub_right_robot_command_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_name_right_robot_command_vel_,1);
	pub_left_robot_command_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_name_left_robot_command_vel_,1);

	pub_right_robot_command_orient_ = nh_.advertise<geometry_msgs::Quaternion>(topic_name_right_robot_command_orient_,1);
	pub_left_robot_command_orient_ = nh_.advertise<geometry_msgs::Quaternion>(topic_name_left_robot_command_orient_,1);


	// pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);


	// topics to communicate with the two QBhands
	pub_right_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_right_hand_command_,1);
	pub_left_hand_command_ = nh_.advertise<qb_interface::handRef>(topic_name_left_hand_command_,1);


	// topis to communicate with the Dynamical Systems
	pub_right_grasp_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_right_grasp_pose_,1);
	pub_left_grasp_pose_ = nh_.advertise<geometry_msgs::Pose>(topic_name_left_grasp_pose_,1);

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

		// pub_right_hand_command_.publish(right_hand_closure_);
		// pub_left_hand_command_.publish(left_hand_closure_);


		// pub_right_hand_command_.publish()

	right_hand_closure_.closure.clear();
	right_hand_closure_.closure.push_back(19000.0);

	left_hand_closure_.closure.clear();
	left_hand_closure_.closure.push_back(19000.0);


	update_grasp_tfs();
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
	
 



 ROS_INFO_STREAM_THROTTLE(1, "the right grasp point is at:" << 
 	right_grasp_pose_(0) << "\t" <<
 	right_grasp_pose_(1) << "\t" <<
 	right_grasp_pose_(2) << "\t" <<
 	right_grasp_pose_(3) << "\t" <<
 	right_grasp_pose_(4) << "\t" <<
 	right_grasp_pose_(5) << "\t" <<
 	right_grasp_pose_(6) );


  ROS_INFO_STREAM_THROTTLE(1, "the left grasp point is at:" << 
 	left_grasp_pose_(0) << "\t" <<
 	left_grasp_pose_(1) << "\t" <<
 	left_grasp_pose_(2) << "\t" <<
 	left_grasp_pose_(3) << "\t" <<
 	left_grasp_pose_(4) << "\t" <<
 	left_grasp_pose_(5) << "\t" <<
 	left_grasp_pose_(6) );




  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;

   pub_right_robot_command_vel_.publish(twist_msg);



   geometry_msgs::Quaternion quat_msg;

   quat_msg.x = right_grasp_pose_(3);
   quat_msg.y = right_grasp_pose_(4);
   quat_msg.z = right_grasp_pose_(5);
   quat_msg.w = right_grasp_pose_(6);

   pub_right_robot_command_orient_.publish(quat_msg);


}




void JointCarryController::wait_for_transformtaions(){

  right_grasp_pose_.setZero();
  right_grasp_pose_(6) = 1; // quat.w = 1
  left_grasp_pose_.setZero();
  left_grasp_pose_(6) = 1; // quat.w = 1

  while (!update_grasp_tfs()) {
	ROS_WARN_STREAM_THROTTLE(1, "Waiting for the TF of the right grasp point: ");
    sleep(1);
  }

 }




bool JointCarryController::update_grasp_tfs()
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
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: /right_grasp" );
    return false;
  }

  try {
    tf_listener_.lookupTransform("right_lwr_base_link", "left_grasp",
                             ros::Time(0), transform);
      left_grasp_pose_(0) = transform.getOrigin().x();
      left_grasp_pose_(1) = transform.getOrigin().y();
      left_grasp_pose_(2) = transform.getOrigin().z();
      left_grasp_pose_(3) = transform.getRotation().x();
      left_grasp_pose_(4) = transform.getRotation().y();
      left_grasp_pose_(5) = transform.getRotation().z();
      left_grasp_pose_(6) = transform.getRotation().w();
  }
  catch (tf::TransformException ex) {
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF: /left_grasp" );
    return false;
  }

  return true;
}