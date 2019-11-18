#include "ros/ros.h"
#include "joint_carry_controller.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_carry_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string topic_name_right_robot_pose;
  std::string topic_name_left_robot_pose;

  std::string topic_name_right_hand_command;
  std::string topic_name_left_hand_command;



  std::string topic_name_right_robot_command_vel;
  std::string topic_name_left_robot_command_vel;

  std::string topic_name_right_robot_command_orient;
  std::string topic_name_left_robot_command_orient;

  std::string topic_name_right_robot_command_wrench;
  std::string topic_name_left_robot_command_wrench;

  std::string topic_name_right_robot_command_damping;
  std::string topic_name_left_robot_command_damping;

  std::string topic_name_right_ft_sensor;
  std::string topic_name_left_ft_sensor;

  std::string topic_name_guard_pose;
  std::string topic_name_guard_twist;

  std::string topic_name_right_grasp_pose;
  std::string topic_name_left_grasp_pose;


  std::string topic_name_right_ds_vel;
  std::string topic_name_left_ds_vel;

  std::string topic_name_guard_desired_velocity;

  double hand_max_closure;
  double hand_grasp_trigger_dist;
  double hand_grasp_complete_dist;

  double guard_weight;
  double guard_ori_damp;


  double filter_time_constant;




  if (!nh.getParam("topic_name_right_robot_pose", topic_name_right_robot_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right robot pose. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_robot_pose", topic_name_left_robot_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left robot pose. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_hand_command", topic_name_right_hand_command))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right hand command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_hand_command", topic_name_left_hand_command))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left hand command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_robot_command_vel", topic_name_right_robot_command_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right robot vel command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_robot_command_vel", topic_name_left_robot_command_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left robot vel command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_robot_command_orient", topic_name_right_robot_command_orient))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right robot orient command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_robot_command_orient", topic_name_left_robot_command_orient))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left robot orient command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_robot_command_wrench", topic_name_right_robot_command_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right robot wrench command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_robot_command_wrench", topic_name_left_robot_command_wrench))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left robot wrench command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_robot_command_damping", topic_name_right_robot_command_damping))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right robot damping command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_robot_command_damping", topic_name_left_robot_command_damping))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left robot damping command. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_ft_sensor", topic_name_right_ft_sensor))   {
    ROS_ERROR("Couldn't retrieve the topic name for right FT sensor. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_ft_sensor", topic_name_left_ft_sensor))   {
    ROS_ERROR("Couldn't retrieve the topic name for left FT sensor. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_guard_pose", topic_name_guard_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the guard pose. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_guard_twist", topic_name_guard_twist))   {
    ROS_ERROR("Couldn't retrieve the topic name for the guard twist. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_grasp_pose", topic_name_right_grasp_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right grasp pose. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_grasp_pose", topic_name_left_grasp_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left grasp pose. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_right_ds_vel", topic_name_right_ds_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the right ds velocity. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_ds_vel", topic_name_left_ds_vel))   {
    ROS_ERROR("Couldn't retrieve the topic name for the left ds velocity. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_guard_desired_velocity", topic_name_guard_desired_velocity))   {
    ROS_ERROR("Couldn't retrieve the topic name for the desired velocity for the guard. ");
    // return -1;
  }


  if (!nh.getParam("hand_max_closure", hand_max_closure))   {
    ROS_ERROR("Couldn't retrieve the maximum closure for the QBhand. ");
    // return -1;
  }

  if (!nh.getParam("hand_grasp_trigger_dist", hand_grasp_trigger_dist))   {
    ROS_ERROR("Couldn't retrieve the trigger distance for the QBhand. ");
    // return -1;
  }

  if (!nh.getParam("hand_grasp_complete_dist", hand_grasp_complete_dist))   {
    ROS_ERROR("Couldn't retrieve the compelete grasp distance for the QBhand. ");
    // return -1;
  }

  if (!nh.getParam("guard_weight", guard_weight))   {
    ROS_ERROR("Couldn't retrieve the the weight of the guard. ");
    // return -1;
  }

  if (!nh.getParam("guard_ori_damp", guard_ori_damp))   {
    ROS_ERROR("Couldn't retrieve the the damping gain for the guard orientation. ");
    // return -1;
  }

  if (!nh.getParam("filter_time_constant", filter_time_constant))   {
    ROS_ERROR("Couldn't retrieve the the time constant for the filter. ");
    // return -1;
  }


  // if (!nh.getParam("K", K_gmm))   {
  //   ROS_ERROR("Couldn't retrieve the number of guassians. ");
  //   // return -1;
  // }


  // if (!nh.getParam("Priors", Priors))   {
  //   ROS_ERROR("Couldn't retrieve Priors. Maybe you need to use [] in case of k=1");
  //   // return -1;
  // }


  ROS_INFO("Starting the joint carry controller...");

  JointCarryController joint_carry_controller(nh, frequency,
      topic_name_right_robot_pose,
      topic_name_left_robot_pose,
      topic_name_right_hand_command,
      topic_name_left_hand_command,
      topic_name_right_robot_command_vel,
      topic_name_left_robot_command_vel,
      topic_name_right_robot_command_orient,
      topic_name_left_robot_command_orient,
      topic_name_right_robot_command_wrench,
      topic_name_left_robot_command_wrench,
      topic_name_right_robot_command_damping,
      topic_name_left_robot_command_damping,
      topic_name_right_ft_sensor,
      topic_name_left_ft_sensor,
      topic_name_guard_pose,
      topic_name_guard_twist,
      topic_name_right_grasp_pose,
      topic_name_left_grasp_pose,
      topic_name_right_ds_vel,
      topic_name_left_ds_vel,
      topic_name_guard_desired_velocity,
      hand_max_closure,
      hand_grasp_trigger_dist,
      hand_grasp_complete_dist,
      guard_weight,
      guard_ori_damp,
      filter_time_constant);
  if (!joint_carry_controller.Init()) {
    return -1;
  }
  else {
    joint_carry_controller.Run();
  }


  return 0;
}