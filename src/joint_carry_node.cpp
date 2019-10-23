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


  std::string topic_name_right_grasp_pose;
  std::string topic_name_left_grasp_pose;


  std::string topic_name_right_ds_vel;
  std::string topic_name_left_ds_vel;
  
 


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
                                              topic_name_right_grasp_pose,
                                              topic_name_left_grasp_pose,
                                              topic_name_right_ds_vel,
                                              topic_name_left_ds_vel);
  if (!joint_carry_controller.Init()) {
    return -1;
  }
  else {
    joint_carry_controller.Run();
  }


  return 0;
}