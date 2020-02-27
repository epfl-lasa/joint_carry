#ifndef __JOINT_CARRY_RECORDER_H__
#define __JOINT_CARRY_RECORDER_H__

#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"


// #include "geometry_msgs/PoseStamped.h"
// #include "sensor_msgs/JointState.h"



class joint_carry_recorder
{
private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;


  ros::Subscriber sub_left_passive_ds_command_vel_;
  // ros::Subscriber sub_left_passive_ds_linear_damping_;
  // ros::Subscriber sub_left_lwr_ee_pose_;
  // ros::Subscriber sub_left_lwr_ee_vel_;

  // ros::Subscriber sub_right_passive_ds_command_vel_;
  // ros::Subscriber sub_right_passive_ds_linear_damping_;
  // ros::Subscriber sub_right_lwr_ee_pose_;
  // ros::Subscriber sub_right_lwr_ee_vel_;

  // ros::Subscriber sub_left_ft_;
  // ros::Subscriber sub_right_ft_;

  // ros::Subscriber sub_obstacle_pose_;
  // ros::Subscriber sub_modulated_velocity_;
  // ros::Subscriber sub_modulated_difference_;

  // ros::Subscriber sub_guard_pose_in_world_;
  // ros::Subscriber sub_guard_twist_in_world_;

  // ros::Subscriber sub_guard_disturbance_;
  // ros::Subscriber sub_tank_disturbance_;

  // ros::Subscriber sub_task_ds_velocity_;




  std::ofstream file_left_passive_ds_command_vel_;
  // std::ofstream file_left_passive_ds_linear_damping_;
  // std::ofstream file_left_lwr_ee_pose_;
  // std::ofstream file_left_lwr_ee_vel_;

  // std::ofstream file_right_passive_ds_command_vel_;
  // std::ofstream file_right_passive_ds_linear_damping_;
  // std::ofstream file_right_lwr_ee_pose_;
  // std::ofstream file_right_lwr_ee_vel_;

  // std::ofstream file_left_ft_;
  // std::ofstream file_right_ft_;

  // std::ofstream file_obstacle_pose_;
  // std::ofstream file_modulated_velocity_;
  // std::ofstream file_modulated_difference_;

  // std::ofstream file_guard_pose_in_world_;
  // std::ofstream file_guard_twist_in_world_;

  // std::ofstream file_guard_disturbance_;
  // std::ofstream file_tank_disturbance_;

  std::ofstream file_task_ds_velocity_;





//   // std::ofstream file_ee_state_;
//   // std::ofstream file_joints_state_;
//   // std::ofstream file_target_state_;

  ros::Time time_start_;

public:
  joint_carry_recorder(ros::NodeHandle& n, double frequency);

  ~joint_carry_recorder();

  bool Init();
  void Run();

private:

  void Callback_left_passive_ds_command_vel(const geometry_msgs::Twist::ConstPtr& msg);
  // void Callback_left_passive_ds_linear_damping(const std_msgs::Float64MultiArray::ConstPtr& msg);
  // void Callback_left_lwr_ee_pose(const geometry_msgs::Pose::ConstPtr& msg);
  // void Callback_left_lwr_ee_vel(const geometry_msgs::Twist::ConstPtr& msg);

//   void Callback_ee_state(const geometry_msgs::Pose::ConstPtr& msg);
//   void Callback_joints_state(const sensor_msgs::JointState::ConstPtr& msg);
//   void Callback_target_state(const geometry_msgs::PoseStamped::ConstPtr& msg);



};

#endif
