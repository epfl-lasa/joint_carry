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
  std::string output_topic_name;
  
  int K_gmm;
  std::vector<double> Priors;
 


  if (!nh.getParam("topic_name_right_robot_pose", topic_name_right_robot_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("topic_name_left_robot_pose", topic_name_left_robot_pose))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("output_topic_name", output_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
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
                                              output_topic_name);
  if (!joint_carry_controller.Init()) {
    return -1;
  }
  else {
    joint_carry_controller.Run();
  }


  return 0;
}