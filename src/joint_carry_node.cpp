#include "ros/ros.h"
#include "joint_carry_controller.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_carry_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  
  int K_gmm;
  std::vector<double> Priors;
 


  if (!nh.getParam("input_topic_name", input_topic_name))   {
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
                                            input_topic_name,
                                        output_topic_name);
  if (!joint_carry_controller.Init()) {
    return -1;
  }
  else {
    joint_carry_controller.Run();
  }


  return 0;
}