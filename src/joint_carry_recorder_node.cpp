#include "joint_carry_recorder.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_carry_recorder_node");
  ros::NodeHandle nh;
  double frequency = 100;

  ROS_INFO("[joint_carry_recorder_node] Starting the recording node ...");

  joint_carry_recorder demo_recorder(nh, frequency);

  if (!demo_recorder.Init())
  {
    return -1;
  }
  else
  {
    demo_recorder.Run();
  }

  return 0;
}