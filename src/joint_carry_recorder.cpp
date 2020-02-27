#include "joint_carry_recorder.h"

joint_carry_recorder::joint_carry_recorder(ros::NodeHandle& n, double frequency) : nh_(n), loop_rate_(frequency)
{
  ROS_INFO_STREAM(
    "[joint_carry_recorder_node] node is created at: " << nh_.getNamespace());  // << " with freq: " << frequency << "Hz");
}

bool joint_carry_recorder::Init()
{


  // initialize the files for saving data
  std::string recPath;

  if (!nh_.getParam("recording_path", recPath))
  {
    ROS_ERROR("[joint_carry_recorder_node] Couldn't retrieve the recording path.");
    return false;
  }
  mkdir(recPath.c_str(), 0777);
  recPath += "demonstrations/";
  mkdir(recPath.c_str(), 0777);

  // Creating a subdirectory for a specific interaction based on time
  time_t rawtime;
  tm* timeinfo;
  char buffer[80];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
  recPath += std::string(buffer);
  mkdir(recPath.c_str(), 0777);

  ROS_INFO_STREAM("[joint_carry_recorder_node] Recording to :" << recPath.c_str());




  // std::ofstream file_left_passive_ds_linear_damping_;
  // std::ofstream file_left_lwr_ee_pose_;
  // std::ofstream file_left_lwr_ee_vel_;


  file_left_passive_ds_command_vel_.open(recPath + "/left_passive_ds_command_vel.txt");
  file_left_passive_ds_command_vel_ << "Time \t x \t y \t z \n";

  // file_left_passive_ds_linear_damping_.open(recPath + "/left_passive_ds_linear_damping.txt");
  // file_left_passive_ds_linear_damping_ << "Time \t x \t y \t z \n";

  // file_left_lwr_ee_pose_.open(recPath + "/left_lwr_ee_pose.txt");
  // file_left_lwr_ee_pose_ << "Time \t x \t y \t z \t qx \t qy \t qz \t qw \n";

  // file_left_lwr_ee_vel_.open(recPath + "/left_lwr_ee_vel.txt");
  // file_left_lwr_ee_vel_ << "Time \t x \t y \t z \t rx \t ry \t rz \n";

  // std::string recPath_joints_state = recPath + "/joints_state.txt";
  // file_joints_state_.open(recPath_joints_state);
  // file_joints_state_ << "Time \t q1 \t q2 \t q3 \t q4 \t q5 \t q6 \t q7 \n";

  // std::string recPath_target_state = recPath + "/target_state.txt";
  // file_target_state_.open(recPath_target_state);
  // file_target_state_ << "Time \t x \t y \t z \t qx \t qy \t qz \t qw \n";

ros::Duration(1).sleep();


  // initialize the ros communication

  sub_left_passive_ds_command_vel_ = nh_.subscribe("/left_lwr/joint_controllers/passive_ds_command_vel", 1,
                                     &joint_carry_recorder::Callback_left_passive_ds_command_vel,
                                     this, ros::TransportHints().reliable().tcpNoDelay());

  // sub_left_passive_ds_linear_damping_ = nh_.subscribe("/left_lwr/joint_controllers/passive_ds_linear_damping", 1,
  //                                       &joint_carry_recorder::Callback_left_passive_ds_linear_damping,
  //                                       this, ros::TransportHints().reliable().tcpNoDelay());

  // sub_left_lwr_ee_pose_ = nh_.subscribe("/left_lwr/ee_pose", 1,
  //                                       &joint_carry_recorder::Callback_left_lwr_ee_pose,
  //                                       this, ros::TransportHints().reliable().tcpNoDelay());

  // sub_left_lwr_ee_vel_ = nh_.subscribe("/left_lwr/ee_vel", 1,
  //                                      &joint_carry_recorder::Callback_left_lwr_ee_vel,
  //                                      this, ros::TransportHints().reliable().tcpNoDelay());






  if (nh_.ok())
  {
    time_start_ = ros::Time::now();
    ros::spinOnce();
    ROS_INFO("[joint_carry_recorder_node] node is initialized.");
    return true;
  }
  else
  {
    ROS_ERROR("[joint_carry_recorder_node] The ros node has a problem.");
    return false;
  }
}

void joint_carry_recorder::Run()
{
  while (nh_.ok())
  {
    ROS_INFO_STREAM_THROTTLE(2 , "Recording ....");
    ros::spinOnce();
    loop_rate_.sleep();
  }
}



/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Callbacks //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
void joint_carry_recorder::Callback_left_passive_ds_command_vel(const geometry_msgs::Twist::ConstPtr& msg)
{

    file_left_passive_ds_command_vel_ << (ros::Time::now() - time_start_).toSec() << "\t" <<
                                        msg->linear.x << "\t" << msg->linear.y << "\t" << msg->linear.z << "\n";



}
// void joint_carry_recorder::Callback_left_passive_ds_linear_damping(const std_msgs::Float64MultiArray::ConstPtr& msg) {

//   file_left_passive_ds_linear_damping_ << (ros::Time::now() - time_start_).toSec() << "\t" <<
//                                     msg->data[0] << "\t" << msg->data[1] << "\t" << msg->data[2] << "\n";
// }

// void joint_carry_recorder::Callback_left_lwr_ee_pose(const geometry_msgs::Pose::ConstPtr& msg) {
//   file_left_lwr_ee_pose_ << (ros::Time::now() - time_start_).toSec() << "\t" <<
//                          msg->position.x << "\t" << msg->position.y << "\t" << msg->position.z << "\t" << 
//                          msg->orientation.x << "\t" << msg->orientation.y << "\t" << 
//                          msg->orientation.z << "\t" << msg->orientation.w << "\n";

// }
// void joint_carry_recorder::Callback_left_lwr_ee_vel(const geometry_msgs::Twist::ConstPtr& msg) {
//     file_left_lwr_ee_vel_ << (ros::Time::now() - time_start_).toSec() << "\t" <<
//                          msg->linear.x << "\t" << msg->linear.y << "\t" << msg->linear.z << "\t" << 
//                          msg->angular.x << "\t" << msg->angular.y << "\t" << msg->angular.z << "\n";

// }



joint_carry_recorder::~joint_carry_recorder()
{
  file_left_passive_ds_command_vel_.close();
  // file_left_passive_ds_linear_damping_.close();
  // file_left_lwr_ee_pose_.close();
  // file_left_lwr_ee_vel_.close();
  // file_joints_state_.close();
  // file_target_state_.close();
}