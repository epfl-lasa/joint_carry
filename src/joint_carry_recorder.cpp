#include "joint_carry_recorder.h"

joint_carry_recorder::joint_carry_recorder(ros::NodeHandle& n,double frequency) : nh_(n),loop_rate_(frequency)
{
  ROS_INFO_STREAM(
    "[joint_carry_recorder_node] node is created at: " << nh_.getNamespace());  // << " with freq: " << frequency << "Hz");
}

bool joint_carry_recorder::Init()
{


  // initialize the files for saving data
  std::string recPath;

  if (!nh_.getParam("recording_path",recPath))
  {
    ROS_ERROR("[joint_carry_recorder_node] Couldn't retrieve the recording path.");
    return false;
  }
  mkdir(recPath.c_str(),0777);
  recPath += "demonstrations/";
  mkdir(recPath.c_str(),0777);

  // Creating a subdirectory for a specific interaction based on time
  time_t rawtime;
  tm* timeinfo;
  char buffer[80];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
  recPath += std::string(buffer);
  mkdir(recPath.c_str(),0777);

  ROS_INFO_STREAM("[joint_carry_recorder_node] Recording to :" << recPath.c_str());




  // std::ofstream file_left_passive_ds_linear_damping_;
  // std::ofstream file_left_lwr_ee_pose_;
  // std::ofstream file_left_lwr_ee_vel_;


  file_left_passive_ds_command_vel_.open(recPath + "/left_passive_ds_command_vel.csv");
  file_left_passive_ds_command_vel_ << "Time,x,y,z \n";

  file_left_passive_ds_linear_damping_.open(recPath + "/left_passive_ds_linear_damping.csv");
  file_left_passive_ds_linear_damping_ << "Time,x,y,z \n";

  file_left_lwr_ee_pose_.open(recPath + "/left_lwr_ee_pose.csv");
  file_left_lwr_ee_pose_ << "Time,x,y,z,qx,qy,qz,qw \n";

  file_left_lwr_ee_vel_.open(recPath + "/left_lwr_ee_vel.csv");
  file_left_lwr_ee_vel_ << "Time,x,y,z,rx,ry,rz \n";



  file_right_passive_ds_command_vel_.open(recPath + "/right_passive_ds_command_vel.csv");
  file_right_passive_ds_command_vel_ << "Time,x,y,z \n";

  file_right_passive_ds_linear_damping_.open(recPath + "/right_passive_ds_linear_damping.csv");
  file_right_passive_ds_linear_damping_ << "Time,x,y,z \n";

  file_right_lwr_ee_pose_.open(recPath + "/right_lwr_ee_pose.csv");
  file_right_lwr_ee_pose_ << "Time,x,y,z,qx,qy,qz,qw \n";

  file_right_lwr_ee_vel_.open(recPath + "/right_lwr_ee_vel.csv");
  file_right_lwr_ee_vel_ << "Time,x,y,z,rx,ry,rz \n";


  file_left_ft_.open(recPath + "/left_ft.csv");
  file_left_ft_ << "Time,x,y,z,rx,ry,rz \n";

  file_right_ft_.open(recPath + "/right_ft.csv");
  file_right_ft_ << "Time,x,y,z,rx,ry,rz \n";


  file_obstacle_pose_.open(recPath + "/obstacle_pose.csv");
  file_obstacle_pose_ << "Time,x,y,z,qx,qy,qz,qw \n";

  file_modulated_velocity_.open(recPath + "/modulated_velocity.csv");
  file_modulated_velocity_ << "Time,x,y,z,rx,ry,rz \n";

  file_modulated_difference_.open(recPath + "/modulated_difference.csv");
  file_modulated_difference_ << "Time,x,y,z,rx,ry,rz \n";

  file_guard_pose_in_world_.open(recPath + "/guard_pose_in_world.csv");
  file_guard_pose_in_world_ << "Time,x,y,z,qx,qy,qz,qw \n";

  file_guard_twist_in_world_.open(recPath + "/guard_twist_in_world.csv");
  file_guard_twist_in_world_ << "Time,x,y,z,rx,ry,rz \n";

  file_guard_disturbance_.open(recPath + "/guard_disturbance.csv");
  file_guard_disturbance_ << "Time,power \n";

  file_tank_disturbance_.open(recPath + "/tank_disturbance.csv");
  file_tank_disturbance_ << "Time,energy \n";

  file_task_ds_velocity_.open(recPath + "/task_ds_velocity.csv");
  file_task_ds_velocity_   << "Time,x,y,z,rx,ry,rz \n";




  // std::string recPath_joints_state = recPath + "/joints_state.csv";
  // file_joints_state_.open(recPath_joints_state);
  // file_joints_state_ << "Time,q1,q2,q3,q4,q5,q6,q7 \n";

  // std::string recPath_target_state = recPath + "/target_state.csv";
  // file_target_state_.open(recPath_target_state);
  // file_target_state_ << "Time,x,y,z,qx,qy,qz,qw \n";

  ros::Duration(1).sleep();


  // initialize the ros communication

  sub_left_passive_ds_command_vel_ = nh_.subscribe("/left_lwr/joint_controllers/passive_ds_command_vel",1,
                                     &joint_carry_recorder::Callback_left_passive_ds_command_vel,
                                     this,ros::TransportHints().reliable().tcpNoDelay());

  sub_left_passive_ds_linear_damping_ = nh_.subscribe("/left_lwr/joint_controllers/passive_ds_linear_damping",1,
                                        &joint_carry_recorder::Callback_left_passive_ds_linear_damping,
                                        this,ros::TransportHints().reliable().tcpNoDelay());

  sub_left_lwr_ee_pose_ = nh_.subscribe("/left_lwr/ee_pose",1,
                                        &joint_carry_recorder::Callback_left_lwr_ee_pose,
                                        this,ros::TransportHints().reliable().tcpNoDelay());

  sub_left_lwr_ee_vel_ = nh_.subscribe("/left_lwr/ee_vel",1,
                                       &joint_carry_recorder::Callback_left_lwr_ee_vel,
                                       this,ros::TransportHints().reliable().tcpNoDelay());


  sub_right_passive_ds_command_vel_ = nh_.subscribe("/right_lwr/joint_controllers/passive_ds_command_vel",1,
                                      &joint_carry_recorder::Callback_right_passive_ds_command_vel,
                                      this,ros::TransportHints().reliable().tcpNoDelay());

  sub_right_passive_ds_linear_damping_ = nh_.subscribe("/right_lwr/joint_controllers/passive_ds_linear_damping",1,
                                         &joint_carry_recorder::Callback_right_passive_ds_linear_damping,
                                         this,ros::TransportHints().reliable().tcpNoDelay());

  sub_right_lwr_ee_pose_ = nh_.subscribe("/right_lwr/ee_pose",1,
                                         &joint_carry_recorder::Callback_right_lwr_ee_pose,
                                         this,ros::TransportHints().reliable().tcpNoDelay());

  sub_right_lwr_ee_vel_ = nh_.subscribe("/right_lwr/ee_vel",1,
                                        &joint_carry_recorder::Callback_right_lwr_ee_vel,
                                        this,ros::TransportHints().reliable().tcpNoDelay());



  sub_left_ft_ = nh_.subscribe("/left/netft_data",1,
                               &joint_carry_recorder::Callback_left_ft,
                               this,ros::TransportHints().reliable().tcpNoDelay());

  sub_right_ft_ = nh_.subscribe("/right/netft_data",1,
                                &joint_carry_recorder::Callback_right_ft,
                                this,ros::TransportHints().reliable().tcpNoDelay());


  sub_obstacle_pose_ = nh_.subscribe("/vrpn_client_node/obstacle/pose",1,
                                     &joint_carry_recorder::Callback_obstacle_pose,
                                     this,ros::TransportHints().reliable().tcpNoDelay());

  sub_modulated_velocity_ = nh_.subscribe("/obstacle_avoidance/modulated_velocity",1,
                                          &joint_carry_recorder::Callback_modulated_velocity,
                                          this,ros::TransportHints().reliable().tcpNoDelay());

  sub_modulated_difference_ = nh_.subscribe("/obstacle_avoidance/modulated_difference",1,
                              &joint_carry_recorder::Callback_modulated_difference,
                              this,ros::TransportHints().reliable().tcpNoDelay());

  sub_guard_pose_in_world_ = nh_.subscribe("/main_node/guard_pose",1,
                             &joint_carry_recorder::Callback_guard_pose_in_world,
                             this,ros::TransportHints().reliable().tcpNoDelay());

  sub_guard_twist_in_world_ = nh_.subscribe("/main_node/guard_twist",1,
                              &joint_carry_recorder::Callback_guard_twist_in_world,
                              this,ros::TransportHints().reliable().tcpNoDelay());

  sub_guard_disturbance_ = nh_.subscribe("/main_node/guard_disturbance",1,
                                         &joint_carry_recorder::Callback_guard_disturbance,
                                         this,ros::TransportHints().reliable().tcpNoDelay());

  sub_tank_disturbance_ = nh_.subscribe("/main_node/tank_disturbance",1,
                                        &joint_carry_recorder::Callback_tank_disturbance,
                                        this,ros::TransportHints().reliable().tcpNoDelay());

  sub_task_ds_velocity_ = nh_.subscribe("/Task1/desired_velocity",1,
                                        &joint_carry_recorder::Callback_task_ds_velocity,
                                        this,ros::TransportHints().reliable().tcpNoDelay());



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
    ROS_INFO_STREAM_THROTTLE(2,"Recording ....");
    ros::spinOnce();
    loop_rate_.sleep();
  }
}



/////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Callbacks //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////// left lwr //////////////////////////////////////////////////

void joint_carry_recorder::Callback_left_passive_ds_command_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
  file_left_passive_ds_command_vel_ << (ros::Time::now() - time_start_).toSec() << "," <<
                                    msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "\n";
}
void joint_carry_recorder::Callback_left_passive_ds_linear_damping(const std_msgs::Float64MultiArray::ConstPtr& msg) {

  file_left_passive_ds_linear_damping_ << (ros::Time::now() - time_start_).toSec() << "," <<
                                       msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << "\n";
}

void joint_carry_recorder::Callback_left_lwr_ee_pose(const geometry_msgs::Pose::ConstPtr& msg) {
  file_left_lwr_ee_pose_ << (ros::Time::now() - time_start_).toSec() << "," <<
                         msg->position.x << "," << msg->position.y << "," << msg->position.z << "," <<
                         msg->orientation.x << "," << msg->orientation.y << "," <<
                         msg->orientation.z << "," << msg->orientation.w << "\n";

}
void joint_carry_recorder::Callback_left_lwr_ee_vel(const geometry_msgs::Twist::ConstPtr& msg) {
  file_left_lwr_ee_vel_ << (ros::Time::now() - time_start_).toSec() << "," <<
                        msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "," <<
                        msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "\n";
}

//////////////////////////// right lwr //////////////////////////////////////////////////

void joint_carry_recorder::Callback_right_passive_ds_command_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
  file_right_passive_ds_command_vel_ << (ros::Time::now() - time_start_).toSec() << "," <<
                                     msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "\n";
}
void joint_carry_recorder::Callback_right_passive_ds_linear_damping(const std_msgs::Float64MultiArray::ConstPtr& msg) {

  file_right_passive_ds_linear_damping_ << (ros::Time::now() - time_start_).toSec() << "," <<
                                        msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << "\n";
}

void joint_carry_recorder::Callback_right_lwr_ee_pose(const geometry_msgs::Pose::ConstPtr& msg) {
  file_right_lwr_ee_pose_ << (ros::Time::now() - time_start_).toSec() << "," <<
                          msg->position.x << "," << msg->position.y << "," << msg->position.z << "," <<
                          msg->orientation.x << "," << msg->orientation.y << "," <<
                          msg->orientation.z << "," << msg->orientation.w << "\n";

}
void joint_carry_recorder::Callback_right_lwr_ee_vel(const geometry_msgs::Twist::ConstPtr& msg) {
  file_right_lwr_ee_vel_ << (ros::Time::now() - time_start_).toSec() << "," <<
                         msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "," <<
                         msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "\n";
}


void joint_carry_recorder::Callback_left_ft(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  file_left_ft_ << (ros::Time::now() - time_start_).toSec() << "," <<
                msg->wrench.force.x << "," << msg->wrench.force.y << "," << msg->wrench.force.z << "," <<
                msg->wrench.torque.x << "," << msg->wrench.torque.y << "," << msg->wrench.torque.z << "\n";
}
void joint_carry_recorder::Callback_right_ft(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  file_right_ft_ << (ros::Time::now() - time_start_).toSec() << "," <<
                 msg->wrench.force.x << "," << msg->wrench.force.y << "," << msg->wrench.force.z << "," <<
                 msg->wrench.torque.x << "," << msg->wrench.torque.y << "," << msg->wrench.torque.z << "\n";
}




void joint_carry_recorder::Callback_obstacle_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  file_obstacle_pose_ << (ros::Time::now() - time_start_).toSec() << "," <<
                      msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z << "," <<
                      msg->pose.orientation.x << "," << msg->pose.orientation.y << "," <<
                      msg->pose.orientation.z << "," << msg->pose.orientation.w << "\n";


}
void joint_carry_recorder::Callback_modulated_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {

  file_modulated_velocity_ << (ros::Time::now() - time_start_).toSec() << "," <<
                           msg->twist.linear.x << "," << msg->twist.linear.y << "," << msg->twist.linear.z << "," <<
                           msg->twist.angular.x << "," << msg->twist.angular.y << "," << msg->twist.angular.z << "\n";

}
void joint_carry_recorder::Callback_modulated_difference(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  file_modulated_difference_ << (ros::Time::now() - time_start_).toSec() << "," <<
                             msg->twist.linear.x << "," << msg->twist.linear.y << "," << msg->twist.linear.z << "," <<
                             msg->twist.angular.x << "," << msg->twist.angular.y << "," << msg->twist.angular.z << "\n";

}

void joint_carry_recorder::Callback_guard_pose_in_world(const geometry_msgs::Pose::ConstPtr& msg) {

  file_guard_pose_in_world_ << (ros::Time::now() - time_start_).toSec() << "," <<
                             msg->position.x << "," << msg->position.y << "," << msg->position.z << "," <<
                             msg->orientation.x << "," << msg->orientation.y << "," <<
                             msg->orientation.z << "," << msg->orientation.w << "\n";

}
void joint_carry_recorder::Callback_guard_twist_in_world(const geometry_msgs::Twist::ConstPtr& msg) {

  file_guard_twist_in_world_ << (ros::Time::now() - time_start_).toSec() << "," <<
                             msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "," <<
                             msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "\n";

}

void joint_carry_recorder::Callback_guard_disturbance(const std_msgs::Float32::ConstPtr& msg) {

  file_guard_disturbance_ << (ros::Time::now() - time_start_).toSec() << "," << msg->data << "\n";

}
void joint_carry_recorder::Callback_tank_disturbance(const std_msgs::Float32::ConstPtr& msg) {

  file_tank_disturbance_ << (ros::Time::now() - time_start_).toSec() << "," << msg->data << "\n";

}

void joint_carry_recorder::Callback_task_ds_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  file_task_ds_velocity_ << (ros::Time::now() - time_start_).toSec() << "," <<
                         msg->twist.linear.x << "," << msg->twist.linear.y << "," << msg->twist.linear.z << "," <<
                         msg->twist.angular.x << "," << msg->twist.angular.y << "," << msg->twist.angular.z << "\n";



}












joint_carry_recorder::~joint_carry_recorder()
{
  file_left_passive_ds_command_vel_.close();
  file_left_passive_ds_linear_damping_.close();
  file_left_lwr_ee_pose_.close();
  file_left_lwr_ee_vel_.close();
  // file_joints_state_.close();
  // file_target_state_.close();
}