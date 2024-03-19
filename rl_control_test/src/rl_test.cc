#include<ros/ros.h>
#include<quadrotor_msgs/PositionCommand.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>

ros::Publisher pos_cmd_pub;

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
  auto odom = msg;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img){

}
void pubPosCmd(){
    quadrotor_msgs::PositionCommand cmd;
    ros::Time time_now = ros::Time::now();
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;

    cmd.position.x = 0;
    cmd.position.y = 0;
    cmd.position.z = 0;

    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;

    cmd.acceleration.x = 0;
    cmd.acceleration.y = 0;
    cmd.acceleration.z = 0;

    cmd.yaw = 0;
    cmd.yaw_dot = 0;
    pos_cmd_pub.publish(cmd);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rl_test_node");
  ros::NodeHandle nh("~");
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  ros::Subscriber odom_sub = nh.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber cloud_sub = nh.subscribe("/cloud", 50, cloudCallback);
  
  ros::Rate rate(50);
  while(ros::ok()){
    pubPosCmd();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
