#pragma once
#include "ros/publisher.h"
#include <chrono>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Control.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

namespace QuadrotorSimulator {

typedef struct _Control {
	double rpm[ 4 ];
} Control;

typedef struct _Disturbance {
	Eigen::Vector3d f;
	Eigen::Vector3d m;
} Disturbance;

class Simulator {
public:
	Simulator( ros::NodeHandle& nh );
	void run();

private:
	ros::Publisher odom_pub,imu_pub;
	ros::Subscriber cmd_sub, f_sub, m_sub, reset_sub;
	ros::ServiceServer pause_srv, unpause_srv, step_srv;
	tf2_ros::TransformBroadcaster	br;

	std::chrono::steady_clock::time_point last_cmd_time;
	quadrotor_msgs::Control		  command;
	Disturbance					  disturbance;
	bool						  pause_step = false;
	int							  step_type	 = 2;
	double						  _init_x, _init_y, _init_z;
	double						  simulation_rate, odom_rate;
	std::string					  quad_name;
	QuadrotorSimulator::Quadrotor quad;
	Control						  control;
	nav_msgs::Odometry				odom;
	sensor_msgs::Imu imu;
	geometry_msgs::TransformStamped transformStamped;

	void force_disturbance_callback( const geometry_msgs::Vector3::ConstPtr& f );
	void moment_disturbance_callback( const geometry_msgs::Vector3::ConstPtr& m );
	void reset_callback( const geometry_msgs::Point::ConstPtr& msg );
	bool pause_step_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
	bool unpause_step_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
	bool step_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
	void cmd_callback( const quadrotor_msgs::Control::ConstPtr& cmd );

	void update_transform();
	void update_odom();
	void update_imu();
	void update_control();
	void update_control_to_hover();
};
//TODO
	Eigen::Vector3d  vee(const Eigen::Matrix3d &R);
};	// namespace QuadrotorSimulator