/*
hpoly
Safe flight corridors for target paths (polyhedra)
*/
#pragma once
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include "misc/visualizer.hpp"
#include "ros/publisher.h"
#include "std_msgs/Empty.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <random>
#include <rl_control_test/Sfc.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

struct Config {
	std::string			  resetQuadTopic;
	std::string			  resetMapTopic;
	std::string			  mapResettedTopic;
	std::string			  mapTopic;
	std::string			  odomTopic;
	std::string			  targetTopic;
	std::string			  hpolySrv;
	double				  dilateRadius;
	double				  voxelWidth;
	int					  rl_rounds;
	int					  min_dist;
	std::vector< double > mapBound;

	Config( const ros::NodeHandle& nh_priv ) {
		nh_priv.getParam( "reset_quad_topic", resetQuadTopic );
		nh_priv.getParam( "reset_map_topic", resetMapTopic );
		nh_priv.getParam( "map_resetted_topic", mapResettedTopic );
		nh_priv.getParam( "map_topic", mapTopic );
		nh_priv.getParam( "target_topic", targetTopic );
		nh_priv.getParam( "odom_topic", odomTopic );
		nh_priv.getParam( "hpoly_srv", hpolySrv );
		nh_priv.getParam( "dilate_radius", dilateRadius );
		nh_priv.getParam( "voxel_width", voxelWidth );
		nh_priv.getParam( "map_bound", mapBound );
		nh_priv.getParam( "rl_rounds", rl_rounds );
		nh_priv.getParam( "min_dist", min_dist );
	}
};

class HPoly {
private:
	Config config;

	ros::NodeHandle	   nh;
	ros::Publisher	   resetQuadPub;
	ros::Publisher	   resetMapPub;
	ros::Subscriber	   mapSub;
	ros::Subscriber	   odomSub;
	ros::Subscriber	   targetSub;
	ros::Subscriber	   mapResttedSub;
	ros::ServiceServer hpolySrv;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;

	bool							mapInitialized, odomInitialized;
	voxel_map::VoxelMap				voxelMap;
	nav_msgs::Odometry				odom;
	Visualizer						visualizer;
	std::vector< Eigen::Vector3d >	startGoal;
	std::vector< Eigen::MatrixX4d > hPolys;
	std::vector< Eigen::Vector3d >	route;
	Trajectory< 5 >					traj;
	double							trajStamp;
	int								min = -25, max = 25, count = 0;
	std::random_device				seed;	 // 硬件生成随机数种子
	std::mt19937					engine;	 // 利用种子生成随机数引擎
	std::mutex						map_mutex;
	Eigen::Vector3d start, goal;
	bool init_goal = false;

public:
	HPoly( const Config& conf, ros::NodeHandle& nh_ );

private:
	void mapCallBack( const sensor_msgs::PointCloud2::ConstPtr& msg );
	void resetMapCallback( const std_msgs::Empty msg );
	void odomCallBack( const nav_msgs::Odometry& msg );
	void generatePoly( Eigen::Vector3d goal );
	void generatePoly( Eigen::Vector3d start, Eigen::Vector3d goal );
	bool srvCallBack( rl_control_test::Sfc::Request& req, rl_control_test::Sfc::Response& res );
	void targetCallBack( const geometry_msgs::PoseStamped::ConstPtr& msg );
	void randomGeneratePoint( Eigen::Vector3d& pt );
	void randomGenerateStartEnd( Eigen::Vector3d& start, Eigen::Vector3d& goal );
};

class benchmark {
public:
	static void tick( const std::string& name );
	static void tock( const std::string& name );

private:
static std::map< std::string,std::chrono::high_resolution_clock::time_point> start_times, end_times;
};

inline void benchmark::tick( const std::string& name ) {
    start_times[ name ] = std::chrono::high_resolution_clock::now();
}

inline void benchmark::tock( const std::string& name ) {
    end_times[ name ] = std::chrono::high_resolution_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>( end_times[ name ] - start_times[ name ] );
    ROS_WARN( "Cost of %s is: %ldms", name.c_str(), time_span.count() );
}

std::map< std::string,std::chrono::high_resolution_clock::time_point> benchmark::start_times, benchmark::end_times;
