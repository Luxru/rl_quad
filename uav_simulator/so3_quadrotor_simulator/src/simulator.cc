#include <chrono>
#include <iostream>
#include <ros/duration.h>
#include "ros/node_handle.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Control.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <quadrotor_simulator/simulator.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <uav_utils/geometry_utils.h>
#include "sophus/so3.hpp"

namespace QuadrotorSimulator {
void Simulator::update_odom() {
	auto state				  = quad.getState();
	odom.pose.pose.position.x = state.x( 0 );
	odom.pose.pose.position.y = state.x( 1 );
	odom.pose.pose.position.z = state.x( 2 );

	Eigen::Quaterniond q( state.R );
	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();

	odom.twist.twist.linear.x = state.v( 0 );
	odom.twist.twist.linear.y = state.v( 1 );
	odom.twist.twist.linear.z = state.v( 2 );

	odom.twist.twist.angular.x = state.omega( 0 );
	odom.twist.twist.angular.y = state.omega( 1 );
	odom.twist.twist.angular.z = state.omega( 2 );

	odom.header.stamp = ros::Time::now();
}

void Simulator::update_imu() {
	QuadrotorSimulator::Quadrotor::State state = quad.getState();
	Eigen::Quaterniond					 q( state.R );
	imu.orientation.x = q.x();
	imu.orientation.y = q.y();
	imu.orientation.z = q.z();
	imu.orientation.w = q.w();

	imu.angular_velocity.x = state.omega( 0 );
	imu.angular_velocity.y = state.omega( 1 );
	imu.angular_velocity.z = state.omega( 2 );

	imu.linear_acceleration.x = quad.getAcc()[ 0 ];
	imu.linear_acceleration.y = quad.getAcc()[ 1 ];
	imu.linear_acceleration.z = quad.getAcc()[ 2 ];

	imu.header.stamp = ros::Time::now();
}

void Simulator::update_control() {
	switch ( command.type ) {
	case quadrotor_msgs::Control::THRUST_BODYRATE: {
		float			force = (command.u[ 0 ])*quad.getMass();//acc
		Eigen::Vector3d body_rate( command.u[ 1 ], command.u[ 2 ], command.u[ 3 ] );
		Eigen::Matrix3d Kinv_ang_vel_tau_ = Eigen::Vector3d( 16.6, 16.6, 5.0 ).asDiagonal();
		Eigen::Matrix3d J				  = quad.getInertia();
		auto			state			  = quad.getState();
		Eigen::Vector3d omega			  = state.omega;
		Eigen::Vector3d omega_err		  = body_rate - omega;
		Eigen::Vector3d body_torque_des	  = J * Kinv_ang_vel_tau_ * omega_err + omega.cross( J * omega );

		float		 M1	 = body_torque_des.x();
		float		 M2	 = body_torque_des.y();
		float		 M3	 = body_torque_des.z();
		const double _kf = quad.getPropellerThrustCoefficient();
		const double _km = quad.getPropellerMomentCoefficient();
		const double kf	 = _kf;
		const double km	 = _km / _kf * kf;
		const double d	 = quad.getArmLength();
		float		 w_sq[ 4 ];
		w_sq[ 0 ] = force / ( 4 * kf ) - M2 / ( 2 * d * kf ) + M3 / ( 4 * km );
		w_sq[ 1 ] = force / ( 4 * kf ) + M2 / ( 2 * d * kf ) + M3 / ( 4 * km );
		w_sq[ 2 ] = force / ( 4 * kf ) + M1 / ( 2 * d * kf ) - M3 / ( 4 * km );
		w_sq[ 3 ] = force / ( 4 * kf ) - M1 / ( 2 * d * kf ) - M3 / ( 4 * km );

		for ( int i = 0; i < 4; i++ ) {
			if ( w_sq[ i ] < 0 )
				w_sq[ i ] = 0;

			control.rpm[ i ] = sqrtf( w_sq[ i ] );
		}
		ROS_INFO_THROTTLE(1,"Input force:%f,bodyrate:%f,%f,%f,pose:%f,%f,%f",force,command.u[1],command.u[2],command.u[3],state.x(0),state.x(1),state.x(2));
		break;
	}
	case quadrotor_msgs::Control::THRUST_TORQUE: {
		float		 force = command.u[ 0 ];//motor force
		float		 M1	   = command.u[ 1 ];
		float		 M2	   = command.u[ 2 ];
		float		 M3	   = command.u[ 3 ];
		const double _kf   = quad.getPropellerThrustCoefficient();
		const double _km   = quad.getPropellerMomentCoefficient();
		const double kf	   = _kf;
		const double km	   = _km / _kf * kf;
		const double d	   = quad.getArmLength();
		float		 w_sq[ 4 ];
		w_sq[ 0 ] = force / ( 4 * kf ) - M2 / ( 2 * d * kf ) + M3 / ( 4 * km );
		w_sq[ 1 ] = force / ( 4 * kf ) + M2 / ( 2 * d * kf ) + M3 / ( 4 * km );
		w_sq[ 2 ] = force / ( 4 * kf ) + M1 / ( 2 * d * kf ) - M3 / ( 4 * km );
		w_sq[ 3 ] = force / ( 4 * kf ) - M1 / ( 2 * d * kf ) - M3 / ( 4 * km );
		for ( int i = 0; i < 4; i++ ) {
			if ( w_sq[ i ] < 0 )
				w_sq[ i ] = 0;
			control.rpm[ i ] = sqrtf( w_sq[ i ] );
		}
		break;
	}
	case quadrotor_msgs::Control::ROTORS_RPM: {
		for ( int i = 0; i < 4; i++ ) {
			control.rpm[ i ] = command.u[ i ];
		}
		break;
	}
	default:
		ROS_ERROR( "Unknown control type" );
		break;
	}
	return;
}

void Simulator::cmd_callback( const quadrotor_msgs::Control::ConstPtr& cmd ) {
	// ROS_INFO("RECV Command");
	command = *cmd;
	last_cmd_time = std::chrono::steady_clock::now();
	return;
}

void Simulator::force_disturbance_callback( const geometry_msgs::Vector3::ConstPtr& f ) {
	disturbance.f( 0 ) = f->x;
	disturbance.f( 1 ) = f->y;
	disturbance.f( 2 ) = f->z;
}

void Simulator::moment_disturbance_callback( const geometry_msgs::Vector3::ConstPtr& m ) {
	disturbance.m( 0 ) = m->x;
	disturbance.m( 1 ) = m->y;
	disturbance.m( 2 ) = m->z;
}

void Simulator::reset_callback( const geometry_msgs::Point::ConstPtr& msg ) {
	quad.reset_callback( msg );
	control = Control();
	command = quadrotor_msgs::Control();
	command.type = command.THRUST_BODYRATE;
	return;
}

bool Simulator::pause_step_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res ) {
	pause_step = true;
	return true;
}

bool Simulator::unpause_step_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res ) {
	pause_step = false;
	return true;
}

bool Simulator::step_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res ) {
	const double dt = 1 / simulation_rate;
	quad.step( dt );
	return true;
}

void Simulator::update_control_to_hover(){
    //in
    double g = quad.getGravity();
    Eigen::Vector3d    des_p,des_v,des_a,des_w,odom_v,odom_p,odom_w;
    Eigen::Quaterniond des_q;
    Eigen::Quaterniond odom_q(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
    odom_v<<odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z;
    odom_p<<odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z;
    odom_w<<odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z;

	des_v = des_a = des_w  = Eigen::Vector3d( 0.0, 0.0, 0.0 );
	des_p <<odom_p.x(),odom_p.y(),1;

	//out
    Eigen::Vector3d des_acc;
    Eigen::Vector3d body_rate;
    Eigen::Vector3d Kp(1.5,1.5,1.5), Kv(1.5,1.5,1.5);
    Eigen::Vector3d KR(5,5,5),Kw(10,10,10);
	Eigen::Vector3d err_p =  des_p - odom_p;
    Eigen::Vector3d err_v =  des_v - odom_v;

	des_acc = des_a + Kv.asDiagonal() * err_v + Kp.asDiagonal() * err_p +Eigen::Vector3d( 0, 0, g);

	//get rotation matrix from euler angle
	auto q = odom_q;
	double yaw_odom = atan2( 2 * ( q.x() * q.y() + q.w() * q.z() ), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z() );
    double sin      = std::sin( yaw_odom );
    double cos      = std::cos( yaw_odom );
    double roll            = ( des_acc( 0 ) * sin - des_acc( 1 ) * cos ) / g;
    double pitch           = ( des_acc( 0 ) * cos + des_acc( 1 ) * sin ) / g;
	des_q = Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitZ() ) * 
    Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY() ) * 
    Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX() );
    Eigen::Matrix3d des_R = des_q.toRotationMatrix();
    Eigen::Matrix3d odom_R = odom_q.toRotationMatrix();

	//TODO
    Eigen::Vector3d err_R =  0.5*vee(odom_R.transpose()*des_R-des_R.transpose()*odom_R);
    Eigen::Vector3d err_w =  des_w-odom_w;
    body_rate = KR.asDiagonal()*err_R + Kw.asDiagonal()*err_w;

    double thrust = des_acc.dot(odom_R*Eigen::Vector3d(0,0,1));
    command.u[0] = thrust;
    command.u[1] = body_rate.x();
    command.u[2] = body_rate.y();
    command.u[3] = body_rate.z();
    command.type = command.THRUST_BODYRATE;
    //not update timestamp
    update_control();
}

Eigen::Vector3d vee(const Eigen::Matrix3d &R){
    //todo
    return Sophus::SO3d::vee(R);;
}

void Simulator::run() {
	ros::Rate	 r( simulation_rate );
	const double dt = 1 / simulation_rate;

	while ( ros::ok() ) {
		ros::spinOnce();
		//not response in 200 ms
        bool hover_flag = std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::steady_clock::now() - last_cmd_time) ) > std::chrono::milliseconds(200);
		// check whether there has a external input
		if (hover_flag) {
			// enter hover mode
			ROS_WARN_THROTTLE(1, "Hover mode");
            update_control_to_hover();
		}
		else {
			// recv command
			auto last = control;
			update_control();
			for ( int i = 0; i < 4; ++i ) {
				//! @bug might have nan when the input is legal
				if ( std::isnan( control.rpm[ i ] ) )
					control.rpm[ i ] = last.rpm[ i ];
			}
		}
		// set input
		quad.setInput( control.rpm[ 0 ], control.rpm[ 1 ], control.rpm[ 2 ], control.rpm[ 3 ] );
		quad.setExternalForce( disturbance.f );
		quad.setExternalMoment( disturbance.m );

		// step
		if ( step_type == 2 || ( step_type == 1 && pause_step == false ) ) {
			quad.step( dt );
		}

		//pub message
		update_odom();
		update_imu();
		update_transform();

		odom_pub.publish( odom );
		imu_pub.publish( imu );
		br.sendTransform( transformStamped );
		
		//sleep
		r.sleep();
	}
}

void Simulator::update_transform() {
	transformStamped.header.stamp			 = ros::Time::now();
	transformStamped.header.frame_id		 = "world";
	transformStamped.child_frame_id			 = quad_name;
	transformStamped.transform.translation.x = odom.pose.pose.position.x;
	transformStamped.transform.translation.y = odom.pose.pose.position.y;
	transformStamped.transform.translation.z = odom.pose.pose.position.z;
	transformStamped.transform.rotation.x	 = odom.pose.pose.orientation.x;
	transformStamped.transform.rotation.y	 = odom.pose.pose.orientation.y;
	transformStamped.transform.rotation.z	 = odom.pose.pose.orientation.z;
	transformStamped.transform.rotation.w	 = odom.pose.pose.orientation.w;
}

Simulator::Simulator( ros::NodeHandle& nh ) {
	nh.param( "simulator/init_state_x", _init_x, 0.0 );
	nh.param( "simulator/init_state_y", _init_y, 0.0 );
	nh.param( "simulator/init_state_z", _init_z, 1.0 );
	nh.param( "rate/simulation", simulation_rate, 1000.0 );
	nh.param( "rate/odom", odom_rate, 200.0 );
	nh.param( "quadrotor_name", quad_name, std::string( "quadrotor" ) );
	// 0-for
	nh.param( "step_type", step_type, 2 );
	ROS_ASSERT( simulation_rate > 0 );
	const ros::Duration odom_pub_duration( 1 / odom_rate );
	const double		dt = 1 / simulation_rate;
	quad.setStatePos( Eigen::Vector3d( _init_x, _init_y, _init_z ) );

	odom_pub  = nh.advertise< nav_msgs::Odometry >( "odom", 100 );
	imu_pub	  = nh.advertise< sensor_msgs::Imu >( "imu", 10 );
	cmd_sub	  = nh.subscribe( "cmd", 100, &Simulator::cmd_callback, this, ros::TransportHints().tcpNoDelay() );
	f_sub	  = nh.subscribe( "force_disturbance", 100, &Simulator::force_disturbance_callback, this, ros::TransportHints().tcpNoDelay() );
	m_sub	  = nh.subscribe( "moment_disturbance", 100, &Simulator::moment_disturbance_callback, this, ros::TransportHints().tcpNoDelay() );
	reset_sub = nh.subscribe( "reset_quad_pos", 10, &Simulator::reset_callback, this, ros::TransportHints().tcpNoDelay() );

	// set step type
	switch ( step_type ) {
	case 0:
		step_srv = nh.advertiseService( "step", &Simulator::step_callback, this );
		break;
	case 1:
		pause_srv	= nh.advertiseService( "pause_step", &Simulator::pause_step_callback, this );
		unpause_srv = nh.advertiseService( "unpause_step", &Simulator::unpause_step_callback, this );
		break;
	}

	// set header
	odom.header.frame_id = "/simulator";
	odom.child_frame_id	 = "/" + quad_name;
	sensor_msgs::Imu imu;
	imu.header.frame_id = "/simulator";
	command.type = command.THRUST_BODYRATE;
	last_cmd_time = std::chrono::steady_clock::now();
}

}  // namespace QuadrotorSimulator