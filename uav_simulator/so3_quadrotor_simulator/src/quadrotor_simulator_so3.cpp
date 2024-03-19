#include "ros/node_handle.h"
#include <quadrotor_simulator/simulator.h>


int main( int argc, char** argv ) {
	ros::init( argc, argv, "quadrotor_simulator_so3" );
	ros::NodeHandle n( "~" );
	QuadrotorSimulator::Simulator sim(n);
	sim.run();
	return 0;
}