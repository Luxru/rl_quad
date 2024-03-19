#include "ros/init.h"
#include <cassert>
#include <chrono>
#include <gcopter/sfc_gen.hpp>
#include <ratio>
#include <sfc_node.hpp>
#include <vector>

HPoly::HPoly( const Config& conf, ros::NodeHandle& nh_ ) :
 config( conf ), nh( nh_ ), tfListener(tfBuffer), mapInitialized( false ), odomInitialized( false ), visualizer( nh ),engine( seed() ) {
	/*         char msg[50];
			sprintf(msg,"%d",config.mapBound.size());
			ROS_WARN(msg);
			ROS_WARN(config.mapTopic.c_str()); */
	// 它计算了一个三维向量，其各个分量分别表示三个方向（x、y、z）上的体素数量。
	const Eigen::Vector3i xyz( ( config.mapBound[ 1 ] - config.mapBound[ 0 ] ) / config.voxelWidth,
							   ( config.mapBound[ 3 ] - config.mapBound[ 2 ] ) / config.voxelWidth,
							   ( config.mapBound[ 5 ] - config.mapBound[ 4 ] ) / config.voxelWidth );
	// 分别代表了空间的 x、y、z 方向的最小边界值，因此这个向量表示了体素地图的起始位置在三维空间中的坐标。
	const Eigen::Vector3d offset( config.mapBound[ 0 ], config.mapBound[ 2 ], config.mapBound[ 4 ] );

	voxelMap = voxel_map::VoxelMap( xyz, offset, config.voxelWidth );

	resetQuadPub = nh.advertise< geometry_msgs::Point >( config.resetQuadTopic, 1 );

	resetMapPub = nh.advertise< std_msgs::Empty >( config.resetMapTopic, 1 );

	mapSub		  = nh.subscribe( config.mapTopic, 1, &HPoly::mapCallBack, this, ros::TransportHints().tcpNoDelay() );
	odomSub		  = nh.subscribe( config.odomTopic, 1, &HPoly::odomCallBack, this, ros::TransportHints().tcpNoDelay() );
	targetSub	  = nh.subscribe( config.targetTopic, 1, &HPoly::targetCallBack, this, ros::TransportHints().tcpNoDelay() );
	mapResttedSub = nh.subscribe( config.mapResettedTopic, 1, &HPoly::resetMapCallback, this, ros::TransportHints().tcpNoDelay() );
	hpolySrv	  = nh.advertiseService( config.hpolySrv, &HPoly::srvCallBack, this );
}

void HPoly::mapCallBack( const sensor_msgs::PointCloud2::ConstPtr& msg ) {
	std::unique_lock< std::mutex > lock( map_mutex );
	if ( !mapInitialized ) {
		voxelMap.resetMap();
		ROS_WARN( "Init grid map." );
		size_t		 cur   = 0;
		const size_t total = msg->data.size() / msg->point_step;
		float*		 fdata = ( float* )( &msg->data[ 0 ] );
		for ( size_t i = 0; i < total; i++ ) {
			cur = msg->point_step / sizeof( float ) * i;

			if ( std::isnan( fdata[ cur + 0 ] ) || std::isinf( fdata[ cur + 0 ] ) || std::isnan( fdata[ cur + 1 ] ) || std::isinf( fdata[ cur + 1 ] ) || std::isnan( fdata[ cur + 2 ] )
				 || std::isinf( fdata[ cur + 2 ] ) ) {
				continue;
			}
			voxelMap.setOccupied( Eigen::Vector3d( fdata[ cur + 0 ], fdata[ cur + 1 ], fdata[ cur + 2 ] ) );
		}

		voxelMap.dilate( std::ceil( config.dilateRadius / voxelMap.getScale() ) );

		mapInitialized = true;
	}
}

void HPoly::resetMapCallback( const std_msgs::Empty msg ) {
	std::unique_lock< std::mutex > lock( map_mutex );
	mapInitialized = false;
	ROS_WARN( "Map reset, Set mapInitialized to false." );
	return;
}

void HPoly::odomCallBack( const nav_msgs::Odometry& msg ) {
	odom			= msg;
	odomInitialized = true;
	return;
}

void HPoly::generatePoly( Eigen::Vector3d goal ) {
	assert( odomInitialized );
	auto&				  pose = odom.pose.pose.position;
	const Eigen::Vector3d start( pose.x, pose.y, pose.z );
	sfc_gen::planPath< voxel_map::VoxelMap >( start, goal, voxelMap.getOrigin(), voxelMap.getCorner(), &voxelMap, 0.01, route );
	visualizer.visualizeRoute( route );
	std::vector< Eigen::Vector3d > pc;
	voxelMap.getSurf( pc );
	sfc_gen::convexCover( route, pc, voxelMap.getOrigin(), voxelMap.getCorner(), 7.0, 3.0, hPolys );
	sfc_gen::shortCut( hPolys );
	visualizer.visualizePolytope( hPolys );
	return;
}

void HPoly::generatePoly( Eigen::Vector3d start, Eigen::Vector3d goal ) {
	assert( odomInitialized );
    // fast 10ms
    benchmark::tick( "planPath" );
	sfc_gen::planPath< voxel_map::VoxelMap >( start, goal, voxelMap.getOrigin(), voxelMap.getCorner(), &voxelMap, 0.01, route );
	benchmark::tock( "planPath" );
    visualizer.visualizeRoute( route );

    //fast 18ms
    benchmark::tick( "voxelMap.getSurf" );
	std::vector< Eigen::Vector3d > pc;
	voxelMap.getSurf( pc );
    benchmark::tock( "voxelMap.getSurf" );

    //slow 540ms 95% time
    
    benchmark::tick( "convexCover" );
	sfc_gen::convexCover( route, pc, voxelMap.getOrigin(), voxelMap.getCorner(), 7.0, 3.0, hPolys );
	benchmark::tock( "convexCover" );

    //fast 0ms
    benchmark::tick( "shortCut" );
    sfc_gen::shortCut( hPolys );
    benchmark::tock( "shortCut" );
	visualizer.visualizePolytope( hPolys );
	return;
}

bool HPoly::srvCallBack( rl_control_test::Sfc::Request& req, rl_control_test::Sfc::Response& res ) {
	std::unique_lock< std::mutex > lock( map_mutex );
	if ( mapInitialized && odomInitialized ) {
		if(!init_goal){
			randomGenerateStartEnd( start, goal );
			init_goal = true;
		}
		geometry_msgs::Point pt;

		pt.x = start[ 0 ];
		pt.y = start[ 1 ];
		pt.z = start[ 2 ];
		resetQuadPub.publish( pt );
        bool is_valid = voxelMap.query( goal ) == 0 && voxelMap.query( start ) == 0;
		if (is_valid) {
            benchmark::tick( "generatePoly" );
			generatePoly( start, goal );
            benchmark::tock( "generatePoly" );
			geometry_msgs::TransformStamped transformStamped;
			while(ros::ok()){
				try{
				transformStamped = tfBuffer.lookupTransform("world", "quadrotor",
										ros::Time(0));
				break;
				}
				catch (tf2::TransformException &ex) {
					ROS_WARN("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
			}
			Eigen::Matrix3d R = Eigen::Quaterniond(transformStamped.transform.rotation.w,transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z).toRotationMatrix();
			Eigen::Matrix4d T;
			T<<R(0,0),R(0,1),R(0,2),transformStamped.transform.translation.x,
				R(1,0),R(1,1),R(1,2),transformStamped.transform.translation.y,
				R(2,0),R(2,1),R(2,2),transformStamped.transform.translation.z,
				0,0,0,1;
			for ( auto& hpoly : hPolys ) {
				std::vector<Eigen::Vector4d> p,pr;
				rl_control_test::Polytope poly;
				for ( int i = 0; i < hpoly.rows(); ++i ) {
					rl_control_test::Halfspace hs;
					hs.h0 = hpoly.row( i )[ 0 ];
					hs.h1 = hpoly.row( i )[ 1 ];
					hs.h2 = hpoly.row( i )[ 2 ];
					hs.h3 = hpoly.row( i )[ 3 ];
					Eigen::Vector4d sc(hs.h0,hs.h1,hs.h2,hs.h3);
					pr.push_back(sc);
					sc = sc.transpose()*T;
					p.push_back(sc);
					poly.halfspaces.emplace_back( hs );
				}
				bool in_hull = true;
				Eigen::Vector4d origin(0,0,0,1);
				for(auto &sc:p){
					if(sc.dot(origin)>0){
						in_hull = false;
						break;
					}
				}
				bool in_hull_raw=true;
				Eigen::Vector4d quad_pose(transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z,1);
				for(auto& sc:pr){
					if(sc.dot(quad_pose)>0){
						in_hull_raw = false;
						break;
					}
				}
				ROS_WARN("in_hull:%d in_hull_raw:%d",in_hull,in_hull_raw);
				assert(in_hull==in_hull_raw);
				res.polys.emplace_back( poly );
			}

			for ( auto& pt : route ) {
				geometry_msgs::Point p;
				p.x = pt[ 0 ];
				p.y = pt[ 1 ];
				p.z = pt[ 2 ];
				res.route.emplace_back( p );
			}
			//do not change the map
			// ++count;
			// if ( count % config.rl_rounds == 0 ) {
			// 	resetMapPub.publish( std_msgs::Empty() );
			// }
			res.ret_code = res.ret_success;
			return true;
		}
		else {
			res.ret_code = res.ret_invalid_target;
			ROS_WARN( "Infeasible Position Selected !!!\n" );
			return true;
		}
	}
	res.ret_code = res.ret_failed;
	return true;
}

void HPoly::targetCallBack( const geometry_msgs::PoseStamped::ConstPtr& msg ) {
	if ( mapInitialized && odomInitialized ) {
		std::unique_lock< std::mutex > lock( map_mutex );
		const double				   zGoal = config.mapBound[ 4 ] + config.dilateRadius + fabs( msg->pose.orientation.z ) * ( config.mapBound[ 5 ] - config.mapBound[ 4 ] - 2 * config.dilateRadius );
		const Eigen::Vector3d		   goal( msg->pose.position.x, msg->pose.position.y, zGoal );
		if ( voxelMap.query( goal ) == 0 ) {
			visualizer.visualizeStartGoal( goal, 0.5, 0 );
			generatePoly( goal );
		}
		else {
			ROS_WARN( "Infeasible Position Selected !!!\n" );
		}
	}
	return;
}

void HPoly::randomGeneratePoint( Eigen::Vector3d& pt ) {
	std::uniform_int_distribution<> distrib( min, max );  // 设置随机数范围，并为均匀分布
	pt << distrib( engine ), distrib( engine ), 1;
	return;
}

void HPoly::randomGenerateStartEnd( Eigen::Vector3d& start, Eigen::Vector3d& goal ) {
	assert( config.min_dist > 0 );
	for ( randomGeneratePoint( start ); voxelMap.query( start ) != 0; randomGeneratePoint( start ) )
		;
	for ( randomGeneratePoint( goal ); sqrt( ( ( start - goal ) * ( start - goal ).transpose() ).sum() ) < config.min_dist || voxelMap.query( goal ) != 0; randomGeneratePoint( goal ) )
		;
	return;
}

int main( int argc, char** argv ) {
	ros::init( argc, argv, "sfc_node" );
	ros::NodeHandle nh_;

	HPoly sfc_node( Config( ros::NodeHandle( "~" ) ), nh_ );

	ros::spin();
	return 0;
}
