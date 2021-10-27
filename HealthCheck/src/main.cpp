#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "HealthCheck.h"
#include "ROSEnums.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::HealthCheck ) );

	std::shared_ptr< ros::NodeHandle > healthCheckNode = std::make_shared< ros::NodeHandle >();

	HealthCheck healthCheck( healthCheckNode, configFiles::fileID::main_file, AUVROS::NodeIDs::HealthCheck );
	healthCheck.startMainLoop();
}