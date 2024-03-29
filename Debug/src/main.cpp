#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "Debug.h"
#include "ROSEnums.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::Debug ) );

	std::shared_ptr< ros::NodeHandle > debugNode = std::make_shared< ros::NodeHandle >();

	Debug debug( debugNode, configFiles::fileID::main_file, AUVROS::NodeIDs::Debug );
	debug.startMainLoop();
}