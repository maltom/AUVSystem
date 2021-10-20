#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "Debug.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "Debug" );

	std::shared_ptr< ros::NodeHandle > debugNode = std::make_shared< ros::NodeHandle >();

	Debug debug( debugNode, configFiles::fileID::main_file );
	debug.startMainLoop();
}