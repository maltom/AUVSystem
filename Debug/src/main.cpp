#include <exception>
#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "Debug.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "Debug" );

	std::shared_ptr< ros::NodeHandle > debugNode = std::make_shared< ros::NodeHandle >();

	std::fstream configFile;
	configFile.open( CONFIG_FILE_PATH, std::fstream::in );
	if( !configFile.is_open() )
	{
		throw "Can't open config file!";
	}

	Debug debug( debugNode, configFile );
    debug.startMainLoop();
}