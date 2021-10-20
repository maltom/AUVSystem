#include <exception>
#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "UDPNode.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "UDPNode" );

	std::shared_ptr< ros::NodeHandle > udpNodeNode = std::make_shared< ros::NodeHandle >();

	std::fstream configFile;
	configFile.open( CONFIG_FILE_PATH, std::fstream::in );
	if( !configFile.is_open() )
	{
		throw "Can't open config file!";
	}

	UDPNode udpNode( udpNodeNode, configFile );
    udpNode.startMainLoop();

	configFile.close();
}