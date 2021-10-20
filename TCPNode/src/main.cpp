#include <exception>
#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "TCPNode.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "TCPNode" );

	std::shared_ptr< ros::NodeHandle > tcpNodeNode = std::make_shared< ros::NodeHandle >();

	std::fstream configFile;
	configFile.open( CONFIG_FILE_PATH, std::fstream::in );
	if( !configFile.is_open() )
	{
		throw "Can't open config file!";
	}

	TCPNode tcpNode( tcpNodeNode, configFile );
    tcpNode.startMainLoop();

	configFile.close();
}