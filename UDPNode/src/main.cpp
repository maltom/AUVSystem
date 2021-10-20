#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "UDPNode.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "UDPNode" );

	std::shared_ptr< ros::NodeHandle > udpNodeNode = std::make_shared< ros::NodeHandle >();

	UDPNode udpNode( udpNodeNode, configFiles::fileID::main_file );
    udpNode.startMainLoop();
}