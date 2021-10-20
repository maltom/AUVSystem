#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "TCPNode.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "TCPNode" );

	std::shared_ptr< ros::NodeHandle > tcpNodeNode = std::make_shared< ros::NodeHandle >();

	TCPNode tcpNode( tcpNodeNode, configFiles::fileID::main_file );
    tcpNode.startMainLoop();
}