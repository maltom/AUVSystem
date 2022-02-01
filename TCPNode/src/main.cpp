#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "ROSEnums.h"
#include "TCPNode.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::TCPNode ) );

	std::shared_ptr< ros::NodeHandle > tcpNodeNode = std::make_shared< ros::NodeHandle >();

	TCPNode tcpNode( tcpNodeNode, configFiles::fileID::main_file, AUVROS::NodeIDs::TCPNode );
	tcpNode.startMainLoop();
}