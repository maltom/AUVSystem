#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "ROSEnums.h"
#include "UDPNode.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::UDPNode ) );

	std::shared_ptr< ros::NodeHandle > udpNodeNode = std::make_shared< ros::NodeHandle >();

	UDPNode udpNode( udpNodeNode, configFiles::fileID::main_file, AUVROS::NodeIDs::UDPNode );
	udpNode.startMainLoop();
}