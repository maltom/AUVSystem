#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "PositioningSLAM.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "PositioningSLAM" );

	std::shared_ptr< ros::NodeHandle > positioningSLAMNode = std::make_shared< ros::NodeHandle >();

	PositioningSLAM positioningSLAM( positioningSLAMNode, configFiles::fileID::main_file );
    positioningSLAM.startMainLoop();
}